#!/usr/bin/env python3
"""Viewer for the ZhiWatch ESP32-C3 USB frame stream.

The firmware streams frames over USB Serial/JTAG with this framing
(all integers little-endian, pixels RGB565 big-endian):

    magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 |
    payload_len u32 | crc32 u32

flags bit0: payload is RLE with entries (run_count u8, color_hi u8, color_lo u8);
otherwise the payload is raw width*height RGB565 samples.

The GUI also sends a small host-to-device input packet so the on-screen
virtual joystick (and the keyboard) can drive the firmware's pointer:

    magic "ZWI1" | version u8 | flags u8 | x i8 | y i8 | buttons u8

x/y are -127..127 joystick axes and buttons bit0 is the joystick push button.

Usage:
    tools/zhiwatch_viewer.py                     # GUI, auto port
    tools/zhiwatch_viewer.py --port /dev/tty.usbmodem101 --scale 3
    tools/zhiwatch_viewer.py --headless 10       # validate stream for 10 s
"""

import argparse
import math
import operator
import struct
import sys
import time
import zlib

MAGIC = b"ZWF2"
# magic, w, h, flags, seq, payload_len, crc32
HEADER = struct.Struct("<4sHHHIII")
FLAG_RLE = 1
MAX_DIM = 4096

# Host -> device input protocol (see examples/esp32c3.rs).
INPUT_MAGIC = b"ZWI1"
INPUT_VERSION = 1
INPUT_PACKET = struct.Struct("<4sBBbbB")
INPUT_BUTTON = 1
# Keep in sync with VIRTUAL_INPUT_TIMEOUT_MS in the ESP32-C3 example.
INPUT_DEVICE_TIMEOUT_S = 0.5
INPUT_HEARTBEAT_S = INPUT_DEVICE_TIMEOUT_S / 5
INPUT_MIN_INTERVAL_S = 0.02

# RGB565 -> RGB888 byte expansion tables (used via bytes.translate).
_R = bytes((v & 0xF8) | (v >> 5) for v in range(256))
_GH = bytes(((v & 0x07) << 5) for v in range(256))
_GL = bytes((v >> 5) for v in range(256))
_B = bytes(((v & 0x1F) << 3) | ((v & 0x1F) >> 2) for v in range(256))


def rgb565_to_rgb(payload: bytes) -> bytes:
    hi = payload[0::2]
    lo = payload[1::2]
    r = hi.translate(_R)
    g = bytes(map(operator.or_, hi.translate(_GH), lo.translate(_GL)))
    b = lo.translate(_B)
    rgb = bytearray(len(payload) // 2 * 3)
    rgb[0::3] = r
    rgb[1::3] = g
    rgb[2::3] = b
    return bytes(rgb)


def rle_decode(data: bytes) -> bytes:
    """Decode entries of (run_count u8, color_hi u8, color_lo u8) to RGB565."""
    out = bytearray()
    i = 0
    n = len(data)
    while i + 3 <= n:
        count = data[i]
        if count == 0:
            raise ValueError("zero-length RLE run")
        out += bytes((data[i + 1], data[i + 2])) * count
        i += 3
    if i != n:
        raise ValueError("trailing RLE bytes")
    return bytes(out)


class FrameParser:
    """Incremental parser: feed() bytes, pull() the newest complete frame."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.bad_headers = 0
        self.payload_bytes = 0
        self.seq = None
        self.dropped_bytes = 0
        self.on_log = None  # callback(str) for non-frame bytes (firmware logs)
        self._pending = None  # newest (width, height, rgb) awaiting pull
        self._log_tail = b""

    def _flush_prefix(self, idx: int):
        if idx and self.on_log:
            text = (self._log_tail + bytes(self.buf[:idx])).decode("utf-8", "replace")
            self._log_tail = b""
            if text.strip():
                self.on_log(text)
        elif idx:
            self._log_tail = (self._log_tail + bytes(self.buf[:idx]))[-256:]
        self.dropped_bytes += idx
        del self.buf[:idx]

    def feed(self, data: bytes):
        self.buf.extend(data)
        while True:
            idx = self.buf.find(MAGIC)
            if idx < 0:
                # Keep a possible partial magic at the tail.
                keep = max(len(self.buf) - (len(MAGIC) - 1), 0)
                self._flush_prefix(keep)
                return
            if len(self.buf) < idx + HEADER.size:
                self._flush_prefix(idx)
                return
            magic, w, h, flags, seq, plen, crc = HEADER.unpack_from(self.buf, idx)
            pixels = w * h if 0 < w <= MAX_DIM and 0 < h <= MAX_DIM else 0
            if magic != MAGIC or pixels == 0:
                self.bad_headers += 1
                self._flush_prefix(idx + 1)
                continue
            if flags & FLAG_RLE:
                ok_len = 0 < plen <= pixels * 3
            else:
                ok_len = plen == pixels * 2
            if not ok_len:
                self.bad_headers += 1
                self._flush_prefix(idx + 1)
                continue
            if len(self.buf) < idx + HEADER.size + plen:
                self._flush_prefix(idx)
                return
            payload = bytes(self.buf[idx + HEADER.size : idx + HEADER.size + plen])
            if zlib.crc32(payload) & 0xFFFFFFFF != crc:
                self.crc_errors += 1
                self._flush_prefix(idx + 1)  # resync: skip this magic only
                continue
            del self.buf[: idx + HEADER.size + plen]
            self.payload_bytes += plen
            if flags & FLAG_RLE:
                try:
                    payload = rle_decode(payload)
                except ValueError:
                    self.bad_headers += 1
                    continue
                if len(payload) != pixels * 2:
                    self.bad_headers += 1
                    continue
            self.frames += 1
            self.seq = seq
            self._pending = (w, h, rgb565_to_rgb(payload))

    def pull(self):
        frame, self._pending = self._pending, None
        return frame


AXIS_KEYS = {
    "Up": (0, -1),
    "Down": (0, 1),
    "Left": (-1, 0),
    "Right": (1, 0),
    "KP_Up": (0, -1),
    "KP_Down": (0, 1),
    "KP_Left": (-1, 0),
    "KP_Right": (1, 0),
}
SPACE_KEYS = frozenset(("space", "KP_Space"))
# X11 synthesizes a KeyRelease/KeyPress pair for every auto-repeat. Delay
# clearing a held key so that pair is treated as one continuous press; macOS
# repeats only KeyPress events, so the same delay is harmless there.
KEY_RELEASE_DEBOUNCE_MS = 40


class InputLink:
    """Send coalesced virtual-joystick state to the firmware over USB serial."""

    def __init__(self, ser):
        self.ser = ser
        self.state = (0, 0, False)
        self.sent = None
        self.sent_at = 0.0
        self.last_error = None

    def set_state(self, x, y, button):
        x = max(-127, min(127, int(x)))
        y = max(-127, min(127, int(y)))
        self.state = (x, y, bool(button))

    def send(self, force=False):
        now = time.monotonic()
        x, y, button = self.state
        active = x != 0 or y != 0 or button
        changed = self.state != self.sent
        if not force:
            if not changed and not (active and now - self.sent_at >= INPUT_HEARTBEAT_S):
                return
            if now - self.sent_at < INPUT_MIN_INTERVAL_S:
                return
        packet = INPUT_PACKET.pack(
            INPUT_MAGIC,
            INPUT_VERSION,
            0,
            x,
            y,
            INPUT_BUTTON if button else 0,
        )
        try:
            written = self.ser.write(packet)
        except Exception as exc:  # port vanished / TX buffer full
            self.last_error = str(exc)
            return
        self.last_error = None
        self.sent_at = now
        if written == len(packet):
            self.sent = self.state


class VirtualInput:
    """Joystick state shared by the on-screen pad and the keyboard."""

    def __init__(self, link: InputLink, after, after_cancel, on_change=None):
        self.link = link
        self.after = after
        self.after_cancel = after_cancel
        self.on_change = on_change
        self.pointer = (0, 0)
        self.pointer_active = False
        self.held = set()
        self._button_sources = set()
        self._release_jobs = {}

    @property
    def button(self):
        return bool(self._button_sources)

    def press_button(self, source):
        if source not in self._button_sources:
            self._button_sources.add(source)
            self.publish(urgent=True)

    def release_button(self, source):
        if source in self._button_sources:
            self._button_sources.discard(source)
            self.publish(urgent=True)

    def axis(self):
        if self.pointer_active:
            return self.pointer
        x = 0
        y = 0
        for key in self.held:
            dx, dy = AXIS_KEYS[key]
            x += dx
            y += dy
        x = max(-1, min(1, x)) * 127
        y = max(-1, min(1, y)) * 127
        return (x, y)

    def publish(self, urgent=False):
        x, y = self.axis()
        self.link.set_state(x, y, self.button)
        self.link.send(force=urgent)
        if self.on_change is not None:
            self.on_change(x, y, self.button)

    def _cancel_release(self, key):
        job = self._release_jobs.pop(key, None)
        if job is not None:
            self.after_cancel(job)

    def _schedule_release(self, key, action):
        self._cancel_release(key)

        def fire():
            self._release_jobs.pop(key, None)
            action()
            self.publish(urgent=True)

        self._release_jobs[key] = self.after(KEY_RELEASE_DEBOUNCE_MS, fire)

    def press_key(self, keysym) -> bool:
        if keysym in AXIS_KEYS:
            self._cancel_release(keysym)
            if keysym not in self.held:
                self.held.add(keysym)
                self.publish(urgent=True)
            return True
        if keysym in SPACE_KEYS:
            self._cancel_release(keysym)
            if "space" not in self._button_sources:
                self._button_sources.add("space")
                self.publish(urgent=True)
            return True
        return False

    def release_key(self, keysym) -> bool:
        if keysym in AXIS_KEYS:
            self._schedule_release(keysym, lambda: self.held.discard(keysym))
            return True
        if keysym in SPACE_KEYS:
            self._schedule_release(
                keysym, lambda: self._button_sources.discard("space")
            )
            return True
        return False

    def pointer_press(self, nx, ny):
        self.pointer_active = True
        self.pointer_to(nx, ny, urgent=True)

    def pointer_to(self, nx, ny, urgent=False):
        if not self.pointer_active:
            return
        nx = max(-1.0, min(1.0, nx))
        ny = max(-1.0, min(1.0, ny))
        self.pointer = (int(round(nx * 127)), int(round(ny * 127)))
        self.publish(urgent=urgent)

    def pointer_release(self):
        if not self.pointer_active:
            return
        self.pointer_active = False
        self.pointer = (0, 0)
        self.publish(urgent=True)

    def release_all(self):
        for job in self._release_jobs.values():
            self.after_cancel(job)
        self._release_jobs.clear()
        self.held.clear()
        self.pointer_active = False
        self.pointer = (0, 0)
        self._button_sources.clear()
        self.publish(urgent=True)


def default_port() -> str:
    import glob

    candidates = sorted(glob.glob("/dev/tty.usbmodem*")) + sorted(
        glob.glob("/dev/tty.usbserial-*")
    )
    return candidates[0] if candidates else ""


def open_port(port: str, reset: bool):
    import serial

    if not port:
        raise SystemExit("No serial port found; pass --port")
    # Non-blocking writes keep the GUI responsive if the device stops reading.
    ser = serial.Serial(port, 115_200, timeout=0.02, write_timeout=0)
    if reset:
        # Toggle DTR/RTS to reboot the ESP32-C3 so the stream starts clean.
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        time.sleep(0.1)
        ser.reset_input_buffer()
    return ser


def ppm_header(w: int, h: int) -> bytes:
    return f"P6\n{w} {h}\n255\n".encode()


def run_headless(ser, parser: FrameParser, seconds: int, show_logs: bool):
    parser.on_log = (lambda text: sys.stdout.write(text)) if show_logs else None
    end = time.time() + seconds
    last_report = time.time()
    last_seq = None
    while time.time() < end:
        data = ser.read(65536)
        if data:
            parser.feed(data)
        if parser.pull():
            pass
        now = time.time()
        if now - last_report >= 1.0:
            last_report = now
            avg = parser.payload_bytes // max(parser.frames, 1)
            print(
                f"frames={parser.frames} seq={parser.seq} crc_err={parser.crc_errors} "
                f"bad_hdr={parser.bad_headers} avg_payload={avg}B",
                flush=True,
            )
        if not data:
            time.sleep(0.02)
    ok = parser.frames > 0 and parser.crc_errors == 0
    avg = parser.payload_bytes // max(parser.frames, 1)
    print(
        f"RESULT: frames={parser.frames} crc_errors={parser.crc_errors} "
        f"bad_headers={parser.bad_headers} last_seq={parser.seq} "
        f"avg_payload={avg}B -> {'OK' if ok else 'FAIL'}"
    )
    return 0 if ok else 1


def run_gui(ser, parser: FrameParser, scale: int, title: str, duration=None):
    import tkinter as tk

    parser.on_log = lambda text: log_tail.__setitem__(
        "text", (log_tail["text"] + text)[-300:]
    )
    log_tail = {"text": ""}

    bg = "#1C1C1E"
    fg = "#F2F2F7"
    dim = "#A0A0A8"
    accent = "#0A84FF"
    pad_size = 176
    pad_radius = 66
    knob_radius = 25
    center = pad_size / 2

    root = tk.Tk()
    root.title(title)
    root.configure(bg=bg)

    body = tk.Frame(root, bg=bg)
    body.pack(fill="both", expand=True)
    canvas = tk.Canvas(
        body,
        bg="black",
        highlightthickness=0,
        width=135 * scale,
        height=240 * scale,
    )
    canvas.pack(side="left", fill="both", expand=True)

    panel = tk.Frame(body, bg=bg, padx=12, pady=12)
    panel.pack(side="right", fill="y")

    tk.Label(
        panel, text="VIRTUAL JOYSTICK", bg=bg, fg=fg, font=("Menlo", 11, "bold")
    ).pack(anchor="w")
    pad = tk.Canvas(
        panel,
        width=pad_size,
        height=pad_size,
        bg=bg,
        highlightthickness=0,
        cursor="crosshair",
    )
    pad.pack(pady=(8, 4))
    pad.create_oval(
        center - pad_radius,
        center - pad_radius,
        center + pad_radius,
        center + pad_radius,
        fill="#2C2C2E",
        outline="#8E8E93",
        width=2,
    )
    # Axis ticks make the neutral center easy to find.
    for dx, dy in ((0, -1), (0, 1), (-1, 0), (1, 0)):
        pad.create_line(
            center + dx * (pad_radius - 14),
            center + dy * (pad_radius - 14),
            center + dx * (pad_radius - 5),
            center + dy * (pad_radius - 5),
            fill="#636366",
            width=2,
        )
    pad.create_oval(
        center - knob_radius,
        center - knob_radius,
        center + knob_radius,
        center + knob_radius,
        fill=accent,
        outline="#FFFFFF",
        width=2,
        tags="knob",
    )

    value_label = tk.Label(
        panel, text="x +000  y +000  up", bg=bg, fg=dim, font=("Menlo", 10)
    )
    value_label.pack(anchor="w")

    press = tk.Canvas(
        panel,
        width=pad_size,
        height=46,
        bg=bg,
        highlightthickness=0,
        cursor="hand2",
    )
    press.pack(pady=(12, 4))
    press_box = press.create_rectangle(
        2, 2, pad_size - 2, 44, fill="#3A3A3C", outline="#8E8E93", width=1
    )
    press_text = press.create_text(
        center, 23, text="PRESS", fill=fg, font=("Menlo", 12, "bold")
    )

    hint = tk.Label(
        panel,
        text="Drag pad / arrow keys\nSpace: press\nP: pause  Q: quit",
        bg=bg,
        fg=dim,
        justify="left",
        font=("Menlo", 10),
    )
    hint.pack(anchor="w", pady=(8, 0))

    status = tk.Label(root, anchor="w", font=("Menlo", 11), bg=bg, fg=fg)
    status.pack(fill="x")

    photo_ref = {"img": None}
    image_id = {"id": None}
    image_size = {"w": 0, "h": 0}
    stats = {"frames": 0, "fps": 0.0, "last_t": time.time(), "last_count": 0}
    paused = {"on": False}

    link = InputLink(ser)

    def on_input_change(x, y, button):
        kx = center + x / 127.0 * pad_radius
        ky = center + y / 127.0 * pad_radius
        pad.coords(
            "knob",
            kx - knob_radius,
            ky - knob_radius,
            kx + knob_radius,
            ky + knob_radius,
        )
        pad.itemconfig("knob", fill="#FF9F0A" if button else accent)
        value_label.config(
            text=f"x {x:+04d}  y {y:+04d}  {'DOWN' if button else 'up'}"
        )
        press.itemconfig(press_box, fill=accent if button else "#3A3A3C")
        press.itemconfig(press_text, text="PRESSED" if button else "PRESS")

    vinput = VirtualInput(link, root.after, root.after_cancel, on_input_change)

    def set_image(frame):
        w, h, rgb = frame
        img = tk.PhotoImage(data=ppm_header(w, h) + rgb, format="PPM")
        if scale != 1:
            img = img.zoom(scale, scale)
        photo_ref["img"] = img
        if image_id["id"] is None:
            image_id["id"] = canvas.create_image(0, 0, image=img, anchor="nw")
            image_size["w"], image_size["h"] = w * scale, h * scale
            canvas.config(width=image_size["w"], height=image_size["h"])
        else:
            canvas.itemconfig(image_id["id"], image=img)
            if (w * scale, h * scale) != (image_size["w"], image_size["h"]):
                image_size["w"], image_size["h"] = w * scale, h * scale
                canvas.config(width=image_size["w"], height=image_size["h"])
        stats["frames"] += 1

    def poll():
        try:
            data = ser.read(65536)
        except Exception as exc:  # port vanished
            status.config(text=f"serial error: {exc}")
            return
        if data:
            parser.feed(data)
            # Render only the newest frame to stay latency-friendly.
            frame = parser.pull()
            while True:
                nxt = parser.pull()
                if nxt is None:
                    break
                frame = nxt
            if frame and not paused["on"]:
                set_image(frame)

        # Resend the latest joystick state while it is deflected so the
        # firmware's host-input timeout cannot interrupt a long hold.
        link.send()

        now = time.time()
        if now - stats["last_t"] >= 1.0:
            count = stats["frames"] - stats["last_count"]
            stats["fps"] = count / (now - stats["last_t"])
            stats["last_t"] = now
            stats["last_count"] = stats["frames"]

        extra = ""
        if paused["on"]:
            extra = "| PAUSED "
        if link.last_error:
            extra += f"| input err {link.last_error[:40]} "
        status.config(
            text=(
                f"fps {stats['fps']:.1f} | frames {stats['frames']} "
                f"| seq {parser.seq} | crc err {parser.crc_errors} "
                f"{extra}| {log_tail['text'].strip()[-70:]}"
            )
        )
        root.after(5, poll)

    def pad_normalized(event):
        dx = (event.x - center) / pad_radius
        dy = (event.y - center) / pad_radius
        distance = math.hypot(dx, dy)
        if distance > 1.0:
            dx /= distance
            dy /= distance
        return dx, dy

    def on_pad_press(event):
        nx, ny = pad_normalized(event)
        vinput.pointer_press(nx, ny)
        return "break"

    def on_pad_drag(event):
        nx, ny = pad_normalized(event)
        vinput.pointer_to(nx, ny)
        return "break"

    def on_pad_release(event=None):
        vinput.pointer_release()
        return "break"

    def on_press_down(event):
        vinput.press_button("mouse")
        return "break"

    def on_press_up(event=None):
        vinput.release_button("mouse")
        return "break"

    def on_any_button_release(event=None):
        # Fallback for a drag that ends outside the pad/button (or when the
        # window manager does not deliver the release to the grabbed widget).
        vinput.pointer_release()
        vinput.release_button("mouse")

    def on_key_press(event):
        if vinput.press_key(event.keysym):
            return "break"
        if event.keysym in ("Escape", "q", "Q"):
            close()
        elif event.keysym in ("p", "P"):
            paused["on"] = not paused["on"]

    def on_key_release(event):
        if vinput.release_key(event.keysym):
            return "break"

    def on_focus_out(event=None):
        vinput.release_all()

    def close():
        vinput.release_all()
        link.send(force=True)
        root.destroy()

    pad.bind("<ButtonPress-1>", on_pad_press)
    pad.bind("<B1-Motion>", on_pad_drag)
    pad.bind("<ButtonRelease-1>", on_pad_release)
    press.bind("<ButtonPress-1>", on_press_down)
    press.bind("<ButtonRelease-1>", on_press_up)
    root.bind("<ButtonRelease-1>", on_any_button_release)
    root.bind("<KeyPress>", on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.bind("<FocusOut>", on_focus_out)
    root.protocol("WM_DELETE_WINDOW", close)

    if duration is not None:
        root.after(int(duration * 1000), close)
    root.after(5, poll)
    root.after(100, root.focus_force)
    root.mainloop()
    print(
        f"GUI done: frames_shown={stats['frames']} crc_err={parser.crc_errors} "
        f"last_seq={parser.seq}"
    )


def run_smoke(seconds: float) -> int:
    """Offline GUI sanity check: build a synthetic PPM frame without a device."""
    import tkinter as tk

    root = tk.Tk()
    root.withdraw()
    w, h = 135, 240
    rgb = bytearray(w * h * 3)
    for y in range(h):
        for x in range(w):
            i = (y * w + x) * 3
            rgb[i] = (x * 2) % 256
            rgb[i + 1] = (y) % 256
            rgb[i + 2] = 128
    img = tk.PhotoImage(data=ppm_header(w, h) + bytes(rgb), format="PPM")
    img = img.zoom(3, 3)
    label = tk.Label(root, image=img)
    label.pack()
    root.after(int(seconds * 1000), root.destroy)
    root.mainloop()
    print("SMOKE OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=default_port(), help="serial port (default: auto)")
    ap.add_argument("--scale", type=int, default=3, help="integer zoom factor (default 3)")
    ap.add_argument("--no-reset", action="store_true", help="do not reset the board on start")
    ap.add_argument("--headless", type=int, metavar="SECONDS", help="validate the stream without a GUI")
    ap.add_argument("--smoke", type=float, metavar="SECONDS", help="offline GUI render test, no device")
    ap.add_argument("--duration", type=float, metavar="SECONDS", help="close the GUI after N seconds")
    ap.add_argument("--show-logs", action="store_true", help="print firmware log lines (headless)")
    args = ap.parse_args()

    if args.smoke is not None:
        sys.exit(run_smoke(args.smoke))

    ser = open_port(args.port, reset=not args.no_reset)
    parser = FrameParser()
    try:
        if args.headless:
            sys.exit(run_headless(ser, parser, args.headless, args.show_logs))
        run_gui(ser, parser, max(args.scale, 1), f"ZhiWatch viewer — {args.port}", args.duration)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
