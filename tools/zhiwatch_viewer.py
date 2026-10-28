#!/usr/bin/env python3
"""Viewer for the ZhiWatch ESP32-C3 USB frame stream.

The firmware streams frames over USB Serial/JTAG with this framing
(all integers little-endian, pixels RGB565 big-endian):

    magic "ZWF2" | width u16 | height u16 | flags u16 | seq u32 |
    payload_len u32 | crc32 u32

flags bit0: payload is RLE with entries (run_count u8, color_hi u8, color_lo u8);
otherwise the payload is raw width*height RGB565 samples.

Usage:
    tools/zhiwatch_viewer.py                     # GUI, auto port
    tools/zhiwatch_viewer.py --port /dev/tty.usbmodem101 --scale 3
    tools/zhiwatch_viewer.py --headless 10       # validate stream for 10 s
"""

import argparse
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
    ser = serial.Serial(port, 115_200, timeout=0.02)
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

    root = tk.Tk()
    root.title(title)
    canvas = tk.Canvas(root, bg="black", highlightthickness=0)
    canvas.pack(fill="both", expand=True)
    status = tk.Label(root, anchor="w", font=("Menlo", 11))
    status.pack(fill="x")

    photo_ref = {"img": None}
    image_id = {"id": None}
    stats = {"frames": 0, "fps": 0.0, "last_t": time.time(), "last_count": 0}
    paused = {"on": False}

    def set_image(frame):
        w, h, rgb = frame
        img = tk.PhotoImage(data=ppm_header(w, h) + rgb, format="PPM")
        if scale != 1:
            img = img.zoom(scale, scale)
        photo_ref["img"] = img
        if image_id["id"] is None:
            image_id["id"] = canvas.create_image(0, 0, image=img, anchor="nw")
            root.geometry(f"{w * scale}x{h * scale}")
        else:
            canvas.itemconfig(image_id["id"], image=img)
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

        now = time.time()
        if now - stats["last_t"] >= 1.0:
            count = stats["frames"] - stats["last_count"]
            stats["fps"] = count / (now - stats["last_t"])
            stats["last_t"] = now
            stats["last_count"] = stats["frames"]

        status.config(
            text=(
                f"fps {stats['fps']:.1f} | frames {stats['frames']} "
                f"| seq {parser.seq} | crc err {parser.crc_errors} "
                f"| {'PAUSED' if paused['on'] else ''} {log_tail['text'].strip()[-80:]}"
            )
        )
        root.after(5, poll)

    def on_key(event):
        if event.keysym in ("Escape", "q"):
            root.destroy()
        elif event.keysym == "space":
            paused["on"] = not paused["on"]

    root.bind("<Key>", on_key)
    if duration is not None:
        root.after(int(duration * 1000), root.destroy)
    root.after(5, poll)
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
