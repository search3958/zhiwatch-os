use super::*;

/// Match the normal window server's cubic-bezier(0, 0, 0, 1) scroll curve.
/// Solving x=s^3 with a short binary search keeps this identical without
/// adding a curve table to the Xiao image.
fn decelerate_scroll(t: f32) -> f32 {
    if t <= 0.0 {
        return 0.0;
    }
    if t >= 1.0 {
        return 1.0;
    }
    let mut low = 0.0f32;
    let mut high = 1.0f32;
    for _ in 0..10 {
        let s = (low + high) * 0.5;
        if s * s * s < t {
            low = s;
        } else {
            high = s;
        }
    }
    let s = (low + high) * 0.5;
    s * s * (3.0 - 2.0 * s)
}

impl Warp4Engine {
    pub fn new(app_name: &str) -> Self {
        Self::from_archive(Warp4Archive::open(app_name))
    }

    pub fn new_embedded(name: &str, sources: &[(&'static str, &'static str)]) -> Self {
        Self::from_archive(Warp4Archive::from_embedded(name, sources))
    }

    pub(crate) fn from_archive(archive: Warp4Archive) -> Self {
        let cfg = archive.read_text("config.ini");
        let screen = ini(&cfg, "screen").unwrap_or_else(|| "main".into());
        let title = ini(&cfg, "name").unwrap_or_else(|| "Warp 4".into());
        let mut this = Self {
            origin: archive.app_name().to_string(),
            archive,
            title,
            screen,
            nodes: Vec::new(),
            roots: Vec::new(),
            fixed_subtree: Vec::new(),
            script: Script::default(),
            state: Vec::new(),
            focused: None,
            keyboard_focus: None,
            focusable_nodes: Vec::with_capacity(MAX_NODES),
            focus_positions: Vec::with_capacity(MAX_NODES),
            focus_cache_valid: false,
            hovered: None,
            pressed: None,
            keyboard_press_until_ns: None,
            spinner_open: None,
            spinner_fade: None,
            width: 0,
            height: 0,
            chrome_height: title_bar_h(),
            scroll: 0,
            scroll_target: 0,
            scroll_start: 0,
            scroll_subpixel: 0,
            scroll_target_subpixel: 0,
            scroll_start_subpixel: 0,
            scroll_started_ns: None,
            content_height: 0,
            last_command: None,
            dirty: true,
            layout_dirty: true,
            damage_regions: [(0, 0, 0, 0); MAX_DAMAGE_REGIONS],
            damage_count: 0,
            damage_full: true,
            now_ns: 0,
            wait_until_ns: None,
            pending: Vec::new(),
            break_requested: false,
            flip_elapsed_ns: 0,
            last_tick_ns: None,
            control_animations: Vec::with_capacity(64),
            last_animation_frame: None,
            caret_phase: None,
            transition_elapsed_ns: None,
            last_clicked_id: None,
            runtime_fps: 0,
            runtime_windows: 0,
            runtime_keys: 0,
            runtime_mouse: 0,
        };
        this.load_screen();
        this
    }

    pub(crate) fn preload_text_cache(&self) {
        if !bdf_font::is_available() {
            return;
        }
        let mut texts = Vec::new();
        for node in &self.nodes {
            for key in ["text", "hint", "textOn", "textOff"] {
                let text = node.attr(key);
                if !text.is_empty() {
                    texts.push(text);
                }
            }
        }
        bdf_font::preload_texts(&texts);
    }

    pub fn set_origin(&mut self, name: &str) {
        self.origin = name.into();
    }

    /// Removes the window-manager title-bar coordinate space for surfaces
    /// embedded directly into an OS-owned card or overlay.
    pub fn set_chrome_visible(&mut self, visible: bool) {
        let next = if visible { title_bar_h() } else { 0 };
        if self.chrome_height != next {
            self.chrome_height = next;
            self.scroll = 0;
            self.scroll_target = 0;
            self.scroll_start = 0;
            self.scroll_subpixel = 0;
            self.scroll_target_subpixel = 0;
            self.scroll_start_subpixel = 0;
            self.scroll_started_ns = None;
            self.dirty = true;
            self.layout_dirty = true;
            self.damage_all();
        }
    }
    pub fn origin(&self) -> &str {
        &self.origin
    }
    pub fn title(&self) -> &str {
        &self.title
    }
    pub fn set_scroll(&mut self, scroll: i32) {
        let max = self
            .content_height
            .saturating_sub(self.height + self.chrome_height);
        let next = scroll.max(0).min(max.max(0));
        if is_xiao() {
            let next_subpixel = next.saturating_mul(3);
            let changed = self.scroll_target_subpixel != next_subpixel
                || self.scroll_subpixel != next_subpixel;
            if self.scroll_target_subpixel != next_subpixel {
                // Queue a new destination while an animation is in flight.
                // Restarting from the current frame made repeated arrow keys
                // visibly stutter and differed from the normal window server.
                if self.scroll_subpixel == self.scroll_target_subpixel {
                    self.scroll_start = self.scroll;
                    self.scroll_start_subpixel = self.scroll_subpixel;
                    self.scroll_started_ns = None;
                }
                self.scroll_target = next;
                self.scroll_target_subpixel = next_subpixel;
            }
            if changed {
                if let Some(idx) = self.spinner_open {
                    self.close_spinner(idx);
                }
                self.hovered = None;
                self.dirty = true;
                self.damage_all();
            }
        } else if self.scroll != next {
            self.scroll = next;
            self.scroll_target = next;
            self.scroll_start = next;
            self.scroll_subpixel = next.saturating_mul(3);
            self.scroll_target_subpixel = self.scroll_subpixel;
            self.scroll_start_subpixel = self.scroll_subpixel;
            self.scroll_started_ns = None;
            if let Some(idx) = self.spinner_open {
                self.close_spinner(idx);
            }
            self.hovered = None;
            self.dirty = true;
            self.damage_all();
        }
    }

    /// Move the active document viewport by a fixed keyboard step.  Xiao uses
    /// this directly for UEFI arrow-key events because those events have no
    /// printable byte to pass through `handle_key`.
    pub fn scroll_step(&self) -> i32 {
        config::get_i32("ui-theme/window/scroll_speed", 30).max(1)
    }

    pub fn scroll_by(&mut self, delta: i32) -> bool {
        let before = if is_xiao() {
            self.scroll_target
        } else {
            self.scroll
        };
        let base = if is_xiao() {
            self.scroll_target
        } else {
            self.scroll
        };
        self.set_scroll(base.saturating_add(delta));
        self.scroll_target != before || self.scroll != before
    }

    /// Return the active document offset in layer pixels.
    ///
    /// Xiao feeds pointer coordinates in viewport space, while the normal
    /// window server feeds Warp4 document coordinates. The kiosk uses this
    /// value to convert its pointer event without duplicating scroll state.
    pub fn scroll_position(&self) -> i32 {
        self.scroll
    }
    pub fn is_animating(&self) -> bool {
        !self.control_animations.is_empty()
            || (is_xiao() && self.scroll_subpixel != self.scroll_target_subpixel)
            || self.transition_elapsed_ns.is_some()
    }
    /// True when the view tree or its visual state changed since the last
    /// paint. Callers can use this to keep a cheap cursor overlay separate
    /// from an expensive full-frame render.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }
    /// Mark the whole layer for repaint. Used whenever content moves (scroll,
    /// layout, screen change) or when a script action may touch arbitrary
    /// nodes, so a partial repaint cannot be trusted.
    pub(crate) fn damage_all(&mut self) {
        self.damage_full = true;
        self.damage_count = 0;
    }

    /// Union a screen-space rectangle `(x, y, w, h)` into the damaged set.
    /// Touching rectangles are coalesced; distant controls remain independent.
    pub(crate) fn damage_rect(&mut self, x: i32, y: i32, w: i32, h: i32) {
        if self.damage_full || w <= 0 || h <= 0 || self.width <= 0 || self.height <= 0 {
            return;
        }

        let x0 = x.max(0);
        let y0 = y.max(0);
        let x1 = x.saturating_add(w).min(self.width);
        let y1 = y.saturating_add(h).min(self.height);
        if x0 >= x1 || y0 >= y1 {
            return;
        }
        let mut merged = (x0, y0, x1 - x0, y1 - y0);

        // Restart after every merge because the enlarged rectangle may now
        // touch a region that was checked earlier.
        let mut i = 0;
        while i < self.damage_count {
            let other = self.damage_regions[i];
            let merged_right = merged.0.saturating_add(merged.2);
            let merged_bottom = merged.1.saturating_add(merged.3);
            let other_right = other.0.saturating_add(other.2);
            let other_bottom = other.1.saturating_add(other.3);
            let touches = merged.0 <= other_right
                && merged_right >= other.0
                && merged.1 <= other_bottom
                && merged_bottom >= other.1;
            if touches {
                let nx = merged.0.min(other.0);
                let ny = merged.1.min(other.1);
                let nr = merged_right.max(other_right);
                let nb = merged_bottom.max(other_bottom);
                merged = (nx, ny, nr - nx, nb - ny);
                self.damage_count -= 1;
                self.damage_regions[i] = self.damage_regions[self.damage_count];
                i = 0;
            } else {
                i += 1;
            }
        }

        if self.damage_count == MAX_DAMAGE_REGIONS {
            // A bounded fixed array avoids heap traffic during input. If an
            // unusually complex frame exceeds it, a full repaint is safest.
            self.damage_all();
            return;
        }
        self.damage_regions[self.damage_count] = merged;
        self.damage_count += 1;
    }

    /// Mark a single node's screen rectangle plus margins for focus rings and
    /// antialiased edges.
    pub(crate) fn damage_node(&mut self, idx: usize) {
        if self.damage_full {
            return;
        }
        let Some((x, y, w, h)) = self.nodes.get(idx).and_then(|node| {
            node.visible().then_some((node.x, node.y, node.w, node.h))
        }) else {
            return;
        };
        let margin = 2;
        let screen_y = self.node_screen_y(idx);
        self.damage_rect(
            x.saturating_sub(margin),
            screen_y.saturating_sub(margin),
            w.max(1).saturating_add(2 * margin),
            h.max(1).saturating_add(2 * margin),
        );
    }

    /// Bounding rectangle for legacy callers. Embedded rendering should use
    /// `take_damage_rects` to preserve disjoint areas.
    pub fn window_damage(&self) -> Option<(i32, i32, i32, i32)> {
        if self.damage_full || self.damage_count == 0 {
            return None;
        }
        let first = self.damage_regions[0];
        let mut x0 = first.0;
        let mut y0 = first.1;
        let mut x1 = first.0.saturating_add(first.2);
        let mut y1 = first.1.saturating_add(first.3);
        for rect in self.damage_regions.iter().take(self.damage_count).skip(1) {
            x0 = x0.min(rect.0);
            y0 = y0.min(rect.1);
            x1 = x1.max(rect.0.saturating_add(rect.2));
            y1 = y1.max(rect.1.saturating_add(rect.3));
        }
        Some((x0, y0, x1 - x0, y1 - y0))
    }

    /// Copy and clear independent damaged rectangles. `None` means a full
    /// repaint is required; `Some(0)` means no pixels need repainting.
    pub fn take_damage_rects(
        &mut self,
        output: &mut [(i32, i32, i32, i32)],
    ) -> Option<usize> {
        if self.damage_full {
            self.damage_full = false;
            self.damage_count = 0;
            return None;
        }
        if self.damage_count > output.len() {
            self.damage_full = true;
            self.damage_count = 0;
            return None;
        }
        let count = self.damage_count;
        output[..count].copy_from_slice(&self.damage_regions[..count]);
        self.damage_count = 0;
        Some(count)
    }

    /// Compatibility helper for band-based clients that only track rows.
    pub fn take_damage_rows(&mut self) -> Option<(i32, i32)> {
        let mut rects = [(0, 0, 0, 0); MAX_DAMAGE_REGIONS];
        let count = self.take_damage_rects(&mut rects)?;
        if count == 0 {
            return None;
        }
        let mut y0 = rects[0].1;
        let mut y1 = y0.saturating_add(rects[0].3);
        for rect in rects.iter().take(count).skip(1) {
            y0 = y0.min(rect.1);
            y1 = y1.max(rect.1.saturating_add(rect.3));
        }
        Some((y0, y1))
    }

    pub fn clear_damage(&mut self) {
        self.damage_full = false;
        self.damage_count = 0;
    }
    pub fn has_focused_input(&self) -> bool {
        self.focused.is_some()
    }

    /// Rebuild the compact focus candidate list after layout/visibility
    /// changes. Navigation itself then performs no heap allocation and skips
    /// non-interactive nodes entirely.
    pub(crate) fn rebuild_focusable_nodes(&mut self) {
        self.focusable_nodes.clear();
        self.focus_positions.clear();
        self.focus_positions.resize(self.nodes.len(), usize::MAX);
        self.focus_cache_valid = true;
        for idx in 0..self.nodes.len() {
            let eligible = {
                let node = &self.nodes[idx];
                node.visible() && self.active_child(idx) && is_keyboard_focusable(node)
            };
            if eligible {
                self.focus_positions[idx] = self.focusable_nodes.len();
                self.focusable_nodes.push(idx);
            }
        }
    }

    /// Synchronize focus candidates before input consumes them. This path is
    /// normally a no-op; it only lays out when a visibility/tree mutation is
    /// pending, preventing one-frame navigation through stale controls.
    pub(crate) fn ensure_focus_cache(&mut self) {
        if self.layout_dirty && self.width > 0 && self.height > 0 {
            self.update(self.width, self.height);
        } else if !self.focus_cache_valid {
            self.rebuild_focusable_nodes();
        }
    }

    /// Move keyboard focus between interactive controls without requiring
    /// every Warp application to implement its own focus script.  Horizontal
    /// arrows are intentionally used here by Xiao; vertical arrows remain
    /// document scrolling keys.
    pub fn focus_direction(&mut self, direction: i32) -> bool {
        self.ensure_focus_cache();
        let focusable = &self.focusable_nodes;
        if focusable.is_empty() {
            return false;
        }
        let current_position = self.keyboard_focus.and_then(|current| {
            self.focus_positions
                .get(current)
                .copied()
                .filter(|position| *position != usize::MAX)
        });
        let next = match current_position {
            Some(position) => {
                let next = position as i32 + if direction < 0 { -1 } else { 1 };
                if next < 0 {
                    focusable[focusable.len() - 1]
                } else {
                    focusable[next as usize % focusable.len()]
                }
            }
            None if direction < 0 => focusable[focusable.len() - 1],
            None => focusable[0],
        };
        let changed = self.keyboard_focus != Some(next);
        let old_focus = self.keyboard_focus;
        let previous_scroll = if is_xiao() {
            self.scroll_target_subpixel
        } else {
            self.scroll_target
        };
        self.keyboard_focus = Some(next);
        self.ensure_keyboard_focus_visible(next);
        let scrolled = if is_xiao() {
            self.scroll_target_subpixel != previous_scroll
        } else {
            self.scroll_target != previous_scroll
        };
        if changed {
            if let Some(idx) = old_focus {
                self.damage_node(idx);
            }
            self.damage_node(next);
        }
        let moved = changed || scrolled;
        if moved {
            self.dirty = true;
        }
        moved
    }

    /// Screen rectangle (x, y, w, h) of the keyboard-focused control.
    pub fn keyboard_focus_rect(&self) -> Option<(i32, i32, i32, i32)> {
        let idx = self.keyboard_focus?;
        let node = self.nodes.get(idx)?;
        Some((node.x, self.node_screen_y(idx), node.w.max(1), node.h.max(1)))
    }

    /// Move focus to the nearest focusable control in one of four directions,
    /// like an arrow key. `dx` and `dy` are -1, 0 or 1 in screen coordinates
    /// (+x right, +y down); only one axis should be non-zero. A candidate must
    /// lie ahead of the current control. Off-axis distance counts double, so a
    /// neighbour straight ahead wins over a nearer one at an angle. Returns
    /// false when nothing lies in that direction, and focus stays put.
    pub fn focus_spatial(&mut self, dx: i32, dy: i32) -> bool {
        if dx == 0 && dy == 0 {
            return false;
        }
        self.ensure_focus_cache();
        let Some(current) = self.keyboard_focus else {
            return self.focus_direction(if dx < 0 || dy < 0 { -1 } else { 1 });
        };
        if self
            .focus_positions
            .get(current)
            .copied()
            .unwrap_or(usize::MAX)
            == usize::MAX
        {
            self.keyboard_focus = None;
            return self.focus_direction(if dx < 0 || dy < 0 { -1 } else { 1 });
        }
        let Some((cx, cy, cw, ch)) = self.keyboard_focus_rect() else {
            return false;
        };
        // Doubled centers keep everything in integers.
        let (ccx, ccy) = (cx as i64 * 2 + cw as i64, cy as i64 * 2 + ch as i64);
        let mut best: Option<(i64, usize)> = None;
        for &idx in &self.focusable_nodes {
            if idx == current {
                continue;
            }
            let node = &self.nodes[idx];
            let ex = node.x as i64 * 2 + node.w.max(1) as i64;
            let ey = self.node_screen_y(idx) as i64 * 2 + node.h.max(1) as i64;
            let vx = ex - ccx;
            let vy = ey - ccy;
            let (ahead, off) = if dx != 0 {
                (vx * dx.signum() as i64, vy.abs())
            } else {
                (vy * dy.signum() as i64, vx.abs())
            };
            if ahead <= 0 {
                continue;
            }
            let score = ahead + 2 * off;
            if best.map_or(true, |(s, _)| score < s) {
                best = Some((score, idx));
            }
        }
        let Some((_, next)) = best else {
            return false;
        };
        self.damage_node(current);
        self.keyboard_focus = Some(next);
        self.ensure_keyboard_focus_visible(next);
        self.damage_node(next);
        self.dirty = true;
        true
    }

    /// Activate the currently focused button as if Space had clicked it.
    /// Text fields are left to the normal text-input path.
    pub fn activate_focused(&mut self) -> bool {
        self.ensure_focus_cache();
        let Some(idx) = self.keyboard_focus else {
            return false;
        };
        if self
            .focus_positions
            .get(idx)
            .copied()
            .unwrap_or(usize::MAX)
            == usize::MAX
            || !self
                .nodes
                .get(idx)
                .is_some_and(|node| is_keyboard_focusable(node) && !is_text_input(node))
        {
            return false;
        }
        let center_x = self.nodes[idx].x + self.nodes[idx].w.max(1) / 2;
        self.activate_index(idx, true, center_x);
        true
    }

    /// Begin the shared classic-Mac-style zoom reveal used when Xiao enters a
    /// new screen.  The overlay is painted by this engine, so applications do
    /// not need transition elements in their XML.
    pub fn start_transition(&mut self) {
        if is_xiao() {
            self.transition_elapsed_ns = Some(0);
            self.dirty = true;
            self.damage_all();
        }
    }
    pub fn set_runtime_metrics(&mut self, fps: u32, windows: usize, keys: u32, mouse: u32) {
        self.runtime_fps = fps;
        self.runtime_windows = windows;
        self.runtime_keys = keys;
        self.runtime_mouse = mouse;
    }
    pub fn hovered_node(&self) -> Option<usize> {
        self.hovered
    }
    pub fn clear_hover(&mut self) {
        if let Some(idx) = self.hovered.take() {
            self.damage_node(idx);
            self.dirty = true;
        }
    }
    pub fn refresh_config(&mut self) {
        self.dirty = true;
        self.damage_all();
    }
    pub fn take_command(&mut self) -> Option<String> {
        self.last_command.take()
    }
    pub fn take_scroll_request(&mut self) -> Option<i32> {
        None
    }
    pub fn tick(&mut self, now_ns: u64) -> bool {
        self.now_ns = now_ns;
        let delta = self
            .last_tick_ns
            .replace(now_ns)
            .map(|previous| now_ns.saturating_sub(previous).min(100_000_000))
            .unwrap_or(0);
        self.flip_elapsed_ns = self.flip_elapsed_ns.saturating_add(delta);

        const ANIMATION_FRAME_NS: u64 = 8_000_000;
        let animation_frame = now_ns / ANIMATION_FRAME_NS;
        let animation_frame_due = self.last_animation_frame != Some(animation_frame);
        if animation_frame_due {
            self.last_animation_frame = Some(animation_frame);
        }

        let mut changed = false;
        let mut transition_finished = false;
        if let Some(elapsed) = self.transition_elapsed_ns.as_mut() {
            *elapsed = elapsed.saturating_add(delta.max(1_000_000));
            transition_finished = *elapsed >= XIAO_TRANSITION_NS;
            if animation_frame_due || transition_finished {
                changed = true;
                self.dirty = true;
                self.damage_all();
            }
        }
        if transition_finished {
            self.transition_elapsed_ns = None;
        }
        if is_xiao() && self.scroll_subpixel != self.scroll_target_subpixel {
            // Use the same timestamp-origin model as the normal window
            // server. Accumulating frame deltas made input bursts and missed
            // timer wakeups change the effective scroll speed.
            let started = *self
                .scroll_started_ns
                .get_or_insert(self.now_ns.saturating_sub(1_000_000));
            let elapsed = self.now_ns.saturating_sub(started);
            let t = (elapsed as f32 / XIAO_SCROLL_ANIMATION_NS as f32).clamp(0.0, 1.0);
            let eased = decelerate_scroll(t);
            let distance = self.scroll_target_subpixel - self.scroll_start_subpixel;
            let next_subpixel = if t >= 1.0 {
                self.scroll_target_subpixel
            } else {
                let amount = distance as f32 * eased;
                // Round to the nearest third-pixel. Truncation left a final
                // fraction pending and produced a visible last-pixel jump.
                let rounded = if amount >= 0.0 {
                    (amount + 0.5) as i32
                } else {
                    (amount - 0.5) as i32
                };
                self.scroll_start_subpixel.saturating_add(rounded)
            };
            if self.scroll_subpixel != next_subpixel {
                self.scroll_subpixel = next_subpixel;
                self.scroll = next_subpixel.div_euclid(3);
                self.dirty = true;
                self.damage_all();
                changed = true;
            }
            if t >= 1.0 {
                self.scroll_start = self.scroll_target;
                self.scroll_start_subpixel = self.scroll_target_subpixel;
                self.scroll_started_ns = None;
            }
        }
        let mut flip_interval_ns: Option<u64> = None;
        for node in &self.nodes {
            if (node.is("ViewFlipper") || node.is("ViewAnimator"))
                && truth(node.attr("autoStart"))
                && node.children.len() > 1
            {
                let interval_ms = parse_i32(node.attr("flipInterval")).max(16) as u64;
                flip_interval_ns =
                    Some(flip_interval_ns.map_or(interval_ms * 1_000_000, |current| {
                        current.min(interval_ms * 1_000_000)
                    }));
            }
        }
        if let Some(interval_ns) = flip_interval_ns {
            if self.flip_elapsed_ns >= interval_ns {
                self.flip_elapsed_ns %= interval_ns;
                if let Some(idx) = self.nodes.iter().position(|node| {
                    (node.is("ViewFlipper") || node.is("ViewAnimator"))
                        && truth(node.attr("autoStart"))
                        && node.children.len() > 1
                }) {
                    let count = self.nodes[idx].children.len() as i32;
                    let next =
                        (parse_i32(self.nodes[idx].attr("displayedChild")).max(0) + 1) % count;
                    set_attr(&mut self.nodes[idx], "displayedChild", &next.to_string());
                    self.rebuild_focusable_nodes();
                    self.layout_dirty = true;
                    self.hovered = None;
                    self.pressed = None;
                    self.dirty = true;
                    self.damage_all();
                    changed = true;
                }
            }
        }

        if let Some(until) = self.wait_until_ns {
            if now_ns >= until {
                self.wait_until_ns = None;
                if !self.pending.is_empty() {
                    let pending = core::mem::take(&mut self.pending);
                    self.execute(&pending);
                    self.damage_all();
                    changed = true;
                }
            }
        }
        if let Some(until) = self.keyboard_press_until_ns {
            if now_ns >= until {
                self.keyboard_press_until_ns = None;
                if let Some(idx) = self.pressed.take() {
                    self.damage_node(idx);
                }
                self.dirty = true;
                changed = true;
            }
        }
        // Rasterize animated controls at the display cadence, not on every
        // 4 ms input poll. The final sample is always invalidated even when an
        // animation ends between display frames.
        if !self.control_animations.is_empty() {
            let mut index = 0;
            while index < self.control_animations.len() {
                let animation = self.control_animations[index];
                let finished = now_ns.saturating_sub(animation.started_ns)
                    >= animation.duration_ns;
                if animation_frame_due || finished {
                    self.damage_node(animation.idx);
                    self.dirty = true;
                    changed = true;
                }
                if finished {
                    self.control_animations.swap_remove(index);
                } else {
                    index += 1;
                }
            }
        }
        // Keep a focused text field alive only when its 500 ms caret phase
        // changes. Previously this invalidated the field on every input poll.
        if let Some(idx) = self.focused {
            let phase = now_ns / 500_000_000;
            if self.caret_phase != Some(phase) {
                self.caret_phase = Some(phase);
                self.damage_node(idx);
                self.dirty = true;
                changed = true;
            }
        } else {
            self.caret_phase = None;
        }
        if let Some(fade) = self.spinner_fade {
            let finished = now_ns.saturating_sub(fade.started_ns) >= 140_000_000;
            if animation_frame_due || finished {
                // The fading popup overlays only its own rectangle; repaint
                // that region plus the spinner control at display cadence.
                let (px, py, pw, ph) = self.spinner_popup_rect(fade.idx);
                self.damage_rect(
                    px.saturating_sub(2),
                    py.saturating_sub(2),
                    pw.saturating_add(4),
                    ph.saturating_add(4),
                );
                self.damage_node(fade.idx);
                self.dirty = true;
                changed = true;
            }
            if finished {
                self.spinner_fade = None;
            }
        }
        changed
    }

    pub fn set_screen(&mut self, screen: &str) {
        let screen = screen
            .trim()
            .trim_end_matches(".w4u")
            .trim_end_matches(".w3u");
        if self.screen != screen {
            self.screen = screen.into();
            self.load_screen();
            self.start_transition();
        }
    }

    pub(crate) fn load_screen(&mut self) {
        // Rebuilding the native view tree must not reset a running app. Keep
        // script variables and user-editable view values across navigation or
        // any other screen reload.
        let previous_state = self.state.clone();
        let mut previous_attrs: Vec<(String, String, String)> = Vec::new();
        for node in &self.nodes {
            let id = node.id();
            if id.is_empty() {
                continue;
            }
            for key in [
                "text",
                "checked",
                "progress",
                "rating",
                "selectedIndex",
                "value",
                "visibility",
                "enabled",
            ] {
                let value = node.attr(key);
                if !value.is_empty() {
                    previous_attrs.push((id.to_string(), key.into(), value.into()));
                }
            }
        }

        self.nodes.clear();
        self.roots.clear();
        self.fixed_subtree.clear();
        self.focusable_nodes.clear();
        self.focus_positions.clear();
        self.focus_cache_valid = false;
        self.focused = None;
        self.keyboard_focus = None;
        self.hovered = None;
        self.pressed = None;
        self.keyboard_press_until_ns = None;
        self.spinner_open = None;
        self.spinner_fade = None;
        self.control_animations.clear();
        self.last_animation_frame = None;
        self.flip_elapsed_ns = 0;
        self.last_tick_ns = None;
        self.transition_elapsed_ns = None;
        self.scroll = 0;
        self.scroll_target = 0;
        self.scroll_start = 0;
        self.scroll_subpixel = 0;
        self.scroll_target_subpixel = 0;
        self.scroll_start_subpixel = 0;
        self.scroll_started_ns = None;
        self.wait_until_ns = None;
        self.pending.clear();
        let mut layout = self.archive.read_text_cow(&format!("{}.w4u", self.screen));
        if layout.is_empty() {
            layout = self.archive.read_text_cow(&format!("{}.w3u", self.screen));
        }
        // A few early Warp 4 examples were distributed as plain XML instead
        // of using the `.w4u` member name. They are the same native view-tree
        // format; accepting the suffix here keeps those archives native while
        // avoiding an HTML conversion path.
        if layout.is_empty() {
            layout = self.archive.read_text_cow(&format!("{}.xml", self.screen));
        }
        if layout.is_empty() && self.screen != "main" {
            layout = self.archive.read_text_cow("main.xml");
        }
        let mut parser = XmlParser::new(&layout);
        if let Some(root) = parser.parse_element(None, &mut self.nodes) {
            self.roots.push(root);
        }
        let mut source = self.archive.read_text_cow(&format!("{}.w4s", self.screen));
        if source.is_empty() {
            source = self.archive.read_text_cow(&format!("{}.w3s", self.screen));
        }
        self.script = parse_script(&source);
        // Init actions are only needed once. Taking them out of the script
        // lets their argument strings be freed instead of kept for the whole
        // session.
        let init = core::mem::take(&mut self.script.init);
        self.execute(&init);
        drop(init);
        // Init provides defaults on the first load. When this is a reload,
        // restore the values owned by the already-running application.
        for (key, value) in previous_state {
            self.set_state(&key, &value);
        }
        for (id, key, value) in previous_attrs {
            if let Some(idx) = self.find(&id) {
                set_attr(&mut self.nodes[idx], &key, &value);
            }
        }
        let chrome_mode = self.has_explicit_scroll();
        self.rebuild_fixed_subtree(chrome_mode);
        self.refresh_visibility();
        self.dirty = true;
        self.layout_dirty = true;
        self.preload_text_cache();
        // The parser grows its vectors geometrically. Trim the unused tail so
        // the steady-state heap matches the actual document size.
        self.nodes.shrink_to_fit();
        self.roots.shrink_to_fit();
        self.fixed_subtree.shrink_to_fit();
        self.pending.shrink_to_fit();
        self.state.shrink_to_fit();
        self.script.clicks.shrink_to_fit();
        self.script.functions.shrink_to_fit();
        for (_, actions) in &mut self.script.clicks {
            actions.shrink_to_fit();
        }
        for (_, actions) in &mut self.script.functions {
            actions.shrink_to_fit();
        }
    }

    pub(crate) fn rebuild_fixed_subtree(&mut self, chrome_mode: bool) {
        self.fixed_subtree.clear();
        self.fixed_subtree.resize(self.nodes.len(), false);
        let roots = self.roots.clone();
        for root in roots {
            self.mark_fixed_subtree(root, chrome_mode);
        }
    }

    pub(crate) fn mark_fixed_subtree(&mut self, idx: usize, chrome_mode: bool) -> bool {
        let mut has_fixed = self.node_is_fixed(idx, chrome_mode);
        let children = self.nodes[idx].children.clone();
        for child in children {
            has_fixed |= self.mark_fixed_subtree(child, chrome_mode);
        }
        self.fixed_subtree[idx] = has_fixed;
        has_fixed
    }

    pub fn update(&mut self, width: i32, height: i32) {
        let width = width.max(1);
        let height = height.max(1);
        if self.width != width || self.height != height {
            self.layout_dirty = true;
        }
        self.width = width;
        self.height = height;
        // Scrolling and hover only invalidate pixels. They do not change the
        // XML layout, so avoid measuring the whole view tree for every frame.
        if !self.layout_dirty {
            self.dirty = false;
            return;
        }
        self.refresh_visibility();
        let roots = self.roots.clone();
        // `height` is the viewport below the window title bar.  Coordinates
        // remain full-layer coordinates so the compositor can apply its
        // window scroll offset without a second layout coordinate system.
        // The generated HTML has no implicit document margin, so the native
        // root starts at the content edge and uses the complete width.
        let mut y = self.chrome_height;
        let usable = self.height;
        for root in roots {
            let forced = if is_match_parent(self.nodes[root].attr("layout_height")) {
                Some(usable)
            } else {
                None
            };
            let h = self.layout(root, 0, y, self.width, forced, "");
            y += h;
        }
        // Scroll extent is deliberately owned by explicit XML scroll
        // containers.  Use each container's actual document bottom instead
        // of adding the largest overflow to the last root: nested
        // ScrollViews otherwise produce an incorrect end position and leave
        // a blank strip after the viewport has moved.
        let mut document_bottom = y;
        for idx in 0..self.nodes.len() {
            if is_scroll_container(&self.nodes[idx]) {
                document_bottom = document_bottom
                    .max(self.nodes[idx].y.saturating_add(self.nodes[idx].content_h));
            }
        }
        self.content_height = document_bottom.max(self.height + self.chrome_height);
        let max_scroll = self
            .content_height
            .saturating_sub(self.height + self.chrome_height)
            .max(0);
        self.scroll = self.scroll.min(max_scroll);
        self.scroll_target = self.scroll_target.min(max_scroll);
        self.scroll_start = self.scroll_start.min(max_scroll);
        self.scroll_subpixel = self.scroll_subpixel.min(max_scroll.saturating_mul(3));
        self.scroll_target_subpixel = self.scroll_target_subpixel.min(max_scroll.saturating_mul(3));
        self.scroll_start_subpixel = self.scroll_start_subpixel.min(max_scroll.saturating_mul(3));
        self.rebuild_focusable_nodes();
        self.damage_all();
        self.dirty = false;
        self.layout_dirty = false;
    }
}
