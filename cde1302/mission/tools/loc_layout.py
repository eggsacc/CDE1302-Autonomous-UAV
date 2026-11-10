#!/usr/bin/env python3
"""
Lay out the localisation tags on the floor grid and write them to config.yaml.

    python mission/tools/loc_layout.py                   # mission/config.yaml
    python mission/tools/loc_layout.py --config other.yaml

The arena floor is a grid of nodes NODE_M (0.46 m) apart.  Node (a, b), as the
team writes it: b counts forward from the start node (0, 0) along the search
length, a counts sideways, + to the drone's right.  In the arena frame of
localisation.py (origin = home tag at the start node, x forward, y right):

    x = b * 0.46        y = a * 0.46

The window shows the grid as seen from behind the drone: start at the bottom,
the drone facing up, its right on the right.  The search area (config.yaml
`area`) is shaded.  Pick a tag ID (22-29, the IDs restored at the end of
tag16h5, which the course's own tags never use), click the node it is on;
click another node to move it; right-click a tag to remove it.  The tag on the
start node (0, 0) is the home tag (precision landing, alignment).  "Save"
rewrites `localisation.home_tag_id` and `localisation.tags` in config.yaml (a
copy of the old file is kept as config.yaml.bak), after checking that the
result loads.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path
from typing import Optional

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # mission/

from settings import (DEFAULT_CONFIG_PATH, TAG16H5_MAX_ID, ConfigError,  # noqa: E402
                      settings_from_dict)

NODE_M = 0.46                                  # distance between grid nodes (m)
LOC_IDS = tuple(range(22, TAG16H5_MAX_ID + 1))  # 22-29: the restored tail of tag16h5
START = (0, 0)


# ---------------------------------------------------------------------------
# Grid <-> arena frame
# ---------------------------------------------------------------------------

def node_to_xy(node: tuple[int, int], spacing: float = NODE_M) -> tuple[float, float]:
    """Node (a sideways, b forward) -> arena (x forward, y right), metres."""
    a, b = node
    return round(b * spacing, 3), round(a * spacing, 3)


def xy_to_node(x: float, y: float, spacing: float = NODE_M,
               tol: float = 0.03) -> Optional[tuple[int, int]]:
    """The node at arena (x, y), or None if it is more than tol from one."""
    a, b = round(y / spacing), round(x / spacing)
    if abs(a * spacing - y) > tol or abs(b * spacing - x) > tol:
        return None
    return int(a), int(b)


def grid_extent(area: dict, spacing: float = NODE_M) -> tuple[range, range]:
    """Columns (a) and rows (b) to show: the search area plus the start node."""
    a0 = min(0, int(round(area["y_min"] / spacing)))
    a1 = max(0, int(round(area["y_max"] / spacing)))
    b1 = max(0, int(round(area["x_max"] / spacing)))
    return range(a0, a1 + 1), range(0, b1 + 1)


# ---------------------------------------------------------------------------
# config.yaml
# ---------------------------------------------------------------------------

def read_layout(text: str, spacing: float = NODE_M) -> tuple[dict[int, tuple[int, int]], list[str]]:
    """{tag id: node} from config.yaml text, plus warnings for tags off the grid."""
    data = yaml.safe_load(text)
    out, warn = {}, []
    for t in (data.get("localisation") or {}).get("tags") or []:
        node = xy_to_node(float(t["x"]), float(t["y"]), spacing)
        if node is None:
            warn.append(f"tag {t['id']} at ({t['x']}, {t['y']}) is not on a grid node; "
                        f"place it again")
        else:
            out[int(t["id"])] = node
    return out, warn


def check_layout(layout: dict[int, tuple[int, int]]) -> Optional[str]:
    """None if the layout is usable, else why not."""
    if not layout:
        return "no localisation tags placed"
    bad = [i for i in layout if i not in LOC_IDS]
    if bad:
        return f"tag id(s) {bad} are not localisation ids {LOC_IDS[0]}-{LOC_IDS[-1]}"
    nodes = list(layout.values())
    if len(nodes) != len(set(nodes)):
        return "two tags on the same node"
    if START not in nodes:
        return "no tag on the start node (0, 0): the home tag is needed for landing"
    return None


def update_config_text(text: str, layout: dict[int, tuple[int, int]],
                       spacing: float = NODE_M) -> str:
    """config.yaml text with localisation.home_tag_id and .tags rewritten.

    Only those lines change, so every comment and setting elsewhere stays.
    Raises ValueError (bad layout) or ConfigError (result does not load).
    """
    problem = check_layout(layout)
    if problem:
        raise ValueError(problem)
    home = next(i for i, n in layout.items() if n == START)
    lines = text.splitlines(keepends=True)
    try:
        start = next(i for i, ln in enumerate(lines) if ln.startswith("localisation:"))
    except StopIteration:
        raise ValueError("config has no `localisation:` section") from None
    end = next((i for i in range(start + 1, len(lines))
                if lines[i].strip() and not lines[i].startswith((" ", "#"))), len(lines))
    nl = "\r\n" if lines[start].endswith("\r\n") else "\n"
    i_home = next((i for i in range(start, end) if re.match(r"  home_tag_id:", lines[i])), None)
    i_tags = next((i for i in range(start, end) if re.match(r"  tags:\s*(#.*)?$", lines[i])),
                  None)
    if i_home is None or i_tags is None:
        raise ValueError("localisation needs `home_tag_id:` and `tags:` lines to update")
    lines[i_home] = re.sub(r"(home_tag_id:\s*)\d+", rf"\g<1>{home}", lines[i_home])
    j = i_tags + 1
    while j < end and lines[j].startswith("    - "):
        j += 1
    items = []
    for tid, node in sorted(layout.items(), key=lambda kv: (kv[1] != START, kv[1][1], kv[1][0])):
        x, y = node_to_xy(node, spacing)
        what = "home tag, start node" if node == START else "node"
        items.append(f"    - {{id: {tid}, x: {x}, y: {y}}}   # {what} {node}{nl}")
    lines[i_tags + 1:j] = items
    new = "".join(lines)
    settings_from_dict(yaml.safe_load(new))         # must still load (raises ConfigError)
    return new


def save_layout(path: Path, layout: dict[int, tuple[int, int]],
                spacing: float = NODE_M) -> Path:
    """Rewrite the layout into the config file, keeping a .bak of the old one."""
    text = path.read_text(encoding="utf-8")
    new = update_config_text(text, layout, spacing)
    shutil.copyfile(path, path.with_name(path.name + ".bak"))
    path.write_text(new, encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# Window
# ---------------------------------------------------------------------------

CELL = 60            # grid pitch at 100 % display scaling (px); scaled to the screen
PAD = 56             # margin left / top (px)
PAD_BOTTOM = 72      # room under the start row for the column numbers and START (px)


class LayoutApp:
    """Tkinter window: the grid on the left, tag ids and Save on the right."""

    def __init__(self, root, config_path: Path, spacing: float = NODE_M) -> None:
        import tkinter as tk
        self.tk, self.root = tk, root
        self.path, self.spacing = config_path, spacing
        text = config_path.read_text(encoding="utf-8")
        data = yaml.safe_load(text)
        self.area = data["area"]
        self.layout, warnings = read_layout(text, spacing)
        self.cols, self.rows = grid_extent(self.area, spacing)
        root.title(f"Localisation tags - {config_path.name}")
        k = max(1.0, root.winfo_fpixels("1i") / 96.0)        # display scaling
        self.cell, self.pad, self.box = round(CELL * k), round(PAD * k), round(16 * k)
        # shrink the grid to fit a smaller screen (title bar + taskbar ~ 22 %)
        fit = (root.winfo_screenheight() * 0.78 - self.pad - PAD_BOTTOM * k) / max(1, len(self.rows) - 1)
        if fit < self.cell:
            k_fit = fit / CELL
            self.cell, self.box = round(fit), round(16 * k_fit)
        self.k = k

        w = self.pad * 2 + (len(self.cols) - 1) * self.cell
        h = self.pad + round(PAD_BOTTOM * k) + (len(self.rows) - 1) * self.cell
        self.canvas = tk.Canvas(root, width=w, height=h, bg="#fcfcfb", highlightthickness=0)
        self.canvas.grid(row=0, column=0, rowspan=3, padx=8, pady=8)
        self.canvas.bind("<Button-1>", self._click)
        self.canvas.bind("<Button-3>", self._right_click)
        self.canvas.bind("<Motion>", self._hover)

        side = tk.Frame(root)
        side.grid(row=0, column=1, sticky="n", padx=8, pady=8)
        tk.Label(side, text="Tag to place", font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.current = tk.IntVar(value=LOC_IDS[-1])
        self.id_labels = {}
        for tid in reversed(LOC_IDS):
            row = tk.Frame(side)
            row.pack(anchor="w", fill="x")
            tk.Radiobutton(row, text=f"{tid}", variable=self.current, value=tid,
                           width=4, anchor="w").pack(side="left")
            self.id_labels[tid] = tk.Label(row, text="", width=26, anchor="w", fg="#52514e")
            self.id_labels[tid].pack(side="left")
        tk.Label(side, justify="left", fg="#52514e", wraplength=260, text=(
            "\nClick a node to put the selected tag there (click again elsewhere "
            "to move it). Right-click a tag to remove it.\n\nThe tag on START (0, 0) "
            "is the home tag. Seen from behind the drone: it faces up, its right is "
            "on the right. Node (a, b) = x b*0.46 m forward, y a*0.46 m right.")).pack(anchor="w")
        buttons = tk.Frame(side)
        buttons.pack(anchor="w", pady=10)
        tk.Button(buttons, text="Save to config", command=self._save, width=14).pack(side="left")
        tk.Button(buttons, text="Revert", command=self._revert, width=8).pack(side="left", padx=6)
        self.status = tk.Label(root, text="; ".join(warnings) or "Ready.", anchor="w",
                               fg="#52514e", justify="left", wraplength=w + 280)
        self.status.grid(row=3, column=0, columnspan=2, sticky="we", padx=8, pady=(0, 8))
        self._draw()

    # -- geometry -------------------------------------------------------------

    def _px(self, node: tuple[int, int]) -> tuple[float, float]:
        a, b = node
        return (self.pad + (a - self.cols.start) * self.cell,
                self.pad + (self.rows[-1] - b) * self.cell)          # forward is up

    def _node_at(self, px: float, py: float) -> Optional[tuple[int, int]]:
        a = round((px - self.pad) / self.cell) + self.cols.start
        b = self.rows[-1] - round((py - self.pad) / self.cell)
        if a not in self.cols or b not in self.rows:
            return None
        cx, cy = self._px((a, b))
        if abs(cx - px) > self.cell * 0.45 or abs(cy - py) > self.cell * 0.45:
            return None
        return a, b

    # -- drawing --------------------------------------------------------------

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")
        s = self.spacing
        # search area (config), shaded
        x0, y0 = self._px((self.area["y_min"] / s, self.area["x_max"] / s))
        x1, y1 = self._px((self.area["y_max"] / s, self.area["x_min"] / s))
        c.create_rectangle(x0, y0, x1, y1, fill="#e8f0fb", outline="#2a78d6", dash=(4, 3))
        k = self.k
        c.create_text((x0 + x1) / 2, y0 - 30 * k, text="search area (far end)", fill="#2a78d6",
                      font=("Segoe UI", 8))
        # grid lines + axis labels
        for a in self.cols:
            px, top = self._px((a, self.rows[-1]))
            _, bottom = self._px((a, self.rows.start))
            c.create_line(px, top, px, bottom, fill="#dcdad5")
            c.create_text(px, bottom + 30 * k, text=str(a), fill="#52514e", font=("Segoe UI", 8))
        for b in self.rows:
            left, py = self._px((self.cols.start, b))
            right, _ = self._px((self.cols[-1], b))
            c.create_line(left, py, right, py, fill="#dcdad5")
            c.create_text(left - 34 * k, py, text=str(b), fill="#52514e", font=("Segoe UI", 8))
        c.create_text(self.pad - 34 * k, self.pad - 30 * k, text="b", fill="#52514e",
                      font=("Segoe UI", 8, "italic"))
        c.create_text(self._px((self.cols[-1], 0))[0] + 34 * k, self._px((0, 0))[1] + 30 * k,
                      text="a",
                      fill="#52514e", font=("Segoe UI", 8, "italic"))
        # start node + drone arrow
        sx, sy = self._px(START)
        c.create_line(sx, sy + 62 * k, sx, sy + 46 * k, arrow="last", width=2, fill="#0b0b0b")
        c.create_text(sx + 30 * k, sy + 56 * k, text="START", fill="#0b0b0b",
                      font=("Segoe UI", 8, "bold"))
        # nodes
        placed = {n: tid for tid, n in self.layout.items()}
        for a in self.cols:
            for b in self.rows:
                px, py = self._px((a, b))
                tid = placed.get((a, b))
                if tid is None:
                    c.create_oval(px - 4, py - 4, px + 4, py + 4, fill="#9a9893", outline="")
                else:
                    home = (a, b) == START
                    b_ = self.box
                    c.create_rectangle(px - b_, py - b_, px + b_, py + b_,
                                       fill="#0b0b0b" if home else "#2a78d6", outline="")
                    c.create_text(px, py, text=str(tid), fill="white",
                                  font=("Segoe UI", 10, "bold"))
        # side panel
        for tid, lab in self.id_labels.items():
            node = self.layout.get(tid)
            if node is None:
                lab.config(text="not placed")
            else:
                x, y = node_to_xy(node, s)
                lab.config(text=f"node {node}  x {x:.2f}  y {y:+.2f}"
                                + ("  HOME" if node == START else ""))

    # -- events ---------------------------------------------------------------

    def _click(self, ev) -> None:
        node = self._node_at(ev.x, ev.y)
        if node is None:
            return
        tid = self.current.get()
        for other, n in list(self.layout.items()):        # one tag per node
            if n == node and other != tid:
                del self.layout[other]
        self.layout[tid] = node
        x, y = node_to_xy(node, self.spacing)
        self._set_status(f"Tag {tid} on node {node}: x {x:.2f} m, y {y:+.2f} m. Not saved yet.")
        self._draw()

    def _right_click(self, ev) -> None:
        node = self._node_at(ev.x, ev.y)
        for tid, n in list(self.layout.items()):
            if n == node:
                del self.layout[tid]
                self._set_status(f"Tag {tid} removed. Not saved yet.")
        self._draw()

    def _hover(self, ev) -> None:
        node = self._node_at(ev.x, ev.y)
        if node is not None:
            x, y = node_to_xy(node, self.spacing)
            self.canvas.config(cursor="hand2")
            self.root.title(f"Localisation tags - node {node} = x {x:.2f} m, y {y:+.2f} m")
        else:
            self.canvas.config(cursor="")

    def _save(self) -> None:
        try:
            save_layout(self.path, self.layout, self.spacing)
        except (ValueError, ConfigError) as exc:
            self._set_status(f"NOT saved: {exc}", error=True)
            return
        home = next(i for i, n in self.layout.items() if n == START)
        self._set_status(f"Saved {len(self.layout)} tags to {self.path} (home tag {home}); "
                         f"old file kept as {self.path.name}.bak.")

    def _revert(self) -> None:
        self.layout, warnings = read_layout(self.path.read_text(encoding="utf-8"), self.spacing)
        self._set_status("; ".join(warnings) or "Reverted to the saved config.")
        self._draw()

    def _set_status(self, text: str, error: bool = False) -> None:
        self.status.config(text=text, fg="#e34948" if error else "#52514e")


def sharp_on_windows() -> None:
    """Tell Windows this window draws itself at full resolution (else a scaled
    screen shows it blurry and enlarged)."""
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Place the localisation tags on the floor grid.")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG_PATH), help="config.yaml to update")
    ap.add_argument("--spacing", type=float, default=NODE_M, help="grid node spacing (m)")
    args = ap.parse_args(argv)
    sharp_on_windows()
    import tkinter as tk
    root = tk.Tk()
    LayoutApp(root, Path(args.config), args.spacing)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
