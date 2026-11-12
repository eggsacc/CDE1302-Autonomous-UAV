#!/usr/bin/env python3
"""
Build the CDE1302 final report (Word .docx) from the analysed flight data.

    python report/analyse_flights.py     # first: figures + data/metrics.json
    python report/build_report.py        # then:  CDE1302_Final_Report_DRAFT.docx

Formatting (set by the team): academic formal, Times New Roman, no text
smaller than 12 pt; single spacing; AT MOST 10 PAGES including the cover
(course rule); standalone (no repository paths, file
or branch names in the text).  Unfinished items are marked in the text as
    [TO BE COMPLETED: ...]   yellow highlight
    [Image placeholder] ...  shaded box (images are added by hand)
so they can be found with Ctrl+F.  See report/AGENT_NOTES.md before editing.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_COLOR_INDEX, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = Path(__file__).resolve().parent
FIG = ROOT / "figures"
OUT = ROOT / "CDE1302_Final_Report_DRAFT.docx"
METRICS = json.loads((ROOT / "data" / "metrics.json").read_text(encoding="utf-8"))
R1, R2 = METRICS["runs"]
DISP = METRICS["dispersion"]

FONT = "Times New Roman"
MONO = "Courier New"
BODY_PT = 12
TEXT_WIDTH_CM = 15.9          # A4 minus 2 x 2.54 cm margins

# ---------------------------------------------------------------------------
# Figure / table numbering (declared up front so the text can cite them)
# ---------------------------------------------------------------------------
FIGS = ["arch", "path1", "odom_div"]
TABS = ["hw", "params", "summary", "positions"]


def F(key):
    return f"Figure {FIGS.index(key) + 1}"


def T(key):
    return f"Table {TABS.index(key) + 1}"


def cm(v):
    return f"{v * 100:.1f} cm"


# ---------------------------------------------------------------------------
# Low-level formatting helpers
# ---------------------------------------------------------------------------

def set_font(run, size=BODY_PT, bold=None, italic=None, mono=False, color=None):
    f = MONO if mono else FONT
    run.font.name = f
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.insert(0, rfonts)
    for a in ("w:ascii", "w:hAnsi", "w:eastAsia", "w:cs"):
        rfonts.set(qn(a), f)
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic
    if color is not None:
        run.font.color.rgb = RGBColor.from_string(color)


def style_font(style, size, bold=False, italic=False, color="000000"):
    style.font.name = FONT
    style.font.size = Pt(size)
    style.font.bold = bold
    style.font.italic = italic
    style.font.color.rgb = RGBColor.from_string(color)
    rpr = style.element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.insert(0, rfonts)
    for a in ("w:ascii", "w:hAnsi", "w:eastAsia", "w:cs"):
        rfonts.set(qn(a), FONT)
    for a in ("w:asciiTheme", "w:hAnsiTheme", "w:eastAsiaTheme", "w:cstheme"):
        if rfonts.get(qn(a)) is not None:
            del rfonts.attrib[qn(a)]


def shade(cell, hex_fill):
    tcpr = cell._element.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tcpr.append(shd)


def para_shade(p, hex_fill):
    ppr = p._element.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    ppr.append(shd)


def para_border(p, color="7F7F7F"):
    ppr = p._element.get_or_add_pPr()
    bdr = OxmlElement("w:pBdr")
    for side in ("top", "left", "bottom", "right"):
        e = OxmlElement(f"w:{side}")
        e.set(qn("w:val"), "single")
        e.set(qn("w:sz"), "6")
        e.set(qn("w:space"), "4")
        e.set(qn("w:color"), color)
        bdr.append(e)
    ppr.append(bdr)


def add_field(paragraph, instr, placeholder=""):
    run = paragraph.add_run()
    set_font(run)
    b = OxmlElement("w:fldChar")
    b.set(qn("w:fldCharType"), "begin")
    i = OxmlElement("w:instrText")
    i.set(qn("xml:space"), "preserve")
    i.text = instr
    s = OxmlElement("w:fldChar")
    s.set(qn("w:fldCharType"), "separate")
    t = OxmlElement("w:t")
    t.text = placeholder
    e = OxmlElement("w:fldChar")
    e.set(qn("w:fldCharType"), "end")
    for el in (b, i, s, t, e):
        run._element.append(el)


INLINE = re.compile(r"(\*\*.+?\*\*|\*[^*]+?\*|`[^`]+?`|\[TO BE COMPLETED:[^\]]*\]|\[TO CONFIRM:[^\]]*\]"
                    r"|\[TBC\])")


def add_rich(p, text, size=BODY_PT, base_bold=False, base_italic=False):
    """Inline markup: **bold**, *italic*, `code`, [TO BE COMPLETED: ...] (highlight)."""
    for part in INLINE.split(text):
        if not part:
            continue
        if part.startswith("**") and part.endswith("**"):
            r = p.add_run(part[2:-2])
            set_font(r, size, bold=True, italic=base_italic)
        elif part.startswith("`") and part.endswith("`"):
            r = p.add_run(part[1:-1])
            set_font(r, size, bold=base_bold)   # course rule: Times New Roman only
        elif (part.startswith("[TO BE COMPLETED") or part.startswith("[TO CONFIRM")
              or part == "[TBC]"):
            r = p.add_run(part)
            set_font(r, size, bold=True, italic=False)
            r.font.highlight_color = WD_COLOR_INDEX.YELLOW
        elif part.startswith("*") and part.endswith("*") and len(part) > 2:
            r = p.add_run(part[1:-1])
            set_font(r, size, bold=base_bold, italic=True)
        else:
            r = p.add_run(part)
            set_font(r, size, bold=base_bold, italic=base_italic)
    return p


# ---------------------------------------------------------------------------
# Document builder
# ---------------------------------------------------------------------------

class Report:
    def __init__(self):
        self.doc = Document()
        self._setup()

    def _setup(self):
        d = self.doc
        sec = d.sections[0]
        sec.page_height, sec.page_width = Cm(29.7), Cm(21.0)
        sec.top_margin = sec.bottom_margin = Cm(2.54)
        sec.left_margin = sec.right_margin = Cm(2.54)
        st = d.styles
        normal = st["Normal"]
        style_font(normal, BODY_PT)
        pf = normal.paragraph_format
        pf.line_spacing_rule = WD_LINE_SPACING.SINGLE
        pf.space_after = Pt(6)
        pf.space_before = Pt(0)
        for name, size, before, after, italic in (("Heading 1", 16, 14, 6, False),
                                                  ("Heading 2", 14, 10, 4, False),
                                                  ("Heading 3", 12, 8, 4, True)):
            h = st[name]
            style_font(h, size, bold=True, italic=italic)
            h.paragraph_format.space_before = Pt(before)
            h.paragraph_format.space_after = Pt(after)
            h.paragraph_format.keep_with_next = True
            h.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        for name in ("Title", "Subtitle", "Caption", "List Bullet", "List Number",
                     "TOC Heading"):
            try:
                style_font(st[name], BODY_PT)
            except KeyError:
                pass
        for lvl in range(1, 4):
            try:
                style_font(st[f"TOC {lvl}"], BODY_PT, bold=(lvl == 1))
            except KeyError:
                pass
        # Ask Word to refresh the TOC / fields when the file is opened.
        settings = d.settings.element
        upd = OxmlElement("w:updateFields")
        upd.set(qn("w:val"), "true")
        settings.append(upd)
        # Footer page number
        fp = sec.footer.paragraphs[0]
        fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
        add_field(fp, "PAGE", "1")

    # -- block elements -----------------------------------------------------
    def h1(self, text, page_break=False):
        if page_break:
            self.page_break()
        return self.doc.add_heading(text, level=1)

    def h2(self, text):
        return self.doc.add_heading(text, level=2)

    def h3(self, text):
        return self.doc.add_heading(text, level=3)

    def p(self, text, align="justify", italic=False, bold=False, space_after=None, indent=None):
        para = self.doc.add_paragraph()
        para.alignment = {"justify": WD_ALIGN_PARAGRAPH.JUSTIFY,
                          "center": WD_ALIGN_PARAGRAPH.CENTER,
                          "left": WD_ALIGN_PARAGRAPH.LEFT,
                          "right": WD_ALIGN_PARAGRAPH.RIGHT}[align]
        if space_after is not None:
            para.paragraph_format.space_after = Pt(space_after)
        if indent is not None:
            para.paragraph_format.left_indent = Cm(indent)
        add_rich(para, text, base_bold=bold, base_italic=italic)
        return para

    def bullets(self, items, numbered=False):
        style = "List Number" if numbered else "List Bullet"
        for it in items:
            para = self.doc.add_paragraph(style=style)
            para.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            para.paragraph_format.space_after = Pt(3)
            add_rich(para, it)

    def equation(self, text, number):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = para.add_run(text)
        set_font(r, BODY_PT, italic=True)
        r = para.add_run(f"        ({number})")
        set_font(r, BODY_PT)
        return para

    def code(self, lines):
        for i, ln in enumerate(lines):
            para = self.doc.add_paragraph()
            para.alignment = WD_ALIGN_PARAGRAPH.LEFT
            para.paragraph_format.space_after = Pt(0)
            para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
            para.paragraph_format.left_indent = Cm(0.5)
            para_shade(para, "F2F2F2")
            r = para.add_run(ln if ln else " ")
            set_font(r, BODY_PT, mono=True)
        self.doc.add_paragraph().paragraph_format.space_after = Pt(0)

    def caption(self, text):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        para.paragraph_format.space_after = Pt(12)
        para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        add_rich(para, text, base_italic=False)
        for r in para.runs[:1]:
            r.bold = True
        return para

    def figure(self, key, path, caption, width_cm=16.0):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        para.paragraph_format.keep_with_next = True
        para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        para.add_run().add_picture(str(path), width=Cm(width_cm))
        self._fig_caption(key, caption)

    def _fig_caption(self, key, caption):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        para.paragraph_format.space_after = Pt(12)
        para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        r = para.add_run(f"{F(key)}. ")
        set_font(r, BODY_PT, bold=True)
        add_rich(para, caption)

    def image_placeholder(self, key, description, caption):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        para.paragraph_format.keep_with_next = True
        para.paragraph_format.space_before = Pt(6)
        para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        para_shade(para, "EDEDED")
        para_border(para)
        r = para.add_run("[Image placeholder] ")
        set_font(r, BODY_PT, bold=True)
        add_rich(para, description)
        self._fig_caption(key, caption)

    def todo(self, text):
        para = self.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
        para_border(para, "C9A100")
        para_shade(para, "FFF8D6")
        r = para.add_run("[TO BE COMPLETED] ")
        set_font(r, BODY_PT, bold=True)
        r.font.highlight_color = WD_COLOR_INDEX.YELLOW
        add_rich(para, text)
        return para

    def table(self, key, caption, header, rows, widths_cm=None, align=None, font_pt=BODY_PT,
              label=None):
        cap = self.doc.add_paragraph()
        cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
        cap.paragraph_format.keep_with_next = True
        cap.paragraph_format.space_before = Pt(6)
        cap.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        r = cap.add_run(f"{T(key)}. " if label is None else label)
        set_font(r, BODY_PT, bold=True)
        add_rich(cap, caption)
        t = self.doc.add_table(rows=1 + len(rows), cols=len(header))
        t.style = "Table Grid"
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        t.autofit = False
        if widths_cm:
            k = TEXT_WIDTH_CM / sum(widths_cm)
            widths_cm = [w * k for w in widths_cm]
            tblpr = t._tbl.tblPr
            tblw = tblpr.find(qn("w:tblW"))
            if tblw is None:
                tblw = OxmlElement("w:tblW")
                tblpr.append(tblw)
            tblw.set(qn("w:type"), "dxa")
            tblw.set(qn("w:w"), str(int(TEXT_WIDTH_CM / 2.54 * 1440)))
            grid = t._tbl.tblGrid
            for gc, w in zip(grid.findall(qn("w:gridCol")), widths_cm):
                gc.set(qn("w:w"), str(int(w / 2.54 * 1440)))
        for j, h in enumerate(header):
            c = t.rows[0].cells[j]
            c.text = ""
            add_rich(c.paragraphs[0], h, size=font_pt, base_bold=True)
            shade(c, "D9D9D9")
        # repeat header row on page breaks
        trpr = t.rows[0]._tr.get_or_add_trPr()
        th = OxmlElement("w:tblHeader")
        th.set(qn("w:val"), "true")
        trpr.append(th)
        for i, row in enumerate(rows, 1):
            for j, val in enumerate(row):
                c = t.rows[i].cells[j]
                c.text = ""
                add_rich(c.paragraphs[0], str(val), size=font_pt)
        for row in t.rows:
            for j, c in enumerate(row.cells):
                for para in c.paragraphs:
                    para.paragraph_format.space_after = Pt(0)
                    para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
                    if align and align[j] == "c":
                        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
                    elif align and align[j] == "r":
                        para.alignment = WD_ALIGN_PARAGRAPH.RIGHT
                if widths_cm:
                    c.width = Cm(widths_cm[j])
        after = self.doc.add_paragraph()
        after.paragraph_format.space_after = Pt(6)
        return t

    def page_break(self):
        para = self.doc.add_paragraph()
        para.add_run().add_break(WD_BREAK.PAGE)

    def save(self, path):
        self.doc.save(str(path))


# ---------------------------------------------------------------------------
# Content
# ---------------------------------------------------------------------------
# Course rules: at most 10 pages including the cover (and any appendix), single
# spacing, Times New Roman >= 12 pt, GitHub link in the report, and these
# points: architecture and data/control flow; exploration and navigation;
# AprilTag detection and localisation; important parameters and decisions;
# results; AI-assisted flight-data analysis; limitations and improvements.
# The text is standalone: no repo paths, file names, branches or config keys.

REPO_URL = "https://github.com/eggsacc/CDE1302-Autonomous-UAV"


def run_label(r):
    return r["name"].split(":")[0]


def title_page(R: Report):
    R.p("", space_after=0)
    for text, size, bold in (("NATIONAL UNIVERSITY OF SINGAPORE", 14, True),
                             ("CDE1302 Introduction to Unmanned Aerial Vehicles", 14, False),
                             ("Academic Year 2026/27", 12, False)):
        para = R.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = para.add_run(text)
        set_font(r, size, bold=bold)
    R.p("", space_after=12)
    para = R.doc.add_paragraph()
    para.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = para.add_run("Autonomous AprilTag Search and Localisation with an Indoor Quadcopter")
    set_font(r, 20, bold=True)
    para = R.doc.add_paragraph()
    para.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = para.add_run("Mini Project Final Report")
    set_font(r, 16)
    para = R.doc.add_paragraph()
    para.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = para.add_run("DRAFT")
    set_font(r, 14, bold=True, color="C00000")
    R.p("", space_after=12)
    para = R.doc.add_paragraph()
    para.alignment = WD_ALIGN_PARAGRAPH.CENTER
    para_shade(para, "EDEDED")
    para_border(para)
    r = para.add_run("[Image placeholder] ")
    set_font(r, BODY_PT, bold=True)
    add_rich(para, "Photograph of the demonstration drone (about 7 cm tall).")
    R.p("", space_after=12)
    R.p("Group [TO BE COMPLETED: group number]  ·  Drone ID 2", align="center")
    R.table(None, "Team members", ["Name", "Matriculation number", "Primary responsibilities"],
            [["[TO BE COMPLETED: name]", "[TO BE COMPLETED]", "[TO BE COMPLETED]"]] * 4,
            widths_cm=[5.5, 4.5, 6.0], label="")
    R.p(f"Source code: {REPO_URL}", align="center")
    R.p("Submitted: [TO BE COMPLETED: date]", align="center")


def sec_summary(R: Report):
    R.h1("Summary", page_break=True)
    R.p(
        "We built a pseudo-autonomous indoor quadcopter that sweeps an allocated rectangle, "
        "finds AprilTags on the floor, returns, lands on its start box and reports each tag's "
        "position. With no GPS or compass, PX4 estimates position from optical flow and a "
        "rangefinder. A laptop plans the sweep and detects tags in a camera stream whose frames "
        "carry the drone's pose at capture. The companion computer flies the sweep on board. "
        f"In the demonstration flights all three search tags were found with no false "
        f"detections. From the take-off command to the land command, the full sweep took "
        f"{R1['mission_time_s']:.0f} s and a faster two-lane sweep {R2['mission_time_s']:.0f} s. "
        f"Surveyed localisation tags cut the distance between the two flights' tag estimates "
        f"from {min(d['raw']['max_pairwise_m'] for d in DISP.values()) * 100:.0f}–"
        f"{max(d['raw']['max_pairwise_m'] for d in DISP.values()) * 100:.0f} cm to "
        f"{min(d['corr']['max_pairwise_m'] for d in DISP.values()) * 100:.0f}–"
        f"{max(d['corr']['max_pairwise_m'] for d in DISP.values()) * 100:.0f} cm. "
        f"AI-assisted log analysis traced sudden "
        f"multi-metre odometry jumps in earlier trials to PX4 switching between two estimators.")


def sec_architecture(R: Report):
    R.h1("1  System Architecture and Data/Control Flow")
    R.p(
        "The task is to search an allocated rectangle of an indoor arena for tag16h5 AprilTags "
        "[1], [2], then return, land and report each tag's position. The arena has no external "
        "positioning system. The operator starts the mission with two typed commands (`takeoff`, "
        "`start`) and holds the radio kill switch, but never steers.")
    R.table("hw", "Vehicle and ground equipment.",
            ["Component", "Part", "Role"],
            [["Flight controller", "MicoAir H743 V2 AIO, PX4 v1.15.4",
              "Cascaded position, velocity, attitude and rate control; EKF2 estimation"],
             ["Flow and range sensor", "MTF-02P", "Horizontal velocity; height above the floor"],
             ["Companion computer", "Seeed XIAO ESP32-S3 Sense",
              "MAVLink bridge, Wi-Fi link, on-board trajectory player, camera"],
             ["Camera", "OV3660, facing down", "320 × 240 JPEG stream to the laptop"],
             ["Ground station", "Laptop, dedicated router", "Mission logic, planning, detection, logs"],
             ["Frame", "Group-designed 3D-printed PLA", "See below"],
             ["Propulsion, battery", "[TO BE COMPLETED]", "[TO BE COMPLETED]"]],
            widths_cm=[3.6, 5.2, 7.2])
    R.p(
        f"Control is split across three computers ({F('arch')}). The **laptop** runs a 10 Hz "
        "mission state machine, the planner, the tag detector and the logging. The **ESP32-S3** "
        "talks MAVLink [5] to PX4 over a 921 600 baud UART and to the laptop over UDP. It arms, "
        "takes off and lands the drone, plays the uploaded trajectory at 20 Hz and streams the "
        "camera. **PX4** [4] closes the control loops. The data loop runs as follows: the laptop "
        "uploads the planned path; the ESP32 turns it into position setpoints; PX4 flies them and "
        "reports its pose; the ESP32 stamps each camera frame with that pose; the laptop detects "
        "tags, updates the tag map and decides the next phase. Because the ESP32 flies the path "
        "itself, Wi-Fi delay or loss does not disturb the flight.")
    R.image_placeholder(
        "arch", "Block diagram: laptop ⇄ router ⇄ ESP32-S3 ⇄ PX4, camera and flow sensor, with the "
        "control cascade (10 Hz mission logic → 20 Hz setpoints → PX4 loops).",
        "System architecture and data/control flow.")
    R.p(
        "Positions are in PX4's local North-East-Down frame, with its origin at take-off and, "
        "with no compass, its x-axis wherever the nose pointed at power-up. Tags are reported in "
        "an arena frame centred on a home tag in the start box. The heading is held constant "
        "throughout the flight, so the drone flies sideways and backwards rather than turning.")
    R.p(
        "Beyond the course baseline, we redesigned the frame. The course frame was made for "
        "carbon fibre but printed in PLA, which is 20–30 times less stiff, and it flexed "
        "visibly. The new arms place material in deep top and bottom lattices joined by trusses, "
        "cutting the simulated deflection under a 30 N·m bending load from 14.4 mm to 5.31 mm "
        "(−63 %) at the same 50 g. They also need no print supports and cut assembly time from "
        "about an hour to 15 minutes.")


def sec_navigation(R: Report):
    R.h1("2  Exploration and Navigation Strategy")
    R.p(
        "**Mission phases and safety.** The laptop steps through CONNECTING → READY → TAKING_OFF "
        "→ HOVERING → EXPLORING → RETURNING → LANDING → DONE. The sweep starts only after the "
        "drone has held position within 10 cm for 2 s. A geofence 0.5 m around the area and "
        "path, or a 10 s telemetry time-out, triggers an automatic landing. The firmware cuts the "
        "motors if Wi-Fi is lost for about 3 s, and the kill switch has final authority.")
    R.p(
        "**Lawnmower sweep.** The area is shrunk by a 0.3 m margin and covered by parallel "
        "lanes along its length. For a shrunk width W and a maximum spacing s_max:")
    R.equation("n = ⌈W / s_max⌉ + 1,    s = W / (n − 1)", 1)
    R.p(
        "This puts the outer lanes on the boundary, with no gap larger than s_max. At height "
        "h = 0.8 m, with focal lengths f_x = 163.5 px and f_y = 153.2 px, the camera footprint is")
    R.equation("w_across = h · 320 / f_x ≈ 1.57 m,    w_along = h · 240 / f_y ≈ 1.25 m", 2)
    R.p(
        "A tag must lie wholly in the image, so the usable swath is about 1.45 m. Four lanes "
        "0.87 m apart therefore cover the area with overlap.")
    R.p(
        "**On-board execution.** The laptop rotates the plan into PX4's frame and uploads it to "
        "the ESP32 in one go. The ESP32 sends PX4 the next lane end as the setpoint, moving on "
        "once the drone is within 15 cm, so PX4's speed limit `MPC_XY_VEL_MAX` sets the cruise "
        "speed. The weakness is sideways correction. PX4 limits the *norm* of the velocity "
        "demand, so with along-track error e_a and cross-track error e_c the corrective speed "
        "across the lane is only")
    R.equation("v_c ≈ v_max · e_c / √(e_a² + e_c²)", 3)
    R.p("At the start of a long lane this is a few cm/s, which explains the cross-track bias "
        "seen in Section 5.")
    R.p(
        "**Alignment, return and landing.** With no compass, a 3° placement error would put the "
        "far end of a lane 0.25 m off line. While hovering, the drone measures the orientation of "
        "the home tag, which is taped square to the floor tiles, and the plan is rotated to match. "
        "After the sweep it flies back and re-observes the home tag. If the tag is more than 8 cm "
        "away, it moves over it (at most three corrections in 10 s) and lands.")


def sec_detection(R: Report):
    R.h1("3  AprilTag Detection and Target Localisation")
    R.p(
        "**Pose at capture.** Wi-Fi latency of 100–200 ms would shift every tag by 5–10 cm at "
        "0.5 m/s. The ESP32 therefore keeps a short history of PX4 poses and embeds the pose "
        "nearest the capture time in each JPEG, as a comment segment that image viewers ignore. "
        "Frames with no PX4 sample within 100 ms are discarded.")
    R.p(
        "**Detection and geolocation.** The laptop runs the course's AprilTag detector. Each "
        "detection gives the tag position t_c in the camera frame. This is mapped into the body "
        "frame (camera pitched 90°, lens 8 cm ahead of centre) and then into PX4's frame using "
        "the pose at capture:")
    R.equation("p_NED = [x, y]_drone + R_z(ψ) · (R_bc · t_c + r_lens)_xy", 4)
    R.p(
        "**Confirmation.** tag16h5 has only 30 codes, so false decodes are possible. Detections "
        "with a low decision margin, corrected bits, a poor pose fit, long range or a high yaw "
        "rate are rejected (" + T("params") + "). For each tag ID, the remaining sightings are clustered "
        "robustly. The seed is the sighting with the most neighbours within 0.2 m, and the "
        "estimate is the median of the sightings within 0.3 m of it. The tag is confirmed once "
        "five sightings agree within 0.2 m. An isolated false decode cannot gather five "
        "neighbours, so it is never confirmed.")
    R.p(
        "**Drift correction.** Optical-flow odometry drifts in scale, heading and position. "
        "Five localisation tags (IDs 22–25 and 29) were placed at surveyed floor-grid positions. "
        "Each visit to one pairs its PX4 position with its surveyed position. After the flight, "
        "a 2-D similarity transform is fitted to all visits by least squares [3]:")
    R.equation("p_arena = s · R(θ) · p_PX4 + t", 5)
    R.p(
        "Here s absorbs the flow scale error, θ the heading error and t the offset. A hybrid "
        "model also interpolates the remaining drift between visits in time, and the software "
        "keeps whichever model best predicts held-out visits. The correction is applied only to "
        "the report and never fed back to PX4, so it cannot cause a jump in flight.")


def sec_params(R: Report):
    R.h1("4  Important Parameters and Implementation Decisions")
    R.table("params", "Key parameters (Run 1, the full search) and the reasons for them.",
            ["Parameter", "Value", "Reason"],
            [["Flight height", "0.8 m", "1.45 m swath; smaller relative rangefinder error"],
             ["Lane spacing (max) / margin", "1.0 m / 0.3 m", "Four lanes, 0.6 m overlap"],
             ["Speed limit `MPC_XY_VEL_MAX`", "0.5 m/s", "≥ 5 frames per tag; tracking"],
             ["Corner acceptance radius", "15 cm", "Smooth corners without cutting lanes"],
             ["Mission tick / setpoint rate", "10 Hz / 20 Hz", "Laptop logic / on-board player"],
             ["Detector gate", "margin ≥ 55, ≤ 1 bit, ≤ 1.5 m, ≤ 30°/s",
              "Reject weak, distant or blurred decodes"],
             ["Pose match window", "100 ms", "Reject frames with no pose"],
             ["Tag confirmation", "5 sightings within 0.2 m", "No false confirmations"],
             ["Hover check", "10 cm for 2 s", "Stable before the sweep"],
             ["Precision landing", "8 cm, ≤ 3 moves, ≤ 10 s", "Land on the start box"],
             ["Geofence / link time-out", "0.5 m / 10 s", "Automatic landing"],
             ["IMU priority", "BMI088 highest, BMI270 lowest", "Avoid estimator switches (§6)"]],
            widths_cm=[5.0, 5.0, 6.0])
    R.p("The main implementation decisions were:")
    R.bullets([
        "**Fly the sweep on board**, not by streaming waypoints from the laptop, so that Wi-Fi "
        "cannot disturb the path.",
        "**Waypoint mode** for the setpoints. A fixed look-ahead point and a velocity "
        "feed-forward follower were also built and flown. Waypoint mode was the fastest and "
        "smoothest, and the shorter flight left less time for drift.",
        "**Detect on the laptop**, with each frame carrying its own pose, so that the full "
        "frame rate is available and latency cannot bias the result.",
        "**Localisation tags instead of UWB.** A teammate trialled a LinkTrack UWB system [6] "
        "fused into PX4 as an external position source. It worked, with 5–7 cm static scatter, "
        "but it sometimes pulled the drone sideways near the far end. In a 5 m × 3 m arena the "
        "flow drift over a one-minute flight (10–25 cm) is similar to the UWB error, so the gain "
        "was insignificant and UWB was left out.",
    ])


def sec_results(R: Report):
    R.h1("5  Experimental Results")
    R.p(
        "Two demonstration flights from 11 November 2026 are reported. Run 1 was the full "
        "search at 0.5 m/s. Run 2 flew only the two outer lanes, with the speed limit raised to "
        "1.5 m/s to find the tags faster. Both completed without steering or a safety trigger, "
        f"and both confirmed the same three search tags with no false tags ({T('summary')}). "
        "Mission time runs from the take-off command to the land command. Positions are PX4 "
        "estimates unless stated as corrected.")
    sight = lambda r: f"{min(t['n_sightings'] for t in r['tags'])}–{max(t['n_sightings'] for t in r['tags'])}"
    runs = (R1, R2)
    rows = [
        ["Lanes (spacing)"] + [f"{r['n_lanes']} ({2.62 / (r['n_lanes'] - 1):.2f} m)" for r in runs],
        ["Speed limit / median lane speed (m/s)"] + [
            f"{r['mpc_xy_vel_max']:.1f} / {r['speed_cruise_median_mps']:.2f}" for r in runs],
        ["Sweep duration (s)"] + [f"{r['sweep_duration_s']:.1f}" for r in runs],
        ["All tags confirmed, from sweep start (s)"] + [f"{r['all_tags_after_sweep_start_s']:.1f}" for r in runs],
        ["Mission time, take-off to land command (s)"] + [f"{r['mission_time_s']:.1f}" for r in runs],
        ["Cross-track error RMS / worst overshoot (cm)"] + [
            f"{r['xt_rms_all_m'] * 100:.1f} / {r['overshoot_max_m'] * 100:.1f}" for r in runs],
        ["Sightings per search tag"] + [sight(r) for r in runs],
        ["Localisation fixes / fitted scale"] + [
            f"{r['loc_fit']['n_fixes']} / {r['loc_fit']['scale']:.3f}" for r in runs],
        ["Home-tag offset on arrival / at landing (cm)"] + [
            f"{r['land_first_offset_cm']:.0f} / {r['land_final_offset_cm']:.0f}" for r in runs],
    ]
    R.table("summary", "Results of the two demonstration flights.",
            ["", "Run 1 (full)", "Run 2 (truncated)"], rows,
            widths_cm=[8.4, 3.8, 3.8], align=["l", "c", "c"])
    R.figure("path1", FIG / "fig_path_run1.png",
             "Run 1 (full sweep, 0.5 m/s). Left: PX4 frame, with the plan, raw tags and "
             "localisation-tag visits. Right: drift-corrected route and tags.", width_cm=12.0)
    pct = lambda r: 100 * r["speed_cruise_median_mps"] / r["mpc_xy_vel_max"]
    R.p(
        f"**Speed and tracking.** The median lane speed was {pct(R1):.0f} % of the limit in "
        f"Run 1 and {pct(R2):.0f} % in Run 2. At 1.5 m/s the 4.6 m lanes were too short to reach "
        f"a steady cruise. The cross-track error grew from {cm(R1['xt_rms_all_m'])} to "
        f"{cm(R2['xt_rms_all_m'])} RMS, and the worst lane-end overshoot from "
        f"{cm(R1['overshoot_max_m'])} to {cm(R2['overshoot_max_m'])}. At speed the error is "
        f"mostly a bias towards the outside of the sweep: the drone enters each lane with "
        f"sideways momentum, and Eq. (3) leaves little authority to remove it. The heading "
        f"stayed within ±2°.")
    R.p(
        f"**Detection.** Each tag was confirmed 0.3–0.5 s after it first appeared (five frames "
        f"at 7–10 frames/s). The measured range of 0.70–0.86 m agrees with the 0.8 m flight "
        f"height. Sightings per tag fell with speed, from {sight(R1)} to {sight(R2)}. In Run 2, "
        f"tag 4 was confirmed with one sighting to spare, close to the camera's limit.")
    rows = []
    for tid in ("2", "4", "6"):
        cells = [tid]
        for r in runs:
            t = next(t for t in r["tags"] if str(t["tag_id"]) == tid)
            cells.append(f"({t['corr_x']:.2f}, {t['corr_y']:.2f})")
        d = DISP[tid]
        cells.append(f"{d['raw']['max_pairwise_m'] * 100:.1f} → {d['corr']['max_pairwise_m'] * 100:.1f}")
        rows.append(cells)
    R.table("positions", "Reported tag positions (x, y) in metres after drift correction, and "
            "the distance between the two runs' estimates before → after correction.",
            ["Tag", "Run 1", "Run 2", "Run 1 to Run 2 (cm)"], rows,
            widths_cm=[1.6, 4.4, 4.4, 5.6], align=["c"] * 4)
    R.p(
        f"**Localisation.** The tags did not move between flights but the drift differed, so the "
        f"distance between the two runs' estimates measures repeatability. Correction brought "
        f"the estimates of tags 2 and 4 within {cm(DISP['2']['corr']['max_pairwise_m'])} and "
        f"{cm(DISP['4']['corr']['max_pairwise_m'])} of each other ({T('positions')}). Before "
        f"correction, PX4 was off by {cm(R1['loc_raw_err_mean_m'])} on average at the "
        f"localisation tags in Run 1. Run 1's nine fixes gave a scale of "
        f"{R1['loc_fit']['scale']:.3f}, i.e. PX4 over-estimated distance by about 2.2 %. Run 2's "
        f"three fixes barely determine the four-parameter fit, and its scale of "
        f"{R2['loc_fit']['scale']:.3f} is implausible. This is why tag 6 still differs by "
        f"{cm(DISP['6']['corr']['max_pairwise_m'])}. On return the home tag was "
        f"{R1['land_first_offset_cm']:.0f} and {R2['land_first_offset_cm']:.0f} cm from where PX4 "
        f"placed it. Precision landing reduced this to {R1['land_final_offset_cm']:.0f} and "
        f"{R2['land_final_offset_cm']:.0f} cm.")
    R.todo("Absolute error of each tag against its surveyed demonstration position, and the "
           "number of search tags in the area.")
    R.p(
        f"**Speed versus coverage.** Run 2 confirmed all tags "
        f"{R2['all_tags_after_sweep_start_s']:.1f} s into the sweep, against "
        f"{R1['all_tags_after_sweep_start_s']:.1f} s, and cut the mission time from "
        f"{R1['mission_time_s']:.1f} s to {R2['mission_time_s']:.1f} s. However, its two lanes "
        f"2.62 m apart never image a central strip about 1.2 m wide. It found every tag only "
        f"because these tags lay near the outer lanes, so only Run 1 is a complete search.")


def sec_ai(R: Report):
    R.h1("6  AI-Assisted Flight-Data Analysis and Visualisation")
    R.p(
        "Every flight produces two logs. The laptop records events, the 10 Hz telemetry route "
        "and every detection. PX4's onboard log (ULog) records estimator, controller and sensor "
        "data at up to several hundred hertz. We used an AI coding assistant "
        "[TO CONFIRM: tool name] to write Python analysis scripts (pandas, matplotlib, pyulog). "
        "These computed the metrics and figures in Section 5: lane-by-lane cross-track error, "
        "speed, sightings, drift fits and run-to-run differences. The team checked each result "
        "against what was seen in flight. This checking mattered: the first speed estimate "
        "showed 0.62 m/s under a 0.5 m/s limit, an artefact of differentiating jittery Wi-Fi "
        "timestamps that smoothing removed. Similarly, an automatic accuracy plot was found to "
        "use reference tag positions from an earlier layout, so it was left out.")
    R.p(
        "**Case study: sudden odometry jumps.** In several trial flights the reported position "
        "jumped by metres, and the drone lunged or yawed violently. The ULogs of two such "
        "flights (A and B) showed this was not a sensor fault. The flight controller has two "
        "IMUs, a BMI088 and a BMI270. With PX4's default setting (`EKF2_MULTI_IMU` = 3) it runs "
        "one EKF2 instance per IMU and publishes whichever it rates healthier. With no compass, "
        "nothing measures yaw, so each instance integrates its own gyroscope. In flight the "
        "BMI270 read about 2°/s less yaw rate than the BMI088 (−2.5°/s in A, −1.8°/s in B), "
        "while both read near zero at rest. This suggests a vibration-related error. Because "
        "optical flow is rotated into the world frame using each instance's own heading, the two "
        f"estimates diverged by 105° and 5.4 m in A, and by over 200° and 6.9 m in B "
        f"({F('odom_div')}). Each stayed self-consistent, so PX4's health checks saw nothing "
        "wrong.")
    R.figure("odom_div", FIG / "fig_odom_divergence.png",
             "Heading (top) and position (bottom) divergence between the two estimator instances "
             "in trial flights A and B, until PX4 switched between them.", width_cm=11.5)
    R.p(
        "A 60 ms innovation spike on the healthy instance (descent onset in A, touchdown in B) "
        "made PX4 switch to the drifted one. It then published the whole difference as an "
        "instant reset: 5.3 m and −106° in A. A was airborne and its setpoints were still in "
        "the old frame, so the drone yawed at up to 94°/s until an automatic landing and the "
        "kill switch ended the flight. For the demonstration we gave the BMI088 the highest "
        "sensor priority and the BMI270 the lowest, so that PX4 falls back to the BMI270 only if "
        "the BMI088 fails. No jump occurred in any demonstration flight. The blame on "
        "the BMI270 rests on both IMUs sharing one rigid board and has not been checked against "
        "an external heading reference.")
    R.todo("Analysis of the demonstration ULogs: the cause of the lane bias, tilt and motor "
           "margins at 1.5 m/s, and confirmation that no estimator switch occurred.")


def sec_limits(R: Report):
    R.h1("7  Limitations, Failure Cases and Improvements")
    R.bullets([
        "**Coverage versus speed.** The fast two-lane sweeps miss the centre of the area. "
        "Three lanes at the full 1.4 m swath and about 0.7 m/s would keep full coverage and "
        "cut the mission time by an estimated third [TO CONFIRM: not yet flown].",
        "**Lane-start bias** of up to 27 cm at 1.5 m/s (Eq. 3). Fix: a short velocity "
        "feed-forward segment after each corner, or an intermediate setpoint on the lane line.",
        "**Detection margin at speed.** Only 6 sightings for one tag in Run 2. Fix: a higher "
        "camera frame rate, or a lower speed.",
        "**Sparse drift fixes.** Run 2's three fixes gave an unreliable correction. Fix: place "
        "localisation tags under the lanes.",
        "**Systematic flow scale error** of 2.2 %. Fix: calibrate the flow sensor's scale.",
        "**Precision-landing hold.** The 8 cm radius is close to the hover wobble, so the drone "
        "corrected until the 10 s limit in both runs. A 10 cm radius would save about 5 s.",
        "**Estimator switching (failure case, Section 6).** The robust fix is a single "
        "estimator instance on the BMI088 (`EKF2_MULTI_IMU` = 0) plus a heading reference, and "
        "companion software that re-anchors its setpoints when PX4 reports a position reset.",
        "**No ground truth.** All positions are PX4 estimates corrected by the localisation "
        "tags. Two flights are too few for statistics, though every trend is consistent and "
        "explained.",
    ])


def sec_note(R: Report):
    R.h1("8  Note to the Teaching Team")
    R.p(
        "We saw severe odometry drift and sudden position and velocity snaps many times in "
        "trials. For a long time we blamed tuning or our own code. Section 6 shows that at "
        "least two came from the flight controller's default configuration. With two IMUs, no "
        "compass and PX4's default `EKF2_MULTI_IMU` = 3, PX4 can silently switch to an estimate "
        "whose heading has drifted by 100° or more. Nothing in QGroundControl or the telemetry "
        "warns of this. Other groups on the same board are probably exposed and may mistake it "
        "for a control bug. We suggest that the course baseline either run a single estimator "
        "on the BMI088 or set the IMU priorities as we did, and that the companion template "
        "handle PX4's position-reset counter.")


def sec_conclusion(R: Report):
    R.h1("9  Conclusion")
    R.p(
        "The system took off, searched, localised the tags, returned and landed on its marker "
        "in every demonstration flight, with no false detections and no intervention. "
        "Four decisions mattered most: flying the sweep on board; stamping each frame with its "
        "pose; robust five-sighting confirmation; and surveyed localisation tags for "
        "alignment, landing and drift correction. Higher speed roughly halved the mission time, "
        "but cost coverage, tracking accuracy and detection margin. For a search with unknown "
        f"tag positions, full coverage at moderate speed is the better trade. Source code: "
        f"{REPO_URL}.")


def sec_refs(R: Report):
    R.h1("References")
    refs = [
        "E. Olson, \"AprilTag: A robust and flexible visual fiducial system,\" in *Proc. IEEE "
        "ICRA*, 2011, pp. 3400–3407.",
        "J. Wang and E. Olson, \"AprilTag 2: Efficient and robust fiducial detection,\" in "
        "*Proc. IEEE/RSJ IROS*, 2016, pp. 4193–4198.",
        "S. Umeyama, \"Least-squares estimation of transformation parameters between two point "
        "patterns,\" *IEEE Trans. PAMI*, vol. 13, no. 4, pp. 376–380, 1991.",
        "PX4 Autopilot, *PX4 User Guide (v1.15)*. https://docs.px4.io/v1.15/",
        "MAVLink, *MAVLink Developer Guide*. https://mavlink.io/",
        "Nooploop, *LinkTrack P-A Series User Manual*.",
    ]
    for i, ref in enumerate(refs, 1):
        para = R.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.LEFT
        para.paragraph_format.left_indent = Cm(0.9)
        para.paragraph_format.first_line_indent = Cm(-0.9)
        para.paragraph_format.space_after = Pt(2)
        r = para.add_run(f"[{i}]\t")
        set_font(r)
        add_rich(para, ref)


def main():
    R = Report()
    title_page(R)
    sec_summary(R)
    sec_architecture(R)
    sec_navigation(R)
    sec_detection(R)
    sec_params(R)
    sec_results(R)
    sec_ai(R)
    sec_limits(R)
    sec_note(R)
    sec_conclusion(R)
    sec_refs(R)
    R.save(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
