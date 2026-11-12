#!/usr/bin/env python3
"""
Build the CDE1302 final report (Word .docx) from the analysed flight data.

    python report/analyse_flights.py     # first: figures + data/metrics.json
    python report/build_report.py        # then:  CDE1302_Final_Report_DRAFT.docx

Formatting (set by the team): academic formal, Times New Roman, no text
smaller than 12 pt.  Unfinished items are marked in the text as
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
R1, R2, R3 = METRICS["runs"]
DISP = METRICS["dispersion"]

FONT = "Times New Roman"
MONO = "Courier New"
BODY_PT = 12
TEXT_WIDTH_CM = 15.9          # A4 minus 2 x 2.54 cm margins

# ---------------------------------------------------------------------------
# Figure / table numbering (declared up front so the text can cite them)
# ---------------------------------------------------------------------------
FIGS = ["drone", "arch", "control", "states", "lawnmower", "projection", "livemap",
        "timeline", "path1", "path2", "path3", "speed", "sightings", "repeat", "loc",
        "ulog_track", "ulog_att", "ulog_ekf", "frame", "frame_sim", "wiring", "uwb_setup"]
TABS = ["hw", "links", "states", "safety", "gate_modes", "filters", "loc_params",
        "runs", "timeline", "tracking", "tags", "positions", "locfit", "landing",
        "ulog_params", "ulog_plan", "frame", "uwb_params", "uwb_cmp", "config"]


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
            set_font(r, size, mono=True, bold=base_bold)
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
        pf.line_spacing_rule = WD_LINE_SPACING.ONE_POINT_FIVE
        pf.space_after = Pt(6)
        pf.space_before = Pt(0)
        for name, size, before, after, italic in (("Heading 1", 16, 24, 12, False),
                                                  ("Heading 2", 14, 18, 6, False),
                                                  ("Heading 3", 12, 12, 6, True)):
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
    def h1(self, text, page_break=True):
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

def title_page(R: Report):
    for _ in range(4):
        R.p("", space_after=0)
    for text, size, bold in (("NATIONAL UNIVERSITY OF SINGAPORE", 14, True),
                             ("CDE1302 Introduction to Unmanned Aerial Vehicles", 14, False),
                             ("Academic Year 2026/27", 12, False)):
        para = R.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = para.add_run(text)
        set_font(r, size, bold=bold)
    R.p("", space_after=24)
    para = R.doc.add_paragraph()
    para.alignment = WD_ALIGN_PARAGRAPH.CENTER
    para.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
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
    R.p("", space_after=36)
    R.p("Group [TO BE COMPLETED: group number]  ·  Drone ID 2", align="center")
    R.table(None, "Team members", ["Name", "Matriculation number", "Primary responsibilities"],
            [["[TO BE COMPLETED: name]", "[TO BE COMPLETED]", "[TO BE COMPLETED]"]] * 4,
            widths_cm=[5.5, 4.5, 6.0], label="")
    R.p("", space_after=24)
    R.p("Submitted: [TO BE COMPLETED: date]", align="center")


def front_matter(R: Report):
    R.h1("Abstract", page_break=True)
    R.p(
        "This report documents the design, implementation and flight demonstration of a "
        "pseudo-autonomous indoor quadcopter that searches an allocated rectangular area for "
        "AprilTag fiducial markers (tag16h5 family) on the floor, returns to its take-off point, "
        "lands, and reports the position of every tag found together with a reconstruction of the "
        "route flown. The vehicle carries a PX4 flight controller that estimates its position from "
        "an optical-flow sensor and a downward rangefinder only, and a Seeed XIAO ESP32-S3 Sense "
        "companion computer that bridges PX4 to a ground laptop over Wi-Fi. The laptop plans a "
        "boustrophedon (lawnmower) sweep, uploads it as one trajectory that the companion computer "
        "executes on board, and detects tags in a JPEG stream in which each frame carries the PX4 "
        "pose at the instant of capture. Tag sightings are projected into the PX4 local frame, "
        "clustered robustly and confirmed after five consistent sightings. Five additional "
        "localisation tags at surveyed positions are used to square the sweep to the arena before "
        "take-off, to land precisely on the home marker, and to correct the reported tag positions "
        "for odometry drift after the flight.")
    R.p(
        f"Three demonstration flights were flown. A complete four-lane sweep with the PX4 horizontal "
        f"speed limit at 0.5 m/s located three search tags (IDs 2, 4 and 6) and "
        f"completed the mission in {R1['t_done_s']:.1f} s. Two shortened two-lane sweeps with the "
        f"limit raised to approximately 1.0 and 1.5 m/s located the same three tags and completed in "
        f"{R2['t_done_s']:.1f} s and {R3['t_done_s']:.1f} s. Faster flight reduced the time to locate "
        f"all tags from {R1['all_tags_after_sweep_start_s']:.1f} s to "
        f"{R3['all_tags_after_sweep_start_s']:.1f} s after the start of the sweep, but increased the "
        f"root-mean-square cross-track error from {cm(R1['xt_rms_all_m'])} to {cm(R3['xt_rms_all_m'])}, "
        f"reduced the number of sightings per tag by a factor of about three, and left fewer "
        f"localisation fixes for drift correction. Drift correction reduced the run-to-run "
        f"dispersion of the reported tag positions from {cm(DISP['2']['raw']['rms_from_mean_m'])}–"
        f"{cm(DISP['6']['raw']['rms_from_mean_m'])} to {cm(DISP['4']['corr']['rms_from_mean_m'])}–"
        f"{cm(DISP['6']['corr']['rms_from_mean_m'])} RMS. Work carried out beyond the mission "
        f"requirements, namely a redesigned 3D-printed frame, the electrical integration and a trial "
        f"of ultra-wideband (UWB) positioning, is documented separately and marked [EXTRAS].")
    R.todo("Absolute tag-position accuracy against the measured demonstration layout will be "
           "added once the surveyed positions of the search tags used on demonstration day are "
           "available (Section 4.6). The PX4 black-box log analysis (Section 5) is pending the "
           "download of the onboard .ulg files.")

    R.h1("Table of Contents", page_break=True)
    para = R.doc.add_paragraph()
    add_field(para, 'TOC \\o "1-3" \\h \\z \\u',
              "Right-click here and choose 'Update Field' to build the table of contents.")


def sec_intro(R: Report):
    R.h1("1  Introduction")
    R.h2("1.1  Background and mission brief")
    R.p(
        "The CDE1302 mini project requires each group to make a small indoor quadcopter carry out "
        "a search task with minimal human intervention. An allocated rectangle on the floor of the "
        "flight arena contains a number of AprilTags [1], [2] of the tag16h5 family, printed with a "
        "120 mm black square. The drone must take off from a start box, explore the rectangle, "
        "return to the start box and land. After the flight the group reports the position of each "
        "tag found and reconstructs the route the drone flew. The arena has no external positioning "
        "system, and the drone has neither GPS nor a magnetometer, so every position the drone knows "
        "comes from integrating its own sensors.")
    R.p(
        "Our system is *pseudo-autonomous*: a laptop plans the search and detects tags in a video "
        "stream received over Wi-Fi, while the drone flies the uploaded path by itself. A human "
        "operator starts the mission with two typed commands (`takeoff`, then `start`) and holds "
        "the radio-control (RC) kill switch throughout, but does not steer the vehicle.")
    R.h2("1.2  Objectives")
    R.bullets([
        "**Coverage:** fly a sweep that brings every point of the allocated area into the "
        "camera's field of view at least once.",
        "**Detection:** detect every search tag in the area with no false confirmations.",
        "**Localisation:** report each tag's position in a frame tied to the arena, as accurately "
        "as the drifting odometry allows.",
        "**Return and landing:** return to the start box and land on it without operator input.",
        "**Traceability:** log every event, sighting and pose so that the route and the tag "
        "estimates can be reconstructed and audited after the flight.",
        "**Safety:** abort to a landing on loss of link, geofence breach or time-out, with the RC "
        "kill switch as the final authority.",
    ])
    R.h2("1.3  Scope and structure of this report")
    R.p(
        "Sections 2 and 3 describe the system and explain how the mission software works, module by "
        "module. Section 4 presents and analyses the three demonstration flights using the "
        "laptop-side mission logs. Section 5 is reserved for the analysis of the PX4 black-box "
        "(ULog) recordings of the same flights. Section 6 documents work that was carried out "
        "in addition to the mission requirements: the redesigned frame, the electrical "
        "integration and the UWB positioning trial. These sections are marked **[EXTRAS]**. "
        "None of them was required for the mission, and the UWB system was not used in the "
        "demonstration flights. Section 7 concludes the report.")


def sec_system(R: Report):
    R.h1("2  System Overview")
    R.h2("2.1  Vehicle platform")
    R.p(
        "The vehicle is the course quadcopter, rebuilt on a frame that the group redesigned for "
        "3D printing (Section 6.1). Its avionics are summarised in "
        f"{T('hw')}. Position is estimated by PX4's EKF2 from the inertial measurement unit (IMU) "
        "and an MTF-02P module that combines a downward optical-flow camera with a time-of-flight "
        "rangefinder. Because the vehicle has no compass, the heading is integrated from the gyroscope "
        "and the axes of the PX4 local frame are fixed by the direction of the nose when EKF2 starts.")
    R.table("hw", "Avionics and sensors used in the mission.",
            ["Component", "Part", "Role in the mission"],
            [["Flight controller", "MicoAir H743 V2, PX4 v1.15.4",
              "Attitude, velocity and position control in OFFBOARD mode; EKF2 state estimation"],
             ["Optical flow and range", "MTF-02P",
              "Horizontal velocity (flow) and height above the floor (rangefinder)"],
             ["Companion computer", "Seeed XIAO ESP32-S3 Sense",
              "MAVLink bridge to PX4 (UART, 921 600 baud), Wi-Fi link, on-board trajectory "
              "player, camera streaming"],
             ["Camera", "OV3660 on the XIAO Sense, facing down",
              "320 × 240 grey-scale frames, JPEG-encoded and streamed to the laptop"],
             ["Ground station", "Windows laptop, Python 3",
              "Mission supervisor, planner, tag detection, logging and live map"],
             ["Network", "Dedicated 2.4 GHz router",
              "Drone 192.168.1.202, laptop 192.168.1.102 (static addresses)"],
             ["Safety", "RC transmitter", "Kill switch held by the operator at all times"]],
            widths_cm=[3.6, 4.8, 7.6])
    R.image_placeholder("drone", "Photograph of the assembled drone, camera side visible.",
                        "The demonstration vehicle.")
    R.h2("2.2  Three-tier software architecture")
    R.p(
        "Control is split across three computers, each doing what it is best placed to do "
        f"({F('arch')}). The **laptop** runs the mission logic, the search planner, the AprilTag "
        "detector and the logging, at a 10 Hz supervisory rate. The **ESP32-S3** runs FreeRTOS "
        "firmware that owns the MAVLink link to PX4. It arms the vehicle, takes off and lands, "
        "plays the uploaded trajectory, streams 20 Hz position setpoints to PX4 and streams camera "
        "frames to the laptop. **PX4** closes the cascaded position, velocity, attitude and rate "
        "loops. This split means that a Wi-Fi drop does not interrupt the flight path. Once a "
        "trajectory has been uploaded, the ESP32 plays it from PX4's position, which it reads over "
        "the UART link, without waiting for the laptop.")
    R.image_placeholder(
        "arch", "System block diagram (laptop ⇄ router ⇄ ESP32-S3 ⇄ PX4, camera and sensors). "
        "Source: the Mermaid diagram 'System overview' in the repository README.md.",
        "System architecture and data flow.")
    R.image_placeholder(
        "control", "Cascaded control diagram (laptop 10 Hz state machine → ESP32 trajectory gate "
        "→ PX4 position/velocity/attitude/rate loops). Source: 'Control architecture' in README.md.",
        "Control architecture across the three computers.")
    R.h2("2.3  Communication links")
    R.table("links", "Communication links and their contents.",
            ["Link", "Transport", "Content"],
            [["Laptop → ESP32 (port 5006)", "UDP",
              "Commands: `CMD_START`, `CMD_GOTO`, `CMD_TRAJ_DATA`, `CMD_TRAJ_START`, `CMD_HOLD`, "
              "`CMD_LAND`, camera keep-alive"],
             ["ESP32 → laptop (port 5005)", "UDP, 10 Hz",
              "Telemetry: PX4 local position, heading, firmware navigation state"],
             ["ESP32 → laptop (port 5009)", "UDP, ≈7–10 frames/s",
              "JPEG frames in ≤1400-byte chunks, each frame carrying the PX4 pose at capture"],
             ["ESP32 ⇄ PX4", "UART 921 600 baud, MAVLink",
              "`SET_POSITION_TARGET_LOCAL_NED` at 20 Hz; `LOCAL_POSITION_NED`, heartbeat, "
              "attitude from PX4"],
             ["ESP32 ⇄ QGroundControl (14550 / 8888)", "UDP MAVLink bridge",
              "Parameter access and gyroscope calibration through the ESP32"],
             ["RC transmitter → PX4", "Radio", "Kill switch"]],
            widths_cm=[4.6, 3.6, 7.8])
    R.h2("2.4  Coordinate frames and conventions")
    R.p(
        "Three right-handed frames are used. The **PX4 local frame** is North-East-Down (NED) in "
        "metres, with its origin where EKF2 started (the take-off spot). Because there is no compass, "
        "its x-axis points wherever the nose pointed at power-up. The **body frame** has x forward, "
        "y right and z down. The **arena frame** has its origin at the centre of the home "
        "localisation tag (ID 29) in the start box, x along the length of the search area and y to "
        "the right. The floor of the arena is tiled on a 0.46 m grid, so a grid node (a, b) lies at "
        "x = 0.46 b, y = 0.46 a. The allocated search area spans x = 0.92–4.60 m and "
        "y = −1.84–1.38 m in this frame. Heading is measured in radians from North, clockwise "
        "positive, and is held constant throughout the flight.")


def sec_software(R: Report):
    R.h1("3  Software Design and Implementation")
    R.p(
        "The mission software lives in `cde1302/mission/` (laptop, Python) and `cde1302/main/` "
        "(ESP32 firmware, C). The laptop code builds on the course-provided helpers in "
        "`cde1302/laptop/` (UDP protocol, camera stream reassembly and an AprilTag detector compiled "
        "from the same C sources as the firmware), which are imported read-only. This section "
        "follows the data from the operator's command to the final report.")

    # 3.1
    R.h2("3.1  Mission supervisor and state machine")
    R.p(
        "`mission.py` is the entry point. It loads and validates `config.yaml`, connects to the "
        "drone, starts the camera pipeline and then calls a non-blocking `Mission.tick()` at 10 Hz. "
        f"Each tick runs the handler of the current phase ({T('states')}) and the safety checks. "
        "Every transition and decision is written to `events.csv` with wall-clock and mission time, "
        "so the behaviour of a flight can be reconstructed exactly.")
    R.table("states", "Mission phases and their exit conditions.",
            ["Phase", "What happens", "Exit condition"],
            [["CONNECTING", "Wait for telemetry from the drone", "First telemetry packet"],
             ["READY", "On the ground; operator may `calibrate` the gyroscopes",
              "Operator types `takeoff` (sends `CMD_START`)"],
             ["TAKING_OFF", "ESP32 switches PX4 to OFFBOARD, arms and climbs to 0.8 m",
              "A probe `CMD_GOTO` to the current position is answered with ARRIVED"],
             ["HOVERING", "Hover is checked; home tag 29 is observed",
              "Horizontal position within 0.10 m for 2 s, then operator types `start`"],
             ["EXPLORING", "Plan aligned to the home tag, uploaded and flown; tags collected",
              "Firmware reports the trajectory finished"],
             ["RETURNING", "Return trajectory to the home tag; precision-landing hold; then the "
              "tag report and drift fit are written",
              "Within 8 cm of the home tag, or 10 s after arriving"],
             ["LANDING", "`CMD_LAND` sent three times", "6 s settling time"],
             ["DONE", "Correction plot and corrected route saved", "—"],
             ["ABORT_LANDING", "Entered from any airborne phase on a safety trigger", "Landed"]],
            widths_cm=[3.2, 7.0, 5.8])
    R.p(
        "Hover cannot be confirmed from altitude because the telemetry packet carries none. "
        "Instead, the laptop sends a `CMD_GOTO` to the drone's own position. The firmware only "
        "answers ARRIVED once its take-off phase is complete, which proves the vehicle is airborne "
        "and under offboard control. The laptop additionally requires the horizontal position to "
        "stay within `hover_radius_m` = 0.10 m for `hover_settle_s` = 2 s.")
    R.image_placeholder("states", "State diagram of the mission phases (CONNECTING → … → DONE, "
                        "with ABORT_LANDING from every airborne phase).",
                        "Mission state machine.")

    # 3.2
    R.h2("3.2  Safety layers")
    R.p(f"Several independent mechanisms can end a flight. They are listed in {T('safety')}, from "
        "the most automatic to the most authoritative.")
    R.table("safety", "Safety mechanisms.",
            ["Layer", "Trigger", "Action"],
            [["Laptop geofence", "Position outside the area, home and planned path grown by 0.5 m",
              "ABORT_LANDING"],
             ["Laptop stale-link check", "No telemetry for 10 s", "ABORT_LANDING"],
             ["Laptop time limit", "Mission time > 600 s", "Stop searching, return and land"],
             ["Trajectory watchdog", "Trajectory not started after 20 uploads, or not finished "
              "within 4 × the planned time + 10 s", "Abort or return"],
             ["Operator", "`land` command or Ctrl+C", "Land immediately"],
             ["Firmware Wi-Fi kill switch", "ESP32 loses the router for more than ≈3 s",
              "Motors stopped"],
             ["Firmware setpoint watchdog", "Velocity-type setpoint not refreshed",
              "Position hold"],
             ["RC kill switch", "Operator", "Motors stopped (final authority)"]],
            widths_cm=[4.2, 7.0, 4.8])

    # 3.3
    R.h2("3.3  Search-pattern planning")
    R.p(
        "`planner.py` generates a boustrophedon (lawnmower) sweep: parallel lanes along the length of "
        "the area (`lanes_along: x`), flown in alternate directions. The area is first shrunk by a "
        "margin m = 0.30 m on every side, so that the waypoints stay clear of the boundary. For a "
        "shrunk width W and a maximum lane spacing s_max the number of lanes and the actual spacing are")
    R.equation("n = ⌈W / s_max⌉ + 1,    s = W / (n − 1)", 1)
    R.p(
        "so that the outer lanes lie exactly on the shrunk boundary and no gap exceeds s_max. With "
        "W = 2.62 m, the full sweep (s_max = 1.0 m) has four lanes 0.873 m apart and eight waypoints. "
        "The truncated sweeps (s_max = 3.0 m) have only the two outer lanes, 2.62 m apart, and four "
        "waypoints.")
    R.p(
        "The lane spacing must not exceed the width of the strip in which a tag can be decoded. With "
        "the course's lens model (f_x = 163.5 px, f_y = 153.2 px at 320 × 240 px) and the camera "
        "looking straight down from h = 0.8 m, the ground footprint is")
    R.equation("w_across = h · 320 / f_x ≈ 1.57 m,    w_along = h · 240 / f_y ≈ 1.25 m", 2)
    R.p(
        "A tag is decoded only when the whole 112 mm black square is inside the image, which leaves "
        "an effective swath of about 1.45 m. The full sweep therefore overlaps adjacent lanes by "
        "about 0.6 m and covers the whole area with margin. The two-lane sweep leaves a central "
        "strip, roughly 1.2 m wide, that the camera does not see along the lanes. That strip is "
        "only crossed by the short transit legs at the two ends (Section 4.9).")
    R.image_placeholder("lawnmower", "Planner output for the full and the truncated sweep "
                        "(`python mission/planner.py --plot`), with the camera footprint drawn "
                        "along each lane.", "Planned lawnmower sweeps.")

    # 3.4
    R.h2("3.4  Trajectory generation and upload")
    R.p(
        "In the configuration used for the demonstration (`flight.nav_mode: trajectory`), the sweep "
        "is not commanded waypoint by waypoint from the laptop. `trajectory.py` samples the polyline "
        "from the drone's current position through all waypoints at a constant speed "
        "(`cruise_speed_mps` = 0.30 m/s) every 0.20 s. It converts the samples into offsets from "
        "the start point, and `CMD_TRAJ_DATA` packets upload them to the ESP32, followed by "
        "`CMD_TRAJ_START`. The full sweep was 282 samples, well below the firmware limit of 2400. "
        "The laptop confirms the start from the firmware's navigation state (TRAJ) and re-uploads "
        "if it is not seen within 2 s. The return home and every precision-landing correction are "
        "uploaded the same way as short trajectories.")
    R.p(
        "Before upload, the plan, written in the arena frame, is rotated and translated into the PX4 "
        "frame using the home tag observed during the hover (Section 3.9). The heading is never "
        "changed in flight: the drone flies the lanes sideways and backwards as required. This avoids "
        "yaw manoeuvres, which would add gyroscope drift and motion blur.")

    # 3.5
    R.h2("3.5  On-board trajectory execution (trajectory gate)")
    R.p(
        "On the ESP32, `traj_gate.c` decides, at every control tick, where along the uploaded path the "
        "position setpoint should be. It is written in plain C99 with no ESP-IDF dependencies, so the "
        "identical source is compiled and unit-tested on the laptop. Three modes were implemented "
        f"and flight-tested ({T('gate_modes')}). None of them has a clock: the target only advances "
        "when the drone does, so a slow or disturbed drone still flies every lane.")
    R.table("gate_modes", "Trajectory-gate modes (firmware menuconfig).",
            ["Mode", "Setpoint sent to PX4", "Observed behaviour"],
            [["Carrot (lookahead)", "Point 25 cm ahead of the drone along the path",
              "Steady speed of about 0.33 m/s, but slows into every corner and surges after it"],
             ["Follow (velocity feed-forward)",
              "Point just ahead of the drone's progress plus a feed-forward velocity along the "
              "path, ramped at corners",
              "Designed for tight cross-track tracking; in flight slower and less smooth than "
              "waypoint mode"],
             ["**Waypoint (used)**", "The next corner of the path, held until the drone is within "
              "15 cm", "Fastest and smoothest; PX4 accelerates up to `MPC_XY_VEL_MAX` on each "
              "lane"]],
            widths_cm=[3.8, 6.2, 6.0])
    R.p(
        "Follow mode was developed in response to an analysis of an earlier PX4 log (Section 5.2). "
        "That analysis showed that a distant position step limits the cross-track correction, "
        "because PX4 saturates the horizontal velocity demand by its *norm*. With an along-track "
        "error e_a, a cross-track error e_c and a speed limit v_max, the corrective velocity "
        "across the lane is only")
    R.equation("v_c ≈ v_max · e_c / √(e_a² + e_c²)", 3)
    R.p(
        "This is about 0.03 m/s for e_c = 0.35 m at the start of a 4.8 m lane with v_max = 0.4 m/s. "
        "In back-to-back flights on 11 November 2026, however, waypoint mode with 15 cm corner "
        "radii proved faster, smoother and in practice more accurate: shorter flights accumulate "
        "less gyroscope and flow drift. It was therefore used for all three demonstration "
        "flights (branch `feature/loc-tags-waypoints`, `CONFIG_TRAJ_WAYPOINTS=y`). A consequence "
        "is that in waypoint mode the flight speed is set by PX4's `MPC_XY_VEL_MAX` and not by "
        "`cruise_speed_mps`. Changing `MPC_XY_VEL_MAX` was therefore how the speed was raised for "
        "the truncated runs.")
    R.p(
        "`mavlink_task.c` sends the resulting `SET_POSITION_TARGET_LOCAL_NED` to PX4 at 20 Hz, with "
        "the heading fixed at its take-off value and the altitude held at "
        "`CONFIG_CRUISE_ALT_CM` = 80 cm.")

    # 3.6
    R.h2("3.6  Camera streaming with the pose at capture")
    R.p(
        "A tag's position on the floor is the drone's position at the moment the image was taken "
        "plus the tag's offset in the image. A Wi-Fi delay of 100–200 ms between capture and "
        "reception would, at 0.5 m/s, shift every tag by 5–10 cm along the direction of flight. To "
        "remove this error source, the firmware pairs each frame with PX4's pose on the ESP32's own "
        "clock. `pose_hist.c` keeps the last 128 `LOCAL_POSITION_NED` samples (≈2.5 s at 50 Hz) "
        "with their arrival times. `camera_stream.c` looks up the sample nearest to the frame's "
        "capture time and inserts it into the JPEG as a 28-byte comment (COM) segment directly "
        "after the start-of-image marker. JPEG decoders ignore COM segments, so the course's viewers "
        "still display the frames. On the laptop, `frame_pose.py` parses the segment (magic "
        "`CDEP`, version, validity flag, gap to the PX4 sample in ms, x, y, heading and yaw rate). "
        "Frames whose pose is more than 100 ms from a PX4 sample are rejected.")

    # 3.7
    R.h2("3.7  AprilTag detection and geolocation")
    R.p(
        "`tag_pipeline.py` runs on a background thread. It keeps the camera stream alive, "
        "reassembles the UDP chunks into JPEGs, decodes them to grey-scale with OpenCV and runs "
        "`apriltag_host.Detector`. This is the course's port of the AprilTag library (esp-apriltag), "
        "compiled for the laptop from the firmware's own C sources, so detections, decision margins and the quality "
        "gate are identical to the on-board detector. Each detection yields the tag's translation "
        "t_c in the camera frame from the homography and the known tag size. The mission then "
        "applies its own camera mount (pitch 90°, lens 8 cm forward of the centre) and the frame "
        "pose:")
    R.equation("p_body = R_bc(θ_pitch) · t_c + r_lens", 4)
    R.equation("p_NED = [x, y]_drone + R_z(ψ_drone) · [p_body,x , p_body,y]", 5)
    R.p(
        f"where R_bc maps the camera axes (x right, y down, z out of the lens) into the body axes and "
        f"ψ is the heading at capture. A detection becomes a *sighting* only if it passes every "
        f"filter in {T('filters')}.")
    R.table("filters", "Detection filters applied before a sighting is accepted.",
            ["Filter", "Threshold", "Purpose"],
            [["Decision margin", "≥ 55", "Reject weak decodes (tag16h5 has only 30 codes)"],
             ["Hamming distance", "≤ 1 bit", "Reject heavily corrected decodes"],
             ["Pose reprojection error", "≤ 0.5", "Reject poorly fitted quadrilaterals"],
             ["Range from camera", "≤ 1.5 m", "Reject distant, low-resolution tags"],
             ["Height of tag relative to drone", "−0.5 m to 0.95 m below", "Reject implausible "
              "geometry"],
             ["Yaw rate at capture", "≤ 30 °/s", "Reject frames blurred by rotation"],
             ["Frame pose available", "PX4 sample within 100 ms", "Reject frames with no pose"]],
            widths_cm=[4.4, 4.2, 7.4])
    R.image_placeholder("projection", "Diagram of the projection chain: image → camera frame → "
                        "body frame → PX4 NED, showing the lens offset and the heading.",
                        "Geolocation of a detected tag.")

    # 3.8
    R.h2("3.8  Tag clustering and confirmation")
    R.p(
        "`tag_map.py` turns the stream of noisy sightings into one estimate per tag ID. A simple "
        "mean is fragile because a single wrong-ID decode or a sighting taken during a jerk can "
        "pull it far away, so the estimate is built robustly at every new sighting:")
    R.bullets([
        "**Seed:** the sighting with the most neighbours within `confirm_radius_m` = 0.20 m (the "
        "densest point).",
        "**Inliers:** all sightings within `outlier_radius_m` = 0.30 m of the seed.",
        "**Estimate:** the per-axis median of the inliers, refined once by re-selecting inliers "
        "around that median.",
        "**Confirmation:** the tag is confirmed the first time at least `min_sightings` = 5 "
        "sightings lie within 0.20 m of the estimate. The estimate keeps improving afterwards.",
    ], numbered=True)
    R.p(
        "An isolated false sighting can never gather five supporting neighbours, so it is never "
        "confirmed. Localisation tag IDs (22–29) are routed to `localisation.py` instead of the tag map.")

    # 3.9
    R.h2("3.9  Localisation tags: alignment, precision landing and drift correction")
    R.p(
        "Optical-flow odometry drifts. Scale errors arise from rangefinder error (at 0.8 m, 1 cm of "
        "height error is a 1.25 % velocity error), heading drifts with the gyroscope, and position "
        "creeps slowly. To give the drifting PX4 frame something fixed to compare against, five "
        "additional tag16h5 tags were placed at surveyed grid nodes (" + T("loc_params") + "). "
        "They use the highest IDs of the family, so they can never be confused with the search "
        "tags. The course-supplied detector table, which stopped at ID 21, was extended to the full "
        "family of 30 codes. The localisation tags are used in three ways. None of them feeds "
        "a position back into PX4.")
    R.table("loc_params", "Localisation tags used in the demonstration (arena frame).",
            ["ID", "Grid node (a, b)", "x (m)", "y (m)", "Role"],
            [["29", "(0, 0)", "0.00", "0.00", "Home tag: alignment, landing spot, drift fix at start and end"],
             ["25", "(−2, 5)", "2.30", "−0.92", "Drift fix"],
             ["23", "(1, 5)", "2.30", "0.46", "Drift fix"],
             ["22", "(−3, 9)", "4.14", "−1.38", "Drift fix"],
             ["24", "(0, 9)", "4.14", "0.00", "Drift fix"]],
            widths_cm=[1.4, 3.0, 1.8, 1.8, 8.0], align=["c", "c", "r", "r", "l"])
    R.h3("Alignment before the sweep")
    R.p(
        "Without a compass, a drone placed 3° off square would sweep about 0.25 m off line at the "
        "far end of a 4.6 m lane. While hovering over the start box, the camera measures the "
        "orientation of the home tag, which is taped square to the floor tiles. The quarter-turn "
        "ambiguity of a square tag is resolved by taking the angle modulo 90°. The plan is then "
        "rotated by that angle and translated onto the tag's observed position before upload. "
        "Rotations larger than 15° are treated as misreadings and ignored.")
    R.h3("Precision landing")
    R.p(
        "The position of the home tag seen during take-off defines the landing target. After the "
        "return trajectory, the drone hovers and re-observes the tag. If the tag is now more than "
        "`land_radius_m` = 8 cm from where the drone thinks it is, because the odometry drifted "
        "during the flight, a short correction trajectory moves the drone over it. This is repeated "
        "at most three times and within at most 10 s before landing.")
    R.h3("Drift correction of the report")
    R.p(
        "Each time a localisation tag is in view, the visit (at least five sightings, separated "
        "from other visits by more than 5 s) gives a *fix*: the tag's robust position in the PX4 "
        "frame paired with its surveyed arena position. After the return, a 2-D similarity transform "
        "is fitted to all fixes by least squares (the closed-form solution of Umeyama [3]):")
    R.equation("p_arena = s · R(θ) · p_PX4 + t", 6)
    R.p(
        "Here s absorbs optical-flow scale error, θ the residual heading error and t the offset. "
        "The fit falls back to a rigid (s = 1) or translation-only model if the fixes span less "
        "than 1 m or if a check fails (RMS > 0.15 m, |θ| > 20°, |s − 1| > 0.25). One global "
        "transform cannot describe drift that changes during the flight. The software therefore "
        "also builds a *time* model, which interpolates the drift measured at each visit to the "
        "time each search tag was seen, and a *hybrid* model, which applies the global transform "
        "and then interpolates the remaining drift in time. In `auto` mode the model with the "
        "lowest leave-one-out prediction error over the visits is chosen. All three demonstration "
        "flights selected the hybrid model.")

    # 3.10
    R.h2("3.10  Logging, live map and post-flight tools")
    R.p(
        "`mission_log.py` creates one folder per run containing `events.csv` (every event and "
        "transition), `trajectory.csv` (the reconstructed route from 10 Hz telemetry), "
        "`detections.csv` (every accepted detection with its camera pose, body offset, drone pose "
        "and NED position), `tags_final.csv` (raw PX4 frame), `tags_corrected.csv` and "
        "`localisation.csv` (arena frame), `trajectory_corrected.csv`, a copy of the configuration "
        "used (`config_used.yaml`), the live-map snapshot `map.png` and the before/after "
        f"correction plot `correction.png`. The live map ({F('livemap')}) shows the area, plan, "
        "trail, candidate and confirmed tags and the camera view with each detection outlined and "
        "labelled as accepted or rejected. Post-flight tools include `tools/relocalise.py` "
        "(recompute the correction from a log, e.g. after re-measuring the localisation tags), "
        "`tools/compare_flights.py` (path, speed, corner and return statistics across runs), "
        "`tools/loc_layout.py` (click-to-place localisation tags on the grid) and "
        "`tools/cam_bench.py` (camera intrinsic and mount calibration on the bench).")
    R.figure("livemap", ROOT / "data" / "flights" / "truncated-exploration-2" / "map.png",
             "Live map at the end of Run 3, as saved by the mission (`map.png`). Left: search "
             "area, planned path, drone trail, confirmed tags (red) and localisation tags "
             "(violet). Right: the last camera frame with a rejected decode outlined. The '3/6' "
             "counter reflects the configured `expected_count`, not a verified tag total.",
             width_cm=16.0)

    # 3.11
    R.h2("3.11  Configuration management")
    R.p(
        "Every mission number lives in `mission/config.yaml`. `settings.py` loads it into typed, "
        "immutable dataclasses, checks ranges and cross-field consistency, and rejects unknown keys, "
        "so that a typing error fails before take-off rather than in the air. A copy of the file "
        "actually used is stored with each run. Firmware parameters (trajectory mode, corner "
        "radius, cruise altitude, network addresses) are set in ESP-IDF `menuconfig` and require "
        "re-flashing. PX4 parameters are set from QGroundControl through the ESP32's MAVLink "
        f"bridge. The values used in the demonstration are listed in Appendix B ({T('config')}).")

    # 3.12
    R.h2("3.12  Verification: simulator and automated tests")
    R.p(
        "`sim.py` replaces the drone and the camera with a simulated vehicle that follows the same "
        "UDP protocol and state sequence. It has configurable command loss, latency, pose noise and "
        "false-positive rate, and it can simulate odometry drift (scale, rotation and creep) to "
        "exercise the localisation correction. A synthetic camera (`synth_camera.py`) renders "
        "real tag16h5 images, so the actual detector is exercised. The full mission runs "
        "end-to-end with `python mission/mission.py --sim --auto --map`.")
    R.p(
        "The repository contains an automated test suite of 24 test modules. The pure-C firmware "
        "modules (`traj_gate`, `pose_hist`, the UWB parser and the UWB-to-PX4 converter) are "
        "compiled on the laptop with a host C compiler and driven from Python, which checks the wire formats and "
        "algorithms on both sides. On the configuration used for this report the suite "
        "completed with **432 tests passed and 1 skipped** "
        "(`python -m pytest mission/tests -q`, run on 12 November 2026).")


def sec_results(R: Report):
    R.h1("4  Mission Demonstration: Flight Results and Analysis")
    R.p(
        "This section analyses the three demonstration flights from the laptop-side mission logs. "
        "All positions below are PX4's own estimates relayed at about 10 Hz, unless stated as "
        "corrected. They are not ground truth: the only external reference available is the "
        "set of localisation tags. The figures and numbers are generated by "
        "`report/analyse_flights.py` from copies of the logs in `report/data/flights/`.")

    R.h2("4.1  Test configuration and procedure")
    R.p(
        "The first flight (Run 1) was the full search of the allocated area. The two subsequent "
        "flights (Runs 2 and 3) reduced the sweep to the two outer lanes and raised PX4's "
        "horizontal speed limit in order to find all tags faster. Everything else was "
        f"unchanged between the runs ({T('runs')}). For each run the drone was placed square "
        "on the home tag, powered up and left still. The operator typed `takeoff`, waited "
        "for the hover message and typed `start`. No further input was given.")
    rows = []
    for r in (R1, R2, R3):
        rows.append([r["name"].split(":")[0], f"{r['lane_spacing_m']:.1f}", f"{r['n_lanes']}",
                     f"{r['n_waypoints']}", f"{r['plan_path_m']:.2f}",
                     f"{r['mpc_xy_vel_max']:.1f}" + ("" if r["cap_confirmed"] else " [TO CONFIRM: from .ulg]"),
                     f"{r['align_deg']:+.2f}"])
    R.table("runs", "Configuration of the three demonstration flights (11 November 2026).",
            ["Run", "Max lane spacing (m)", "Lanes", "Way-points", "Planned sweep length (m)",
             "MPC_XY_VEL_MAX (m/s)", "Plan rotation at alignment (°)"],
            rows, widths_cm=[1.6, 2.2, 1.5, 1.8, 2.6, 3.6, 2.7],
            align=["l", "c", "c", "c", "c", "c", "c"])
    R.p(
        "Common to all runs: flight height 0.8 m; waypoint-mode trajectory gate with 15 cm corner "
        "radius; camera at the firmware default (≈7–10 frames/s, JPEG quality 60, 320 × 240); "
        "`min_sightings` = 5; drift model `auto`; UWB not used (`uwb.send_to_px4: false`).")

    R.h2("4.2  Mission timeline")
    R.figure("timeline", FIG / "fig_timeline.png",
             "Phase durations of the three flights, from the take-off command to the end of the "
             "landing settle time.")
    rows = []
    for r in (R1, R2, R3):
        rows.append([r["name"].split(":")[0], f"{r['t_explore_s']:.1f}", f"{r['sweep_duration_s']:.1f}",
                     f"{r['all_tags_after_sweep_start_s']:.1f}", f"{r['return_duration_s']:.1f}",
                     f"{r['settle_duration_s']:.1f}", f"{r['t_done_s']:.1f}"])
    R.table("timeline", "Mission timing (s). Times are from the take-off command unless stated.",
            ["Run", "Sweep starts", "Sweep duration", "All tags confirmed (from sweep start)",
             "Return flight", "Precision-landing hold", "Mission complete"],
            rows, widths_cm=[1.6, 2.0, 2.2, 3.4, 2.2, 2.6, 2.4], align=["l"] + ["c"] * 6)
    R.p(
        f"Take-off to a confirmed hover took 10–16 s in every run. In Run 2 the operator typed "
        f"`start` at 10.1 s, but the hover was only confirmed at {R2['t_hover_confirmed_s']:.1f} s, "
        f"because the position had to stay within 10 cm for 2 s. The supervisor correctly held the "
        f"sweep until then. The sweep itself shrank from {R1['sweep_duration_s']:.1f} s to "
        f"{R2['sweep_duration_s']:.1f} s and {R3['sweep_duration_s']:.1f} s. With fixed take-off, "
        f"return and landing overheads of 30–34 s, the total mission time fell from "
        f"{R1['t_done_s']:.1f} s to {R3['t_done_s']:.1f} s (−48 %). In Runs 1 and 3 the "
        f"precision-landing hold ran to its 10 s limit (Section 4.8).")

    R.h2("4.3  Flight paths")
    R.p(
        f"{F('path1')}–{F('path3')} show each flight in the PX4 frame as flown, next to the same "
        "route after drift correction in the arena frame. The planned path is drawn after the "
        "alignment rotation, from the hover point through the lane ends. In every run the drone "
        "followed the planned sequence of lanes, passed all waypoints, and returned over the "
        "home tag. The trajectory never triggered the geofence or any other safety layer.")
    for i, r in enumerate((R1, R2, R3), 1):
        R.figure(f"path{i}", FIG / f"fig_path_run{i}.png",
                 f"{r['name']} (MPC_XY_VEL_MAX = {r['mpc_xy_vel_max']:.1f} m/s"
                 + ("" if r["cap_confirmed"] else ", to be confirmed") + "). Left: PX4 frame "
                 "with the planned path, raw tag estimates and raw localisation-tag visits. "
                 "Right: drift-corrected route and tags with the surveyed localisation tags.",
                 width_cm=15.5)

    R.h2("4.4  Speed and path-tracking performance")
    R.figure("speed", FIG / "fig_speed.png",
             "Ground speed (from 10 Hz position telemetry, ±0.2 s central difference) during "
             "the sweep and the return, with the PX4 horizontal speed limit.")
    R.p(
        f"The speed traces in {F('speed')} confirm that in waypoint mode the speed limit, and not "
        f"the planned trajectory speed, sets the cruise speed. The median lane speed was "
        f"{R1['speed_cruise_median_mps']:.2f}, {R2['speed_cruise_median_mps']:.2f} and "
        f"{R3['speed_cruise_median_mps']:.2f} m/s, i.e. 74 %, 70 % and 83 % of the limit. The drone "
        f"never exceeded the limit (maxima {R1['speed_max_mps']:.2f}, {R2['speed_max_mps']:.2f} and "
        f"{R3['speed_max_mps']:.2f} m/s). At 1.5 m/s the 4.6 m lanes became too short to hold a "
        f"cruise: the trace in Run 3 is a sequence of acceleration-deceleration peaks, one per leg.")
    rows = []
    for r in (R1, R2, R3):
        for l in r["lanes"]:
            rows.append([r["name"].split(":")[0], f"{l['lane']}", f"{l['duration_s']:.1f}",
                         f"{l['xt_mean_m'] * 100:+.1f}", f"{l['xt_rms_m'] * 100:.1f}",
                         f"{l['xt_max_m'] * 100:.1f}", f"{l['overshoot_m'] * 100:.1f}"])
    R.table("tracking", "Cross-track error on each lane (excluding 0.3 m at each end) and overshoot "
            "past the lane end. Positive mean = right of the direction of travel.",
            ["Run", "Lane", "Lane time (s)", "Mean (cm)", "RMS (cm)", "Max |e| (cm)",
             "Overshoot (cm)"],
            rows, widths_cm=[1.8, 1.4, 2.4, 2.4, 2.4, 2.6, 2.6], align=["l"] + ["c"] * 6)
    R.p(
        f"Path tracking degrades steadily with speed ({T('tracking')}). The overall RMS cross-track "
        f"error grew from {cm(R1['xt_rms_all_m'])} to {cm(R2['xt_rms_all_m'])} and "
        f"{cm(R3['xt_rms_all_m'])}, and the worst lane-end overshoot from "
        f"{cm(R1['overshoot_max_m'])} to {cm(R3['overshoot_max_m'])}. In the fast runs the error is "
        f"mostly a *bias*, not noise: the mean error is close to the RMS, and it always lies on the "
        f"outside of the sweep. The mechanism follows from Eq. (3). The drone arrives at each lane "
        f"start carrying momentum from the transit leg. Once it is within 15 cm of the corner, the "
        f"setpoint jumps to the far end of the lane. The resulting error vector is almost entirely "
        f"along the lane, so PX4's norm-limited velocity demand leaves only a few cm/s to remove the "
        f"sideways offset. The offset therefore persists for most of the lane. Since the outer lanes "
        f"lie only 0.3 m inside the area boundary, a 27 cm outward bias at 1.5 m/s brings the "
        f"drone close to the edge of the area. The heading stayed within "
        f"{min(r['sweep_heading_min_deg'] for r in (R1, R2, R3)):+.1f}° to "
        f"{max(r['sweep_heading_max_deg'] for r in (R1, R2, R3)):+.1f}° during all sweeps, "
        f"so yaw control did not contribute to the error.")

    R.h2("4.5  Tag detection performance")
    rows = []
    for r in (R1, R2, R3):
        for t in r["tags"]:
            rows.append([r["name"].split(":")[0], f"{t['tag_id']}", f"{t['n_sightings']}",
                         f"{t['seen_for_s']:.1f}", f"{t['speed_while_seen_mps']:.2f}",
                         f"{t['confirm_delay_s']:.2f}", f"{t['decision_margin_median']:.0f}",
                         f"{t['range_median_m']:.2f}"])
    R.table("tags", "Detection statistics for each search tag.",
            ["Run", "Tag", "Sightings", "In view (s)", "Mean speed in view (m/s)",
             "First seen → confirmed (s)", "Median decision margin", "Median range (m)"],
            rows, widths_cm=[1.6, 1.2, 1.9, 1.7, 2.3, 2.6, 2.4, 2.1], align=["l"] + ["c"] * 7)
    R.p(
        "The same three search tags (IDs 2, 4 and 6) were confirmed in every run, and no false tag "
        "was confirmed. Each tag was confirmed 0.3–0.5 s after it was first seen, which corresponds "
        "to five consecutive frames at the camera's 7–10 frames/s. The measured range "
        "of 0.70–0.86 m agrees with the 0.8 m flight height, which is an independent check of the "
        "camera model and the tag size.")
    R.figure("sightings", FIG / "fig_sightings.png",
             "Accepted sightings per search tag in each run. The dashed line is the confirmation "
             "threshold of five sightings.")
    R.p(
        f"The number of sightings per tag fell roughly in proportion to speed ({F('sightings')}): "
        f"from 24–41 in Run 1 to 6–13 in Run 3. A tag stays in the 1.25 m long footprint for "
        f"about 1.25 / v seconds, giving roughly f · 1.25 / v frames at a frame rate f. At "
        f"1.3 m/s and 7–10 frames/s this is only 7–10 frames, of which tag 4 yielded six accepted sightings, one above the "
        f"confirmation threshold. The median decision margin also fell from about 100–108 to "
        f"82–97 in Run 3, consistent with motion blur at higher speed. Run 3 was therefore "
        f"close to the detection limit of the current camera configuration. A further speed increase "
        f"would have risked unconfirmed tags.")
    R.p(
        f"The localisation tags show the same trend more sharply. In the truncated sweeps tag 24 "
        f"(between the two lanes at the far end) was visible only from the transit leg, and "
        f"tag 23 lay 0.62 m from the nearer lane, at the edge of the footprint. They gave "
        f"{R2['detections_by_id'].get('24', 0)} and {R2['detections_by_id'].get('23', 0)} "
        f"sightings in Run 2, and none in Run 3. Tag 25 gave {R3['detections_by_id'].get('25', 0)} "
        f"sightings in Run 3, below the five needed for a fix. This directly weakened the drift "
        f"correction of Run 3 (Section 4.7).")

    R.h2("4.6  Tag position estimates and run-to-run consistency")
    rows = []
    for tid in ("2", "4", "6"):
        for r in (R1, R2, R3):
            t = next(t for t in r["tags"] if str(t["tag_id"]) == tid)
            rows.append([tid, r["name"].split(":")[0], f"({t['raw_x']:.3f}, {t['raw_y']:.3f})",
                         f"({t['corr_x']:.3f}, {t['corr_y']:.3f})", f"{t['shift_m'] * 100:.1f}"])
    R.table("positions", "Reported tag positions (x, y) in metres: raw PX4 frame and drift-corrected "
            "arena frame.",
            ["Tag", "Run", "Raw PX4 frame", "Corrected (arena)", "Correction (cm)"],
            rows, widths_cm=[1.4, 1.8, 4.6, 4.6, 3.0], align=["c", "l", "c", "c", "c"])
    R.figure("repeat", FIG / "fig_repeatability.png",
             "Run-to-run scatter of each tag estimate. Hollow markers: raw PX4 frame; filled: "
             "drift-corrected arena frame; thin lines join the two estimates of the same run.")
    d = DISP
    R.p(
        "The surveyed positions of the search tags on demonstration day were not available when this "
        "draft was written, so absolute accuracy cannot yet be stated. The reference values "
        "stored in `config.yaml` (`truth_tags`) belong to an earlier test layout and are deliberately "
        "*not* used. Repeatability across the three flights is nevertheless a strong indicator, "
        "because the tags did not move between runs while the drift differed in every run. "
        f"Without correction, the three estimates of each tag scattered by "
        f"{cm(d['2']['raw']['rms_from_mean_m'])}, {cm(d['4']['raw']['rms_from_mean_m'])} and "
        f"{cm(d['6']['raw']['rms_from_mean_m'])} RMS about their mean (tags 2, 4 and 6), with "
        f"worst pairwise differences up to {cm(d['6']['raw']['max_pairwise_m'])}. After correction "
        f"the scatter fell to {cm(d['2']['corr']['rms_from_mean_m'])}, "
        f"{cm(d['4']['corr']['rms_from_mean_m'])} and {cm(d['6']['corr']['rms_from_mean_m'])}. "
        f"The correction brought the estimates of tags 2 and 4 into agreement within "
        f"{cm(d['2']['corr']['max_pairwise_m'])} and {cm(d['4']['corr']['max_pairwise_m'])}. "
        f"Tag 6, in the far corner diagonally opposite the area's start, retained the largest "
        f"scatter, and the corrected estimate of Run 3 is the outlier ({F('repeat')}).")
    R.todo("Insert the surveyed positions of tags 2, 4 and 6 for the demonstration layout, then "
           "re-run `python mission/tools/relocalise.py <run> --truth \"2:x,y 4:x,y 6:x,y\"` for "
           "each run and add a table of absolute errors (raw and corrected) here. "
           "If more than three search tags were placed in the area, state which were missed and why.")

    R.h2("4.7  Odometry drift and localisation-tag correction")
    rows = []
    for r in (R1, R2, R3):
        f = r["loc_fit"]
        rows.append([r["name"].split(":")[0].replace("Run ", ""), f"{f['n_fixes']}",
                     ", ".join(str(i) for i in r["loc_ids_fixed"]),
                     f"{r['loc_raw_err_mean_m'] * 100:.1f} / {r['loc_raw_err_max_m'] * 100:.1f}",
                     f"{f['turn_deg']:+.2f}", f"{f['scale']:.3f}",
                     f"({f['shift_x']:+.2f}, {f['shift_y']:+.2f})", f"{f['rms_m'] * 100:.1f}",
                     f"{f['loo_global_cm']:.1f} / {f['loo_time_cm']:.1f} / {f['loo_hybrid_cm']:.1f}"])
    R.table("locfit", "Drift at the localisation tags and the fitted correction.",
            ["Run", "Fixes", "Tags fixed", "Raw error mean / max (cm)", "Turn (°)", "Scale",
             "Shift (m)", "Fit RMS (cm)", "Leave-one-out global / time / hybrid (cm)"],
            rows, widths_cm=[1.1, 1.4, 2.5, 2.2, 1.4, 1.4, 2.4, 1.5, 2.4], align=["c"] * 9)
    R.figure("loc", FIG / "fig_loc.png",
             "Position error at each localisation-tag visit (L<ID>.<visit>) before correction "
             "(raw PX4 estimate against the surveyed position) and after the global similarity fit.")
    R.p(
        f"Uncorrected, PX4's position at the localisation tags was in error by "
        f"{cm(R1['loc_raw_err_mean_m'])} on average in Run 1 and {cm(R2['loc_raw_err_mean_m'])} in "
        f"Run 2, with maxima of {cm(R1['loc_raw_err_max_m'])} and {cm(R2['loc_raw_err_max_m'])} "
        f"({T('locfit')}). This is consistent with the run-to-run scatter of the raw tag estimates. "
        f"Runs 1 and 2 independently estimated almost the same scale factor (0.978), i.e. "
        f"PX4 over-estimated distances by about 2.2 %. The fitted frame rotation of −2.4° to −3.8° "
        f"has the sign and size expected from the placement error measured at the take-off "
        f"alignment (+2.65° and +4.95°). The repeated scale suggests a systematic "
        f"optical-flow or rangefinder scale error that could be calibrated out on the flow sensor "
        f"itself. In Runs 1 and 2 the hybrid model predicted held-out visits to "
        f"{R1['loc_fit']['loo_hybrid_cm']:.1f} cm and {R2['loc_fit']['loo_hybrid_cm']:.1f} cm.")
    R.p(
        f"Run 3 shows the limits of the method. With only three fixes (tag 29 twice and tag 22 once) "
        f"for a four-parameter model, the fit is nearly exact (RMS {R3['loc_fit']['rms_m'] * 100:.1f} cm, "
        f"residual at tag 22 below 1 mm) but poorly determined. The fitted scale of "
        f"{R3['loc_fit']['scale']:.3f} contradicts the 0.978 of the other runs, and the leave-one-out "
        f"error rose to {R3['loc_fit']['loo_hybrid_cm']:.1f} cm. The larger correction applied to "
        f"tag 6 in Run 3 should therefore be treated with caution. A sweep that is fast and sparse "
        f"collects fewer fixes, and drift correction depends on those fixes.")

    R.h2("4.8  Return to home and precision landing")
    rows = []
    for r in (R1, R2, R3):
        rows.append([r["name"].split(":")[0], f"{r['return_duration_s']:.1f}",
                     f"{r['land_first_offset_cm']:.0f}", f"{r['land_targets']}",
                     f"{r['land_correction_moves']}", f"{r['settle_duration_s']:.1f}",
                     f"{r['land_final_offset_cm']:.0f}"])
    R.table("landing", "Return and precision landing on the home tag.",
            ["Run", "Return flight (s)", "Home-tag offset on arrival (cm)", "Landing targets set",
             "Correction moves", "Hold before landing (s)", "Offset at land command (cm)"],
            rows, widths_cm=[1.6, 2.2, 2.8, 2.2, 2.2, 2.4, 2.6], align=["l"] + ["c"] * 6)
    R.p(
        f"On arrival over the take-off point the home tag appeared 9, 23 and 17 cm away from "
        f"where PX4 placed it, a direct measurement of the drift accumulated during each flight "
        f"({T('landing')}). Precision landing reduced the offset to 2–5 cm before the land command "
        f"in every run, so the drone landed on the start box each time. In Runs 1 and 3 the drone "
        f"kept making 2–3 cm corrections until the 10 s limit, because the 8 cm acceptance radius "
        f"is comparable to the hover wobble. This cost about 5 s per flight without improving the "
        f"result. Raising `land_radius_m` to 0.10 m, or accepting the position after one correction "
        f"within 2 × the radius, would remove most of this hold.")

    R.h2("4.9  Discussion")
    R.h3("Speed versus coverage and accuracy")
    R.p(
        "The truncated runs met their aim of finding the known tags faster: all three tags were "
        f"confirmed {R2['all_tags_after_sweep_start_s']:.1f} s and {R3['all_tags_after_sweep_start_s']:.1f} s "
        f"after the sweep started, against {R1['all_tags_after_sweep_start_s']:.1f} s for the full "
        "search. They came, however, at three costs. First, coverage: the two-lane sweep does not "
        "image the central strip of the area (Section 3.3). It found all tags only because the "
        "tags in this layout lay close to the outer lanes. Without prior knowledge of the tag "
        "positions, only Run 1 constitutes a complete search. Second, path tracking: the "
        "cross-track bias of up to 27 cm at 1.5 m/s reduces the margin to the area boundary and "
        "shifts the camera footprint. Third, evidence: fewer frames per tag and fewer "
        "localisation fixes leave both detection and drift correction closer to their limits.")
    R.p(
        "For a search with unknown tag positions, the better route to a shorter mission is to keep "
        "full coverage and recover time elsewhere. Lanes could be spaced at the full effective "
        "swath of about 1.4 m, giving three lanes instead of four. The speed could be raised "
        "moderately to about 0.7 m/s, where Run 2 shows that detection and tracking are still "
        "comfortable. The precision-landing hold could be shortened. Three lanes 1.31 m apart leave "
        "only about 0.14 m of overlap, which must be checked against the cross-track error at "
        "that speed. Together these changes would cut the Run 1 mission time by an estimated "
        "one third while retaining complete coverage. "
        "[TO CONFIRM: validate by a flight with lane_spacing 1.4 m and MPC_XY_VEL_MAX 0.7 m/s.]")
    R.h3("Limitations of this analysis")
    R.bullets([
        "Positions are PX4 estimates, not ground truth. Path-tracking errors are relative to the "
        "estimated, not the true, position.",
        "Absolute tag accuracy awaits the surveyed demonstration layout (Section 4.6).",
        "The 1.0 m/s speed limit of Run 2 is inferred from the flight data and must be confirmed "
        "from the onboard parameters (Section 5).",
        "Speeds are differentiated from 10 Hz Wi-Fi telemetry with occasional gaps of up to "
        f"{max(r['telemetry_dt_max_s'] for r in (R1, R2, R3)) * 1000:.0f} ms. The higher-rate "
        "PX4 log will refine them.",
        "Three flights are too few for statistical conclusions; the trends are, however, "
        "monotonic and physically explained.",
    ])


def sec_ulog(R: Report):
    R.h1("5  Black-Box Flight Log Analysis (PX4 ULog)")
    R.todo(
        "This section is a framework awaiting the PX4 ULog files of the three demonstration flights "
        "(11 November 2026, approximately 11:13, 11:22 and 11:29 local time). Download them from the "
        "flight controller's SD card (QGroundControl → Analyze Tools → Log Download), place them in "
        "`report/data/ulog/`, and follow `report/AGENT_NOTES.md`, Section 5, to fill in each subsection.")
    R.h2("5.1  Purpose and data source")
    R.p(
        "The laptop logs record what the drone *reported* over Wi-Fi at about 10 Hz. The PX4 "
        "ULog records the flight controller's internal state at rates of up to several hundred "
        "hertz: setpoints, estimates, sensor data, controller outputs, estimator health and "
        "power. It is therefore the authoritative source for verifying the controller "
        "behaviour inferred in Section 4 and for examining aspects that are invisible from the "
        "laptop: attitude control, the optical-flow fusion, motor saturation and battery sag. "
        "The flights are to be identified in the ULogs by their OFFBOARD segments (navigation "
        "state 14) and aligned with the mission logs using the take-off command time.")
    R.h2("5.2  Findings from a pre-demonstration log")
    R.p(
        "A ULog from an earlier test flight (≈65 s, offboard sweep at 0.4 m/s with step position "
        "setpoints) was analysed before the demonstration (`cde1302/px4-flight-log-findings.md`). "
        "Its findings shaped the final configuration:")
    R.bullets([
        "**Cross-track correction is limited by the speed cap** when position setpoints are "
        "distant steps (Eq. 3): 0.35–0.4 m off-axis errors persisted along every leg. This motivated "
        "the follow mode (Section 3.5) and explains the bias measured in Section 4.4.",
        "**No heading reference:** with `EKF2_MAG_TYPE` = None the yaw is gyroscope-integrated; "
        "the EKF's own yaw standard deviation grew from about 3.7° to 5.1° in 60 s. This "
        "motivated the take-off alignment to the home tag and short flight times.",
        "**Rate and attitude loops were not the bottleneck** (attitude standard deviation ≈1°); "
        "the vehicle is strongly over-powered (hover throttle ≈0.155).",
        "**Optical-flow fusion was healthy** (innovation ≈0.1 rad/s RMS, test ratios ≤0.14) at "
        "≈0.47 m height. Flying at 0.8 m in the demonstration reduced the relative effect "
        "of rangefinder error by about 40 %.",
        "**Parameter clean-up needed:** external-vision and GPS fusion were still enabled without a "
        "source, logging (`SDLOG_PROFILE` 161) was too sparse for estimator analysis, and an RC "
        "roll trim offset blocked auto-tuning.",
    ])
    R.p(
        "A further log from 10 November 2026 (`cde1302/log_185_UnknownDate.ulg`, 2 min 25 s, "
        "MicoAir H743 V2, PX4 v1.15.4) is also available. It records `MPC_XY_VEL_MAX` = 0.5 m/s, "
        "`MPC_XY_P` = 1.3, `MPC_ACC_HOR` = 3.0 m/s², `EKF2_EV_CTRL` = 0 and `SDLOG_PROFILE` = 161. "
        "It precedes the demonstration and is not one of the three flights analysed here.")
    R.h2("5.3  Parameter verification")
    R.table("ulog_params", "PX4 parameters to be read from each demonstration ULog "
            "([TBC] = to be completed from the log).",
            ["Parameter", "Run 1", "Run 2", "Run 3", "Relevance"],
            [["MPC_XY_VEL_MAX", "0.5", "1.0 [TBC]", "1.5", "Cruise speed in waypoint mode"],
             ["MPC_XY_CRUISE", "[TBC]", "[TBC]", "[TBC]", "Auto-mode cruise speed"],
             ["MPC_ACC_HOR, MPC_ACC_HOR_MAX", "[TBC]", "[TBC]", "[TBC]", "Lane-end overshoot"],
             ["MPC_XY_P, MPC_XY_VEL_P_ACC", "[TBC]", "[TBC]", "[TBC]", "Cross-track correction"],
             ["EKF2_OF_CTRL, EKF2_HGT_REF", "[TBC]", "[TBC]", "[TBC]", "Estimator sources"],
             ["EKF2_EV_CTRL", "[TBC]", "[TBC]", "[TBC]", "Confirms UWB not fused"],
             ["SDLOG_PROFILE", "[TBC]", "[TBC]", "[TBC]", "Logged topics and rates"]],
            widths_cm=[5.0, 1.8, 2.2, 1.8, 5.1], align=["l", "c", "c", "c", "l"])
    R.h2("5.4  Position and velocity tracking")
    R.todo("Plot `trajectory_setpoint` against `vehicle_local_position` (x, y, vx, vy) for each "
           "sweep. Report the cross-track and along-track errors per lane at the controller rate, "
           "and compare them with Table " + str(TABS.index("tracking") + 1) + ". Check whether "
           "the horizontal velocity setpoint saturates at MPC_XY_VEL_MAX during lanes, and how "
           "much of it is directed across the lane (verification of Eq. 3).")
    R.image_placeholder("ulog_track", "Setpoint versus estimate, x and y, Run 1 and Run 3 "
                        "(to be generated by report/analyse_ulog.py).",
                        "PX4 position tracking during the sweep.")
    R.h2("5.5  Attitude and rate control")
    R.todo("Compare `vehicle_attitude_setpoint` with `vehicle_attitude` (roll, pitch) and "
           "`vehicle_rates_setpoint` with `vehicle_angular_velocity`. Report the peak tilt in "
           "each run (expected to rise with speed and at lane ends), RMS attitude tracking "
           "error, and any actuator saturation (`actuator_motors`, `control_allocator_status`).")
    R.image_placeholder("ulog_att", "Roll and pitch, setpoint versus estimate, for the three runs.",
                        "Attitude tracking.")
    R.h2("5.6  Estimator (EKF2) health")
    R.todo("From `estimator_status`, `estimator_innovations`, `estimator_optical_flow_vel` and "
           "`distance_sensor`: optical-flow quality and innovation test ratios against speed, "
           "range-finder height and its noise, yaw variance growth over each flight, and any "
           "position resets (`vehicle_local_position.xy_reset_counter`). Relate the growth of "
           "the yaw variance to the frame rotation found by the drift fit (Table "
           + str(TABS.index("locfit") + 1) + ").")
    R.image_placeholder("ulog_ekf", "Optical-flow quality, innovation test ratio and height for "
                        "the three runs.", "EKF2 estimator health.")
    R.h2("5.7  Power, thrust margin and vibration")
    R.todo("From `battery_status` and `actuator_motors`: voltage sag and current at hover and at "
           "peak speed, mean motor output (thrust margin), and from `sensor_combined` or "
           "`vehicle_imu_status` the vibration level. State the energy used per flight.")
    R.h2("5.8  Summary of black-box findings")
    R.table("ulog_plan", "Questions the ULog analysis is to answer.",
            ["Question", "Topic(s)", "Answer"],
            [["Was MPC_XY_VEL_MAX = 1.0 m/s in Run 2?", "parameters", "[TO BE COMPLETED]"],
             ["Is the outward lane bias caused by norm-limited velocity demand?",
              "trajectory_setpoint, vehicle_local_position", "[TO BE COMPLETED]"],
             ["Did tilt or motor output approach limits at 1.5 m/s?",
              "vehicle_attitude, actuator_motors", "[TO BE COMPLETED]"],
             ["Did optical-flow quality drop at higher speed?", "estimator_optical_flow_vel",
              "[TO BE COMPLETED]"],
             ["Does the yaw variance explain the fitted frame rotation?", "estimator_states",
              "[TO BE COMPLETED]"]],
            widths_cm=[6.4, 5.2, 4.4])


def sec_extras(R: Report):
    R.h1("6  [EXTRAS] Additional Work Beyond the Mission Scope")
    R.p(
        "The work in this section was carried out by the group in addition to the mission "
        "requirements. The mission software does not depend on it, and the UWB system was "
        "not used in the demonstration flights.")

    R.h2("6.1  [EXTRAS] Redesigned 3D-printed frame")
    R.p(
        "The course frame was designed to be cut from carbon-fibre plate but is printed in PLA, "
        "which is roughly 20–30 times less stiff. The printed frame flexed visibly under load. "
        "The group therefore redesigned the frame for fused-deposition printing. The goals were "
        "stiffer arms, faster assembly and no print supports. The design is in `mechanical/` "
        "(STEP assembly and STL parts; source repository `github.com/eggsacc/CDE1302-drone-frame`).")
    R.p(
        "Bending stiffness scales with the second moment of area, which for a rectangular section "
        "of width b and depth t is")
    R.equation("I = b · t³ / 12", 7)
    R.p(
        "Stiffness therefore grows with the cube of the section depth. The new arms move material "
        "away from the bending axis into deep top and bottom lattices held apart by thin triangular "
        "trusses. This gives a much deeper section for about the same mass. In simulation under "
        f"the same 30 N·m bending load, the maximum deflection fell from 14.4 mm to 5.31 mm "
        f"(−63 %) ({T('frame')}).")
    R.table("frame", "Original and redesigned frames.",
            ["Property", "Original (course)", "Redesign"],
            [["Maximum deflection, 30 N·m bending (PLA, simulated)", "14.4 mm", "5.31 mm (−63 %)"],
             ["Assembly time", "≈1 h", "≈15 min"],
             ["Print supports", "Needed", "None"],
             ["Print time", "≈3.5 h", "≈3.5 h"],
             ["Printed frame mass", "≈50 g", "≈50 g"],
             ["All-up mass with battery", "—", "≈224 g"]],
            widths_cm=[7.6, 4.0, 4.4])
    R.p(
        "Further features: splayed, replaceable landing feet that absorb landing impact (they "
        "replace nylon spacers that often came loose); press-fit nut retainers so stand-offs fit "
        "without tools; a battery-strap pass-through under the flight controller; modelled wire "
        "channels that keep leads away from the propellers; and a cover for the MTF-02P sensor. "
        "Every part respects the printing limits of PLA, which are bridges up to about 20 mm and "
        "overhangs below about 55°. The propeller guards bridge at most 16 mm and use 45° "
        "chamfers in place of flat overhangs. Small supports that cannot be avoided are modelled "
        "into the main frame as separate bodies that are imported together. All parts fit on one "
        "256 × 256 mm build plate.")
    R.figure("frame", ROOT.parent / "mechanical" / "assets" / "frame_render.png",
             "Rendering of the redesigned frame.", width_cm=12.0)
    R.image_placeholder("frame_sim", "Bending simulation of the original and the redesigned frame "
                        "(deflection colour maps), and the arm cross-sections.",
                        "Structural comparison of the two frames.")

    R.h2("6.2  [EXTRAS] Electrical integration and networking")
    R.p(
        "The XIAO ESP32-S3 Sense is connected to a telemetry port of the flight controller: "
        "ESP32 GPIO5 (D4, TX) to the flight controller's RX and GPIO4 (D3, RX) to its TX, at "
        "921 600 baud. The MTF-02P is connected to the flight controller, which uses it with "
        "`EKF2_OF_CTRL` enabled and `EKF2_HGT_REF` = range sensor. The laptop and drone share a "
        "dedicated router with no internet access and static addresses derived from the drone ID "
        "(drone 192.168.1.(200 + ID), laptop 192.168.1.(100 + ID)). This avoids DHCP delays and "
        "keeps the 2.4 GHz channel free of other traffic. If the ESP32 loses the router for more "
        "than about 3 s, the firmware stops the motors. The VL53L5CX time-of-flight ring of the course "
        "template (obstacle avoidance) is not needed in an obstacle-free area and is compiled out.")
    R.image_placeholder("wiring", "Wiring diagram or photograph of the assembled electronics.",
                        "Electrical integration.")
    R.todo("Complete the component list (motors, ESCs, battery cells and capacity, receiver) and a "
           "power budget (hover current, expected endurance) from `electrical/README.md`.")

    R.h2("6.3  [EXTRAS] Ultra-wideband (UWB) positioning trial")
    R.p(
        "Because optical-flow odometry drifts, a team member evaluated a Nooploop LinkTrack UWB "
        "system as an absolute horizontal position reference (branches `uwb-testing` and "
        "`claude-one-shot`; documentation in `cde1302/mission/UWB_*.md` and `UWB_documentation/`). "
        "A LinkTrack tag on the drone (UART to ESP32 GPIO9 at 921 600 baud, NODE_FRAME2 protocol, "
        "50 Hz) ranged to four anchors placed around the arena.")
    R.h3("Method")
    R.p(
        "The integration is *loosely coupled*. The UWB module computes its own x–y position, "
        "which is fed to PX4 as an external-vision position measurement and fused by EKF2 with "
        "the IMU and optical flow. UWB and PX4 use different frames: the UWB frame has its origin "
        "at anchor A0, z up and axes set by the anchors, while the PX4 frame has its origin at "
        "power-up and z down. A 2-D rigid alignment with a mirror term is therefore estimated in "
        "flight:")
    R.equation("p_PX4 = R(θ) · M · p_UWB + t", 8)
    R.p(
        "where M flips the y-axis (z-up to z-down). While the drone flies the first part of the "
        "sweep (at least 0.8 m of path, 50 matched samples), the laptop pairs UWB fixes with PX4 "
        "positions streamed by the ESP32. It solves Eq. (8) by least squares with one outlier-"
        "rejection pass and sends the result to the ESP32 (`CMD_UWB_ALIGN`). The ESP32 then "
        "transforms every UWB fix itself and sends it to PX4 at 20 Hz with its reported precision, "
        f"so the laptop is no longer in the loop. The PX4 parameters used are listed in "
        f"{T('uwb_params')}.")
    R.table("uwb_params", "PX4 parameters for UWB fusion (trial only).",
            ["Parameter", "Value", "Purpose"],
            [["EKF2_EV_CTRL", "1", "Fuse horizontal position only"],
             ["EKF2_EV_NOISE_MD", "0", "Use the precision reported with each fix"],
             ["EKF2_EVP_NOISE", "0.15 m", "Lower bound on the assumed fix error"],
             ["EKF2_EVP_GATE", "3 SD", "Reject inconsistent fixes"],
             ["EKF2_EV_DELAY", "50 ms", "Measurement latency"],
             ["EKF2_EV_POS_X / Y / Z", "−0.04 / 0 / 0.03 m", "Antenna offset from the flight controller"]],
            widths_cm=[4.6, 4.0, 7.4])
    R.h3("Results and decision")
    R.p(
        "On the `uwb-testing` branch the alignment was first obtained from a dedicated 1 m "
        "calibration square (`simple_uwb_calibration.py`, fit accepted below 0.10 m RMS), and "
        "later solved during the sweep itself. The UWB feed to PX4 was flown and worked. Static "
        "fixes scattered by 5–7 cm. In some flights, however, the drone pulled sideways near the "
        "far end of the area. The suspected causes were the estimator restarting the UWB fusion "
        "and poorer anchor coverage there. A continuous-alignment "
        "fix was implemented but not flown. More fundamentally, in an arena of about "
        "5 m × 3 m the optical-flow drift over a one-minute flight is itself only of order "
        "10–25 cm (Section 4.7). That is comparable to the UWB noise and anchor-geometry errors, "
        "so the measurable benefit was insignificant. The localisation-tag correction achieved "
        "comparable consistency with no extra hardware and no risk of in-flight position jumps. UWB "
        "was therefore disabled for the demonstration (`uwb.send_to_px4: false`; the firmware "
        "feature remains compiled in but inactive without an alignment).")
    R.table("uwb_cmp", "UWB trial results.",
            ["Metric", "Optical flow only", "Optical flow + UWB"],
            [["Return-to-home error", "[TO BE COMPLETED]", "[TO BE COMPLETED]"],
             ["Drift at far end of area", "[TO BE COMPLETED]", "[TO BE COMPLETED]"],
             ["Lane cross-track RMS", "[TO BE COMPLETED]", "[TO BE COMPLETED]"],
             ["Position resets / jumps", "[TO BE COMPLETED]", "[TO BE COMPLETED]"]],
            widths_cm=[6.0, 5.0, 5.0])
    R.todo("Add the UWB trial results from the teammate's flights (e.g. `tools/compare_flights.py` "
           "output on the uwb-testing logs) and a photo of the anchor layout.")
    R.image_placeholder("uwb_setup", "UWB anchor layout around the arena and tag mounting on the "
                        "drone.", "UWB trial set-up.")


def sec_conclusion(R: Report):
    R.h1("7  Conclusions and Future Work")
    R.p(
        "The group built a pseudo-autonomous search system that takes off, sweeps an allocated area, "
        "detects and localises floor-mounted AprilTags, returns and lands on its start marker without "
        "operator steering. Three demonstration flights all completed the mission and located the "
        "same three search tags (IDs 2, 4 and 6), with no false confirmations and no safety intervention. The design "
        "choices that mattered most were:")
    R.bullets([
        "executing the whole sweep on board, so that Wi-Fi latency and drops do not affect the "
        "flight path;",
        "tagging each camera frame with the PX4 pose on the drone's own clock, so that "
        "transmission delay does not bias tag positions;",
        "robust median-based clustering with a five-sighting confirmation rule;",
        "surveyed localisation tags that align the sweep before take-off, guide the landing and "
        "correct the report for drift. The correction reduced the run-to-run scatter of the "
        f"reported tag positions from up to {cm(DISP['6']['raw']['rms_from_mean_m'])} RMS to "
        f"{cm(DISP['4']['corr']['rms_from_mean_m'])}–{cm(DISP['6']['corr']['rms_from_mean_m'])} RMS.",
    ])
    R.p(
        "Raising the speed limit to about 1.5 m/s on a two-lane sweep halved the mission time. It "
        "did so at the cost of full coverage, a cross-track bias of up to 27 cm, and detection "
        "and drift correction operating close to their limits. Recommended future work:")
    R.bullets([
        "Re-plan for speed with full coverage: about 1.4 m lane spacing at about 0.7 m/s, and a "
        "shorter precision-landing hold.",
        "Fix the lane-start bias: either a short velocity feed-forward segment after each corner, or "
        "an intermediate setpoint on the lane line before the far corner.",
        "Calibrate the optical-flow scale (both Runs 1 and 2 estimated 2.2 %) and the camera "
        "intrinsics on the bench (`tools/cam_bench.py`).",
        "Place localisation tags under the lanes so that every sweep, including sparse ones, "
        "collects enough fixes.",
        "Raise the camera frame rate (branch `camera-fps`) to restore the sighting margin at "
        "higher speed.",
        "Complete the black-box analysis (Section 5) to confirm the controller-level explanations "
        "given here.",
    ])


def sec_refs(R: Report):
    R.h1("References")
    refs = [
        "E. Olson, \"AprilTag: A robust and flexible visual fiducial system,\" in *Proc. IEEE Int. "
        "Conf. Robotics and Automation (ICRA)*, 2011, pp. 3400–3407.",
        "J. Wang and E. Olson, \"AprilTag 2: Efficient and robust fiducial detection,\" in *Proc. "
        "IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, 2016, pp. 4193–4198.",
        "S. Umeyama, \"Least-squares estimation of transformation parameters between two point "
        "patterns,\" *IEEE Trans. Pattern Analysis and Machine Intelligence*, vol. 13, no. 4, "
        "pp. 376–380, 1991.",
        "PX4 Autopilot, *PX4 User Guide (v1.15)*. [Online]. Available: https://docs.px4.io/v1.15/",
        "MAVLink, *MAVLink Developer Guide*. [Online]. Available: https://mavlink.io/",
        "Espressif Systems, *ESP-IDF Programming Guide (ESP32-S3)*. [Online]. Available: "
        "https://docs.espressif.com/projects/esp-idf/",
        "Nooploop, *LinkTrack P-A Series User Manual*. Included in the repository under "
        "`UWB_documentation/`.",
        "[TO BE COMPLETED: CDE1302 course materials (mini-project brief, lab handouts) as required "
        "by the module's citation style.]",
    ]
    for i, ref in enumerate(refs, 1):
        para = R.doc.add_paragraph()
        para.alignment = WD_ALIGN_PARAGRAPH.LEFT
        para.paragraph_format.left_indent = Cm(1.0)
        para.paragraph_format.first_line_indent = Cm(-1.0)
        r = para.add_run(f"[{i}]\t")
        set_font(r)
        add_rich(para, ref)


def appendices(R: Report):
    R.h1("Appendix A  Repository Structure and Branches")
    R.code([
        "CDE1302-Autonomous-UAV/",
        "├── cde1302/",
        "│   ├── main/       ESP32-S3 firmware",
        "│   ├── laptop/     course laptop tools",
        "│   ├── mission/    mission software, tests",
        "│   └── components/ MAVLink, esp-apriltag",
        "├── mechanical/     [EXTRAS] frame CAD, STLs",
        "├── electrical/     [EXTRAS] wiring, network",
        "├── UWB_documentation/  [EXTRAS] datasheets",
        "└── report/         this report and its data",
    ])
    R.p("Branches relevant to this report:")
    R.bullets([
        "`feature/loc-tags-waypoints`: firmware and mission software flown in the demonstration "
        "(waypoint-mode trajectory gate, localisation tags, drift correction).",
        "`feature/loc-tags`: the same with follow-mode (velocity feed-forward) trajectory gate.",
        "`claude-one-shot`: main development line of the mission (trajectory sweep, frame-pose "
        "tagging, first UWB integration).",
        "`uwb-testing`: [EXTRAS] UWB calibration, set-goal and lawnmower tests.",
        "`camera-fps`: faster OV3660 stream with laptop-only detection (not merged).",
        "`main`: the original course template.",
    ])

    R.h1("Appendix B  Key Parameters Used in the Demonstration")
    R.table("config", "Key mission, firmware and PX4 parameters.",
            ["Parameter", "Value", "Where"],
            [["Search area (arena frame)", "x 0.92–4.60 m, y −1.84–1.38 m, margin 0.30 m", "config.yaml `area`"],
             ["Lane spacing (max)", "1.0 m (Run 1), 3.0 m (Runs 2, 3)", "config.yaml `sweep`"],
             ["Trajectory sample period / planned speed", "0.20 s / 0.30 m/s", "config.yaml `flight`"],
             ["Hover check", "0.10 m for 2 s", "config.yaml `flight`"],
             ["Precision landing", "radius 0.08 m, ≤3 corrections, ≤10 s", "config.yaml `flight`, `localisation`"],
             ["Geofence margin / stale link / time limit", "0.5 m / 10 s / 600 s", "config.yaml `safety`"],
             ["Search tag size / localisation tag size", "0.112 m / 0.127 m (black square)", "config.yaml `tags`, `localisation`"],
             ["Camera mount", "90° pitch (down), 0.08 m forward", "config.yaml `tags`"],
             ["Detector gate", "margin ≥55, Hamming ≤1, pose error ≤0.5, range ≤1.5 m", "config.yaml `tags`"],
             ["Confirmation", "5 sightings within 0.20 m; outliers beyond 0.30 m", "config.yaml `tags`"],
             ["Drift model", "auto (selected hybrid in all runs), similarity", "config.yaml `localisation`"],
             ["Trajectory gate", "waypoint mode, 15 cm corner radius", "menuconfig"],
             ["Cruise altitude", "80 cm", "menuconfig `CRUISE_ALT_CM`"],
             ["Camera stream", "10 fps nominal, JPEG quality 60, QVGA", "menuconfig"],
             ["Horizontal speed limit", "0.5 / 1.0 [TBC] / 1.5 m/s", "PX4 `MPC_XY_VEL_MAX`"],
             ["UWB", "not sent to PX4", "config.yaml `uwb.send_to_px4: false`"]],
            widths_cm=[5.2, 6.4, 4.4])

    R.h1("Appendix C  Event Log of Run 1 (Excerpt)")
    ev = (ROOT / "data" / "flights" / "full-exploration" / "events.csv").read_text(encoding="utf-8")
    import csv
    import io
    rows = []
    keep = {"TAKEOFF_CMD", "FLYING_CONFIRMED", "HOVER_CONFIRMED", "HOME_TAG", "ALIGNED",
            "TRAJ_STARTED", "TAG_CONFIRMED", "TRAJ_END", "LAND_TARGET", "LOC_FIT", "LOC_DRIFT",
            "LAND_CMD", "MISSION_COMPLETE"}
    for r in csv.DictReader(io.StringIO(ev)):
        if r["event"] in keep and r["mission_s"]:
            rows.append([f"{float(r['mission_s']):.1f}", r["event"], r["details"]])
    R.table(None, "Selected events of Run 1 (`events.csv`).",
            ["t (s)", "Event", "Details"], rows, widths_cm=[1.6, 3.6, 10.8], label="Table C1. ")

    R.h1("Appendix D  Reproducing the Results")
    R.p("From the repository root, with Python 3.10 or newer and numpy, pandas, matplotlib, "
        "PyYAML and python-docx installed:")
    R.code([
        "python report/analyse_flights.py",
        "python report/build_report.py",
        "",
        "cd cde1302",
        "python -m pytest mission/tests -q",
        "python mission/mission.py --sim --auto --map",
    ])
    R.p("The first command writes the figures and `report/data/metrics.json`; the second writes "
        "`report/CDE1302_Final_Report_DRAFT.docx`. The test suite gave 432 passed and 1 skipped "
        "on 12 November 2026.")
    R.p("Instructions for continuing this report, including the ULog analysis, are in "
        "`report/AGENT_NOTES.md`.")


def main():
    R = Report()
    title_page(R)
    front_matter(R)
    sec_intro(R)
    sec_system(R)
    sec_software(R)
    sec_results(R)
    sec_ulog(R)
    sec_extras(R)
    sec_conclusion(R)
    sec_refs(R)
    appendices(R)
    R.save(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
