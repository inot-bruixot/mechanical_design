"""
Build Docs/OPTION_D.pdf -- a beginner's-guide PDF for the Option D pan/tilt
CAD model (mechanical_design/OPTION_D.step), matching the structure of
Docs/ASSEMBLY_PanTilt_Beginner_Guide.pdf (title page -> Chapter 1 BOM + part
cards -> Chapter 2 movement -> Chapter 3 mechanism + engineering note ->
Chapter 3 continued assembly steps -> disclaimer), per Docs/Requirements_
design_pan_tilt_D.docx and the pipeline documented in the mechanical-
engineer agent's memory (feedback_step_guide_pipeline.md).

Unlike the reference guide (which describes MACHINED/BOUGHT metal parts in
the original design), Option D's parts are honestly labelled 3D PRINTED or
BOUGHT, matching what this design actually is per the locked plan's
standing rule: every part must be either easily 3D-printable or a real
purchasable item.

Geometry source: re-runs build_option_d.py (via runpy) to get the exact
in-memory CadQuery `parts` dict and `COLOURS` used for the already-verified,
committed OPTION_D.step -- not a fresh re-derivation, so the pictures in
this PDF are guaranteed to match the exported STEP file exactly.
"""

import runpy
import math
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location

from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib import colors as rl_colors
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image as RLImage,
                                 Table, TableStyle, PageBreak, KeepTogether, HRFlowable)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER

# ---------------------------------------------------------------------------
# 0. Build the model (reuses build_option_d.py's exact, already-verified
#    geometry -- do not re-derive dimensions here).
# ---------------------------------------------------------------------------
print("Running build_option_d.py to get live geometry ...")
BUILD_NS = runpy.run_path("/home/inot/CLAUDE/mechanical_design/build_option_d.py")
parts = BUILD_NS["parts"]
COLOURS = BUILD_NS["COLOURS"]
print(f"Got {len(parts)} part instances.")

ASSET_DIR = "/home/inot/CLAUDE/Docs/option_d_guide_assets"
os.makedirs(ASSET_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# 1. Mesh once, cache triangles per part name
# ---------------------------------------------------------------------------
print("Meshing all parts ...")
tri_cache = {}
for name, wp in parts.items():
    shape = wp.val().wrapped
    BRepMesh_IncrementalMesh(shape, 0.3)
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    tris = []
    while exp.More():
        f = TopoDS.Face_s(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is not None:
            trsf = loc.Transformation()
            nodes = [tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1)]
            pts = np.array([[p.X(), p.Y(), p.Z()] for p in nodes])
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                tris.append([pts[a - 1], pts[b - 1], pts[c - 1]])
        exp.Next()
    tri_cache[name] = tris
print("Meshing done.")


def render_scene(names, path, elev=22, azim=35, highlight=None, dim_others=False,
                  figsize=(6, 6), colour_override=None, title=None):
    """Render a subset of parts (by name) into a PNG. `highlight` (set of
    names) get full color/opacity; everything else in `names` gets a faded
    grey if dim_others is True. colour_override: dict name->rgb to force."""
    fig = plt.figure(figsize=figsize)
    ax = fig.add_subplot(111, projection="3d")
    allpts = []
    for name in names:
        tris = tri_cache.get(name)
        if not tris:
            continue
        if colour_override and name in colour_override:
            c = colour_override[name]
        else:
            c = COLOURS.get(name, (0.8, 0.8, 0.8))
        alpha = 0.98
        if dim_others and highlight and name not in highlight:
            c = (0.82, 0.82, 0.82)
            alpha = 0.55
        pc = Poly3DCollection(tris, facecolor=c, edgecolor=(0.15, 0.15, 0.15),
                               linewidth=0.05, alpha=alpha)
        ax.add_collection3d(pc)
        for t in tris:
            allpts.extend(t)
    pts = np.array(allpts)
    if len(pts):
        c = pts.mean(axis=0)
        r = np.max(np.linalg.norm(pts - c, axis=1)) * 1.05
        ax.set_xlim(c[0] - r, c[0] + r)
        ax.set_ylim(c[1] - r, c[1] + r)
        ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    if title:
        ax.set_title(title, fontsize=10)
    plt.tight_layout(pad=0.1)
    plt.savefig(path, dpi=160, transparent=True)
    plt.close(fig)


def best_angle(name, candidates=((20, 30), (20, 120), (20, -60), (60, 30), (0, 0), (0, 90))):
    """Pick the (elev,azim) that maximizes 2D projected bbox area -- avoids
    edge-on views for thin/flat parts (per feedback_step_guide_pipeline.md)."""
    tris = tri_cache.get(name)
    if not tris:
        return candidates[0]
    pts = np.array([p for t in tris for p in t])
    c = pts.mean(axis=0)
    best = None
    best_area = -1
    for elev, azim in candidates:
        er, ar = math.radians(elev), math.radians(azim)
        # camera basis
        forward = np.array([math.cos(er) * math.cos(ar), math.cos(er) * math.sin(ar), math.sin(er)])
        up0 = np.array([0, 0, 1.0])
        right = np.cross(forward, up0)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0, 0])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        rel = pts - c
        u = rel @ right
        v = rel @ up
        area = (u.max() - u.min()) * (v.max() - v.min())
        if area > best_area:
            best_area = area
            best = (elev, azim)
    return best


# ---------------------------------------------------------------------------
# 2. BOM definition -- 16 unique part designs, plain-language descriptions.
#    Mirrors the reference guide's numbering/description style, but labels
#    are honest to Option D's own manufacturing (3D PRINTED / BOUGHT), not
#    copied from the machined-metal original.
# ---------------------------------------------------------------------------
BOM = [
    # (num, key, display name, type, qty, description)
    (1, "base_shell", "Main Housing", "3D PRINTED", 1,
     "The outer shell the whole mechanism is built inside. A round-profile printed housing that "
     "carries the pan gear, both pan bearings, and the pan motor. In this layout-study model it is "
     "shown as a solid block (not hollowed out) -- good enough to check overall size and clearances, "
     "not a print-ready wall-thickness design."),
    (2, "motor_base", "Pan Motor Mounting Plate", "3D PRINTED", 1,
     "A flat bracket that holds the pan-axis motor in a fixed position. Its screw holes and shaft "
     "clearance hole are positioned around the motor's real off-center output shaft, not its own "
     "centreline, so the shaft lines up correctly with the gear underneath."),
    (3, "gear2base_platform", "Pan Platform / Tilt Tower Base", "3D PRINTED", 1,
     "Sits on top of the pan shaft and rotates with it -- this is the part that carries everything "
     "else (motor, gears, shaft) around whenever the mechanism pans. Two towers on top hold the "
     "tilt shaft's two bearings."),
    (4, "tilt_motor_bracket", "Tilt Motor Mounting Plate", "3D PRINTED", 1,
     "A flat bracket, mounted behind the tilt axis, that holds the tilt-axis motor. Like the pan "
     "motor's bracket, its shaft clearance hole is positioned to match that motor's own real "
     "off-center shaft, not an idealised centred one."),
    (5, "tilt_hub", "Tilt Hub", "3D PRINTED", 1,
     "A small collar that clamps onto the tilt shaft with a flat-sided key and a grub screw. Carries "
     "two bolt-on flanges, one at each end, where the Payload Fork attaches."),
    (6, "cam_holder", "Payload Fork (\"Cam Holder\")", "3D PRINTED", 1,
     "The fork-shaped bracket that carries the payload. Bolted rigidly to the Tilt Hub's two "
     "flanges -- by design decision (see Chapter 3's engineering note) it does NOT have a "
     "free-spinning joint of its own; it only moves because the Tilt Hub underneath it moves."),
    (7, "pan_gear_60T", "Pan Gear, 60 tooth", "3D PRINTED", 1,
     "The big gear that produces the pan motion, keyed to the Pan Shaft and meshing with the pan "
     "motor's small pinion gear."),
    (8, "tilt_gear_60T", "Tilt Gear, 60 tooth", "3D PRINTED", 1,
     "The big gear that produces the tilt motion, keyed to the Tilt Shaft and meshing with the tilt "
     "motor's small pinion gear."),
    (9, "pan_pinion_20T", "Pinion Gear, 20 tooth", "3D PRINTED", 2,
     "A small gear pressed onto each motor's own output shaft. One drives the pan gear, the other "
     "drives the tilt gear -- in both cases the small gear turns 3 times for every 1 turn of the "
     "big gear it drives, trading speed for turning force (see Chapter 3)."),
    (10, "pan_motor_28BYJ48", "28BYJ-48 Stepper Motor", "BOUGHT", 2,
     "A real, off-the-shelf 28BYJ-48 stepper motor (the one named in the requirements). One drives "
     "pan, the other drives tilt. Its output shaft is not in the middle of the round motor body -- "
     "both mounting brackets (Parts 2 and 4) are shaped to line up with the real, off-center shaft "
     "position rather than an idealised centred one."),
    (11, "pan_bearing_608_bottom", "608ZZ Ball Bearing (pan shaft)", "BOUGHT", 2,
     "A real, off-the-shelf 608ZZ ball bearing (8mm bore) -- the same size used in skateboard "
     "wheels, cheap and easy to source. Two of these support the Pan Shaft, spaced apart for "
     "stability rather than using one large flanged bearing like the original design's."),
    (12, "tilt_bearing_625_A", "625ZZ Ball Bearing (tilt shaft)", "BOUGHT", 2,
     "A real, off-the-shelf 625ZZ ball bearing (5mm bore), one pressed into each of the platform's "
     "two towers, supporting the Tilt Shaft."),
    (13, "pan_shaft", "Pan Shaft", "BOUGHT", 1,
     "A short length of 8mm steel rod, cut to length and given a flat-sided section where the pan "
     "gear keys onto it. Steel rod is used here instead of a printed shaft for stiffness and wear "
     "resistance where it rides in the bearings."),
    (14, "tilt_shaft", "Tilt Shaft", "BOUGHT", 1,
     "A short length of 5mm steel rod, cut to length, with flat-sided sections where the tilt gear "
     "and the Tilt Hub key onto it."),
    (15, "pan_spacer_A", "Pan Shaft Spacer", "3D PRINTED", 2,
     "A short printed sleeve that sits on the pan shaft between the gear and each bearing, keeping "
     "everything positioned correctly along the shaft's length."),
    (16, "tilt_spacer_1", "Tilt Shaft Spacer", "3D PRINTED", 2,
     "A short printed sleeve on the tilt shaft, between each bearing and the Tilt Hub, keeping "
     "everything positioned correctly along the shaft's length."),
]
assert len(BOM) == 16

# movement classification for the Chapter 2 diagram (3-tier, same convention
# as the reference guide): grey = never moves, blue = carried by PAN,
# red = carried by PAN and additionally swings with TILT
GREY = {"base_shell", "motor_base", "pan_motor_28BYJ48", "pan_bearing_608_bottom", "pan_bearing_608_top"}
RED = {"tilt_shaft", "tilt_gear_60T", "tilt_hub", "cam_holder"}
# everything else in `parts` is BLUE (carried by pan only)
ALL_NAMES = list(parts.keys())
BLUE = set(ALL_NAMES) - GREY - RED - {"payload_placeholder"}

MOVE_TEXT = {
    "base_shell": "Never moves (fixed) -- everything else is built on or inside it.",
    "motor_base": "Bolted fixed to the housing; never moves.",
    "gear2base_platform": "Carried around by pan -- spinning it about the pan axis IS the pan motion.",
    "tilt_motor_bracket": "Carried around by pan (bolted to the platform); does not itself tilt.",
    "tilt_hub": "Carried around by pan; pivots with tilt (it IS the tilt motion, along with the tilt shaft/gear).",
    "cam_holder": "Carried around by pan and tilt; rigidly bolted to the Tilt Hub, no joint of its own.",
    "pan_gear_60T": "Carried around by pan; spinning it about the pan axis IS the pan motion.",
    "tilt_gear_60T": "Carried around by pan; spinning it about the tilt axis IS the tilt motion.",
    "pan_pinion_20T": "One copy spins in place to drive pan; the other is carried by pan and spins in place to drive tilt.",
    "pan_motor_28BYJ48": "One copy is bolted to the fixed plate (never moves); the other is carried by pan (bolted to the tilt bracket).",
    "pan_bearing_608_bottom": "Outer ring fixed in the housing; inner ring spins with the pan shaft.",
    "tilt_bearing_625_A": "Outer ring fixed in the tower (carried by pan); inner ring spins with the tilt shaft.",
    "pan_shaft": "Spins about the pan axis -- this rotation IS the pan motion.",
    "tilt_shaft": "Carried around by pan; spins about the tilt axis -- this rotation IS the tilt motion.",
    "pan_spacer_A": "Carried around by pan; spins with the pan shaft.",
    "tilt_spacer_1": "Carried around by pan; spins with the tilt shaft.",
}

# ---------------------------------------------------------------------------
# 3. Render images
# ---------------------------------------------------------------------------
print("Rendering hero shot ...")
hero_path = os.path.join(ASSET_DIR, "hero.png")
render_scene([n for n in ALL_NAMES if n != "payload_placeholder"], hero_path,
             elev=24, azim=40, figsize=(6, 6))

print("Rendering part cards ...")
part_img_paths = {}
for num, key, disp, typ, qty, desc in BOM:
    elev, azim = best_angle(key)
    p = os.path.join(ASSET_DIR, f"part_{num:02d}.png")
    render_scene([key], p, elev=elev, azim=azim, figsize=(3, 3))
    part_img_paths[key] = p

print("Rendering movement diagram ...")
move_colour_override = {}
for n in GREY:
    move_colour_override[n] = (0.6, 0.6, 0.6)
for n in BLUE:
    move_colour_override[n] = (0.27, 0.45, 0.85)
for n in RED:
    move_colour_override[n] = (0.85, 0.2, 0.2)
move_path = os.path.join(ASSET_DIR, "movement.png")
render_scene([n for n in ALL_NAMES if n != "payload_placeholder"], move_path,
             elev=24, azim=40, figsize=(6, 6), colour_override=move_colour_override)

print("Rendering gear mechanism schematic ...")
fig, ax = plt.subplots(figsize=(5.5, 3.2))
cd = BUILD_NS["CENTRE_DISTANCE"]
r_pin = BUILD_NS["GEAR_R_PIN"]
r_big = BUILD_NS["GEAR_R_BIG"]
ax.add_patch(plt.Circle((0, 0), r_pin, facecolor="#4577C4", edgecolor="black", linewidth=1.2))
ax.add_patch(plt.Circle((cd, 0), r_big, facecolor="#4577C4", edgecolor="black", linewidth=1.2, alpha=0.85))
ax.annotate("", xy=(0, r_pin + 6), xytext=(0, 2), arrowprops=dict(arrowstyle="->", lw=1.6))
ax.annotate("", xy=(cd, 2), xytext=(cd, r_big * 0.35), arrowprops=dict(arrowstyle="->", lw=1.6, color="#555"))
ax.text(0, r_pin + 8, "turns 3 times", ha="center", fontsize=9)
ax.text(cd, -r_big - 8, "turns 1 time,\nwith 3x the force", ha="center", fontsize=9)
ax.text(0, -r_pin - 5, f"Motor pinion\n{BUILD_NS['N_PIN']} teeth", ha="center", fontsize=8)
ax.text(cd, r_big + 5, f"Driven gear\n{BUILD_NS['N_BIG']} teeth", ha="center", fontsize=8)
ax.text(cd / 2, r_big + 14, "= 3 : 1 gear\nreduction (both axes)", ha="center", fontsize=9, style="italic")
ax.set_xlim(-r_pin - 15, cd + r_big + 15)
ax.set_ylim(-r_big - 18, r_big + 22)
ax.set_aspect("equal")
ax.axis("off")
plt.tight_layout()
gear_path = os.path.join(ASSET_DIR, "gear_schematic.png")
plt.savefig(gear_path, dpi=160, transparent=True)
plt.close(fig)

print("Rendering assembly-step images ...")
STEP_GROUPS = [
    ("Step 1 - Fit the bearings",
     "Press the two 608ZZ bearings into the bottom of the housing (Part 1), and the two 625ZZ "
     "bearings into the platform's twin towers (Part 12). All four are moving joints -- their outer "
     "rings are fixed in place; their inner rings are what everything else will spin inside.",
     ["base_shell", "pan_bearing_608_bottom", "pan_bearing_608_top"]),
    ("Step 2 - Fit the pan shaft, spacers, and pan gear",
     "Slide the Pan Shaft (Part 13) up through the two 608ZZ bearings from underneath -- moving "
     "joint, it should spin freely once through. Fit the two Pan Shaft Spacers (Part 15) to hold "
     "everything at the right height, and lock the Pan Gear (Part 7) onto the shaft's flat-sided "
     "section -- fixed joint, the gear and shaft now always turn together.",
     ["base_shell", "pan_bearing_608_bottom", "pan_bearing_608_top", "pan_shaft",
      "pan_spacer_A", "pan_spacer_B", "pan_gear_60T"]),
    ("Step 3 - Add the pan motor",
     "Fix the Pan Motor Mounting Plate (Part 2) in place -- fixed joint, it never moves. Fit the "
     "motor body, output shaft, and pinion gear (Parts 9/10, first copies) to the plate and check "
     "the pinion meshes cleanly with the pan gear below it.",
     ["base_shell", "pan_bearing_608_bottom", "pan_bearing_608_top", "pan_shaft",
      "pan_spacer_A", "pan_spacer_B", "pan_gear_60T", "motor_base", "pan_motor_28BYJ48",
      "pan_pinion_20T"]),
    ("Step 4 - Add the pan platform and tilt-motor bracket",
     "Fit the Pan Platform (Part 3) onto the top of the pan shaft -- fixed joint, it now travels "
     "around with pan. Fix the Tilt Motor Mounting Plate (Part 4) to the platform, and fit the "
     "second motor/shaft/pinion copy (Parts 9/10) to it.",
     ["base_shell", "pan_bearing_608_bottom", "pan_bearing_608_top", "pan_shaft",
      "pan_spacer_A", "pan_spacer_B", "pan_gear_60T", "motor_base", "pan_motor_28BYJ48",
      "pan_pinion_20T", "gear2base_platform", "tilt_motor_bracket", "tilt_motor_28BYJ48",
      "tilt_pinion_20T"]),
    ("Step 5 - Fit the tilt shaft, spacers, and tilt gear",
     "Slide the Tilt Shaft (Part 14) through the platform's two 625ZZ bearings -- moving joint. Fit "
     "the two Tilt Shaft Spacers (Part 16), then lock the Tilt Gear (Part 8) onto the shaft's "
     "flat-sided section and check it meshes cleanly with the tilt pinion.",
     ["base_shell", "pan_bearing_608_bottom", "pan_bearing_608_top", "pan_shaft",
      "pan_spacer_A", "pan_spacer_B", "pan_gear_60T", "motor_base", "pan_motor_28BYJ48",
      "pan_pinion_20T", "gear2base_platform", "tilt_motor_bracket", "tilt_motor_28BYJ48",
      "tilt_pinion_20T", "tilt_shaft", "tilt_bearing_625_A", "tilt_bearing_625_B",
      "tilt_spacer_1", "tilt_spacer_2", "tilt_gear_60T"]),
    ("Step 6 - Fit the Tilt Hub and Payload Fork",
     "Clamp the Tilt Hub (Part 5) onto the tilt shaft's flat-sided section with its grub screw -- "
     "fixed joint. Bolt the Payload Fork (Part 6) onto the Tilt Hub's two flanges -- also a fixed "
     "joint, by design decision (see Chapter 3): the fork does not have a free-spinning joint of "
     "its own, it only moves because the Tilt Hub beneath it moves.",
     [n for n in ALL_NAMES if n != "payload_placeholder"]),
]
step_img_paths = []
for i, (title, body, names) in enumerate(STEP_GROUPS, start=1):
    p = os.path.join(ASSET_DIR, f"step_{i:02d}.png")
    render_scene(names, p, elev=22, azim=38, figsize=(4.2, 4.2))
    step_img_paths.append(p)

print("All renders done.")

# ---------------------------------------------------------------------------
# 4. PDF assembly (reportlab platypus) -- colours match this project's
#    established guide palette (see feedback_step_guide_pipeline.md):
#    navy header #1A3A5C, 3D-printed label green #2E7D32, bought label
#    brown #8A5A00.
# ---------------------------------------------------------------------------
NAVY = rl_colors.HexColor("#1A3A5C")
GREEN = rl_colors.HexColor("#2E7D32")
BROWN = rl_colors.HexColor("#8A5A00")
TAN_BG = rl_colors.HexColor("#FBF3E3")
TAN_BORDER = rl_colors.HexColor("#C9A25C")
LIGHT_GREY = rl_colors.HexColor("#F2F2F2")

styles = getSampleStyleSheet()
title_style = ParagraphStyle("TitleBig", parent=styles["Title"], textColor=NAVY, fontSize=22, spaceAfter=4)
subtitle_style = ParagraphStyle("Subtitle", parent=styles["Normal"], fontSize=12, textColor=rl_colors.HexColor("#444444"), spaceAfter=14)
h1_style = ParagraphStyle("H1", parent=styles["Heading1"], textColor=NAVY, spaceBefore=6, spaceAfter=10)
h2_style = ParagraphStyle("H2", parent=styles["Heading2"], textColor=NAVY, spaceBefore=10, spaceAfter=6, fontSize=13)
body_style = ParagraphStyle("Body", parent=styles["Normal"], fontSize=9.3, leading=13)
small_style = ParagraphStyle("Small", parent=styles["Normal"], fontSize=8, leading=11, textColor=rl_colors.HexColor("#555555"))
part_name_style = ParagraphStyle("PartName", parent=styles["Normal"], fontSize=10.5, leading=13, fontName="Helvetica-Bold", textColor=NAVY)
label_green_style = ParagraphStyle("LabelGreen", parent=styles["Normal"], fontSize=8, leading=10, fontName="Helvetica-Bold", textColor=GREEN)
label_brown_style = ParagraphStyle("LabelBrown", parent=styles["Normal"], fontSize=8, leading=10, fontName="Helvetica-Bold", textColor=BROWN)
desc_style = ParagraphStyle("Desc", parent=styles["Normal"], fontSize=8.3, leading=11.5)
step_title_style = ParagraphStyle("StepTitle", parent=styles["Heading2"], textColor=NAVY, fontSize=11.5, spaceAfter=4)
italic_disclaimer = ParagraphStyle("Disclaimer", parent=styles["Normal"], fontSize=8, leading=11,
                                    textColor=rl_colors.HexColor("#666666"), alignment=TA_CENTER, fontName="Helvetica-Oblique")
engnote_style = ParagraphStyle("EngNote", parent=styles["Normal"], fontSize=8.5, leading=12,
                                textColor=rl_colors.HexColor("#5c4400"), fontName="Helvetica-Oblique")

OUT_PDF = "/home/inot/CLAUDE/Docs/OPTION_D.pdf"
doc = SimpleDocTemplate(OUT_PDF, pagesize=letter,
                         leftMargin=0.6 * inch, rightMargin=0.6 * inch,
                         topMargin=0.55 * inch, bottomMargin=0.55 * inch)

story = []

# ---- Title page ----
story.append(Paragraph("OPTION D", title_style))
story.append(Paragraph("A Beginner's Guide to the ~1/3-Scale, 28BYJ-48 Pan/Tilt Redesign", subtitle_style))
story.append(RLImage(hero_path, width=2.6 * inch, height=2.6 * inch))
story.append(Spacer(1, 10))
intro_txt = (
    "This guide explains, in plain language, the Option D pan/tilt mechanism -- a smaller rebuild of "
    "the original <b>ASSEMBLY_pan_tilt</b> reference design, sized around a real, off-the-shelf "
    "28BYJ-48 stepper motor (one for pan, one for tilt) instead of the original's generic placeholder "
    "motor. It has the same number of moving parts and the same two-axis mechanism as the original: "
    "it swivels left-right (<b>pan</b>) and has a second, separate motor and gear set that produces a "
    "tipping motion (<b>tilt</b>) for whatever is mounted on it. No prior mechanical design or CAD "
    "experience is assumed -- every technical word is explained the first time it is used."
)
story.append(Paragraph(intro_txt, body_style))
story.append(Spacer(1, 10))

W = 109.0
H = 137.0
D = 109.0
stats_data = [
    ["Part types / total pieces", "16 types / 22 pieces"],
    ["Overall assembled size (measured)", f"~ {W:.0f} x {H:.0f} x {D:.0f} mm (W x H x D)"],
    ["Size vs. the original (245 x 298 x 270mm)", "roughly 1/3 on two axes; taller relatively, driven by the real motor's fixed size (see Chapter 3)"],
    ["Pan / tilt gear reduction", "3 : 1 (each axis, both verified)"],
    ["Motor", "Real 28BYJ-48 stepper (x2) -- the original used an unbranded placeholder can"],
]
stats_tbl = Table([[Paragraph(f"<b>{a}</b>", small_style), Paragraph(b, small_style)] for a, b in stats_data],
                   colWidths=[2.6 * inch, 3.6 * inch])
stats_tbl.setStyle(TableStyle([
    ("GRID", (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#CCCCCC")),
    ("BACKGROUND", (0, 0), (0, -1), LIGHT_GREY),
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("LEFTPADDING", (0, 0), (-1, -1), 6), ("RIGHTPADDING", (0, 0), (-1, -1), 6),
    ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
]))
story.append(stats_tbl)
story.append(Spacer(1, 12))
org_txt = (
    "How this guide is organised: Chapter 1 introduces every physical part with a picture and a "
    "number. Chapters 2 and 3 refer back to those same part numbers throughout, so you can always "
    "flip back to Chapter 1 if you forget what \"Part 6\" is."
)
story.append(Paragraph(org_txt, body_style))
story.append(PageBreak())

# ---- Chapter 1: BOM ----
story.append(Paragraph("Chapter 1 - Parts of the Design", h1_style))
ch1_intro = (
    "Every physical piece of this mechanism is listed below with a number, a plain-language "
    "description of what it does, and how it's meant to be made. Parts labelled <font color='#2E7D32'>"
    "<b>3D PRINTED</b></font> are meant to be printed on an ordinary home FDM printer; parts labelled "
    "<font color='#8A5A00'><b>BOUGHT</b></font> are real, off-the-shelf items you would order rather "
    "than make. This mix (rather than the original design's machined metal) is a direct requirement of "
    "this redesign: every part must be either easily printable or purchasable, no exceptions."
)
story.append(Paragraph(ch1_intro, body_style))
story.append(Spacer(1, 8))

bom_rows = [[Paragraph("<b>#</b>", small_style), Paragraph("<b>Part name</b>", small_style),
             Paragraph("<b>Type</b>", small_style), Paragraph("<b>Qty</b>", small_style)]]
for num, key, disp, typ, qty, desc in BOM:
    lbl = Paragraph(f"<font color='#2E7D32'><b>{typ}</b></font>" if typ == "3D PRINTED"
                     else f"<font color='#8A5A00'><b>{typ}</b></font>", small_style)
    bom_rows.append([Paragraph(str(num), small_style), Paragraph(disp, small_style), lbl, Paragraph(str(qty), small_style)])
bom_tbl = Table(bom_rows, colWidths=[0.35 * inch, 3.6 * inch, 1.3 * inch, 0.5 * inch])
bom_tbl.setStyle(TableStyle([
    ("GRID", (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#CCCCCC")),
    ("BACKGROUND", (0, 0), (-1, 0), NAVY),
    ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ("LEFTPADDING", (0, 0), (-1, -1), 5), ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ("ROWBACKGROUNDS", (0, 1), (-1, -1), [rl_colors.white, LIGHT_GREY]),
]))
story.append(bom_tbl)
story.append(PageBreak())

# ---- Part cards, 5 per page (matches the reference guide's pacing) ----
CARDS_PER_PAGE = 5
for i in range(0, len(BOM), CARDS_PER_PAGE):
    chunk = BOM[i:i + CARDS_PER_PAGE]
    rows = []
    for num, key, disp, typ, qty, desc in chunk:
        img = RLImage(part_img_paths[key], width=1.15 * inch, height=1.15 * inch)
        lbl_style = label_green_style if typ == "3D PRINTED" else label_brown_style
        text_block = [
            Paragraph(f"{num}. {disp}", part_name_style),
            Paragraph(f"{typ}&nbsp;&nbsp;&nbsp;Qty: {qty}", lbl_style),
            Spacer(1, 3),
            Paragraph(desc, desc_style),
        ]
        rows.append([img, text_block])
    card_tbl = Table(rows, colWidths=[1.3 * inch, 5.1 * inch])
    card_tbl.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 8), ("BOTTOMPADDING", (0, 0), (-1, -1), 8),
        ("LINEBELOW", (0, 0), (-1, -2), 0.4, rl_colors.HexColor("#DDDDDD")),
    ]))
    story.append(card_tbl)
    story.append(PageBreak())

# ---- Chapter 2: Movement ----
story.append(Paragraph("Chapter 2 - Movement", h1_style))
ch2_intro = (
    "This mechanism moves in two intended directions, called axes:<br/><br/>"
    "<b>Pan</b> - swivels the top of the mechanism left and right, like shaking your head 'no'. "
    "The Pan Shaft (Part 13) and Pan Gear (Part 7) spin about a vertical axis running up through the "
    "housing. Everything mounted above them -- the whole tilt stage, Parts 3-12 and 14-16 -- is "
    "carried around with this rotation.<br/><br/>"
    "<b>Tilt</b> - a second, separate motor and gear pair tips the payload up and down, like nodding "
    "your head 'yes'. Because this tilt stage sits on top of the pan stage, it also swings left-right "
    "whenever pan happens."
)
story.append(Paragraph(ch2_intro, body_style))
story.append(Spacer(1, 8))
story.append(RLImage(move_path, width=3.4 * inch, height=3.4 * inch))
legend_txt = (
    "<font color='#999999'><b>Grey</b></font> = never moves (fixed) &nbsp;&nbsp; "
    "<font color='#4577D9'><b>Blue</b></font> = moves with PAN &nbsp;&nbsp; "
    "<font color='#D93333'><b>Red</b></font> = moves with PAN + TILT"
)
story.append(Paragraph(legend_txt, small_style))
story.append(Spacer(1, 10))

move_rows = [[Paragraph("<b>#</b>", small_style), Paragraph("<b>Part</b>", small_style), Paragraph("<b>How it moves</b>", small_style)]]
for num, key, disp, typ, qty, desc in BOM:
    move_rows.append([Paragraph(str(num), small_style), Paragraph(disp, small_style),
                       Paragraph(MOVE_TEXT.get(key, ""), small_style)])
move_tbl = Table(move_rows, colWidths=[0.35 * inch, 2.1 * inch, 3.3 * inch])
move_tbl.setStyle(TableStyle([
    ("GRID", (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#CCCCCC")),
    ("BACKGROUND", (0, 0), (-1, 0), NAVY), ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("LEFTPADDING", (0, 0), (-1, -1), 5), ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ("ROWBACKGROUNDS", (0, 1), (-1, -1), [rl_colors.white, LIGHT_GREY]),
]))
story.append(move_tbl)
story.append(PageBreak())

# ---- Chapter 3: mechanism + engineering note ----
story.append(Paragraph("Chapter 3 - How the Motion Is Actually Produced", h1_style))
ch3_txt = (
    "Both axes use exactly the same trick, just aimed in different directions. A small 20-tooth gear "
    "(the pinion, Part 9) is fixed to each motor's own output shaft, and it meshes with a much bigger "
    "60-tooth gear (Part 7 for pan, Part 8 for tilt) fixed to the shaft it drives. A gear is a toothed "
    "wheel that locks its teeth into a matching wheel so the two always turn together at a fixed "
    "ratio. This is called a <b>gear reduction</b>:"
)
story.append(Paragraph(ch3_txt, body_style))
story.append(Spacer(1, 6))
story.append(RLImage(gear_path, width=4.6 * inch, height=2.7 * inch))
ch3_txt2 = (
    "Because the big gear has three times as many teeth as the small one, the small gear must spin "
    "all the way round three times to make the big gear turn once. That trade means the gear pair "
    "also multiplies turning force (<b>torque</b> -- a twisting force, like the effort needed to turn "
    "a stiff doorknob) by 3x: whatever torque the small gear applies, the big gear delivers three "
    "times as much, at one third the speed. Both stages use the same 20-tooth/60-tooth pair, so both "
    "axes get an identical 3:1 speed-for-torque trade."
)
story.append(Paragraph(ch3_txt2, body_style))
story.append(Spacer(1, 6))
ch3_txt3 = (
    "The gear teeth themselves are not modelled with real involute tooth profiles in this file -- "
    "each gear is shown as a plain disc sized to the correct pitch diameter for meshing and clearance "
    "purposes, which is enough to verify the two gears sit at the right centre distance (measured "
    "here at 40.0mm on both stages) and don't collide with anything around them. A manufacturing "
    "drawing would need real cut teeth; this design-intent model does not."
)
story.append(Paragraph(ch3_txt3, body_style))
story.append(Spacer(1, 10))

engnote_txt = (
    "<b><i>Engineering note - the third rotation axis question, now resolved for this design:</i></b> "
    "The original reference file's \"Cam Holder\" part had pivot bores that lined up with neither the "
    "pan axis nor the gear-driven tilt axis -- a genuine open question about whether the original "
    "design intended a free-spinning third joint there, that could not be settled from the geometry "
    "alone. For Option D, the project owner made an explicit decision: the Payload Fork (Part 6, this "
    "design's equivalent part) is built <b>rigidly bolted</b> to the Tilt Hub beneath it, with no "
    "working joint of its own. It moves only because the Tilt Hub moves. This is a deliberate design "
    "choice for this redesign, not a re-discovery of what the original file's geometry actually meant."
)
story.append(Table([[Paragraph(engnote_txt, engnote_style)]], colWidths=[6.4 * inch],
                    style=TableStyle([
                        ("BACKGROUND", (0, 0), (-1, -1), TAN_BG),
                        ("BOX", (0, 0), (-1, -1), 0.8, TAN_BORDER),
                        ("LEFTPADDING", (0, 0), (-1, -1), 8), ("RIGHTPADDING", (0, 0), (-1, -1), 8),
                        ("TOPPADDING", (0, 0), (-1, -1), 7), ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
                    ])))
story.append(Spacer(1, 8))
size_note_txt = (
    "A note on overall size: this model measures {:.0f} x {:.0f} x {:.0f} mm (W x H x D), somewhat "
    "larger than a literal 1/3 scale of the original (~82 x 90 x 99mm) would give. That overshoot is "
    "not a mistake -- it comes directly from the real 28BYJ-48 motor's own fixed can size, mounting-tab "
    "spacing, and off-center shaft position, none of which shrink just because the rest of the design "
    "does, and this design uses two of them (one per axis). The project owner explicitly accepted this "
    "trade-off during planning: the real motor's needs take priority over hitting an exact 1/3 figure."
).format(W, H, D)
story.append(Paragraph(size_note_txt, body_style))
story.append(PageBreak())

# ---- Chapter 3 continued: Assembly ----
story.append(Paragraph("Chapter 3 (continued) - Assembly", h1_style))
assy_intro = (
    "Build the mechanism outward from the housing. This order is inferred from how the parts fit "
    "together in the geometry (which surfaces are pressed together vs. simply touching), not from any "
    "assembly instructions in the file, since STEP files don't store build order. Two kinds of joints "
    "come up throughout:<br/><br/>"
    "<b>Fixed / rigid joint</b> - bolted, keyed, or press-fit so the two parts never move relative to "
    "each other again.<br/>"
    "<b>Moving joint</b> - a bearing or a shaft spinning inside a hole, deliberately free to rotate."
)
story.append(Paragraph(assy_intro, body_style))
story.append(Spacer(1, 10))

# Step 1 alone, then steps in pairs (matches the reference guide's pacing)
title1, body1, _ = STEP_GROUPS[0]
step1_block = [Paragraph(title1, step_title_style), Paragraph(body1, body_style), Spacer(1, 4),
               RLImage(step_img_paths[0], width=2.6 * inch, height=2.6 * inch)]
story.append(KeepTogether(step1_block))
story.append(PageBreak())

for pair_start in range(1, len(STEP_GROUPS), 2):
    pair = STEP_GROUPS[pair_start:pair_start + 2]
    cells = []
    for j, (title, body, _) in enumerate(pair):
        idx = pair_start + j
        block = [Paragraph(title, step_title_style), Spacer(1, 3),
                 RLImage(step_img_paths[idx], width=2.3 * inch, height=2.3 * inch), Spacer(1, 4),
                 Paragraph(body, small_style)]
        cells.append(block)
    if len(cells) == 1:
        row = [cells[0], ""]
    else:
        row = cells
    step_tbl = Table([row], colWidths=[3.1 * inch, 3.1 * inch])
    step_tbl.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 10)]))
    story.append(step_tbl)
    story.append(Spacer(1, 14))

story.append(Spacer(1, 20))
disclaimer_txt = (
    "This is a beginner-oriented, plain-language explanation of the Option D pan/tilt CAD model "
    "(OPTION_D.step), built as a ~1/3-scale redesign of ASSEMBLY_pan_tilt.STEP around a real, "
    "off-the-shelf 28BYJ-48 stepper motor. Every measurement above was taken directly from that "
    "file's own 3D geometry, not guessed. This is a design-intent / layout-study model (bought "
    "components and gears are simplified stand-ins at the correct size, not release-ready "
    "manufacturing geometry) -- real-world fits, tolerances, and print settings should be checked "
    "against the STEP file before manufacturing anything from it."
)
story.append(Paragraph(disclaimer_txt, italic_disclaimer))


def add_footer(canvas, doc_):
    canvas.saveState()
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(rl_colors.HexColor("#888888"))
    canvas.drawString(0.6 * inch, 0.35 * inch, "OPTION_D - Beginner's Guide")
    canvas.drawRightString(letter[0] - 0.6 * inch, 0.35 * inch, f"Page {doc_.page}")
    canvas.restoreState()


doc.build(story, onFirstPage=add_footer, onLaterPages=add_footer)
print(f"\nWrote {OUT_PDF}")
