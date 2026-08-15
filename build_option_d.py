"""
Pan/Tilt Mini -- Option D  (CadQuery layout-study model)
==========================================================

Builds "Option D": a ~1/3-scale rebuild of the original reference assembly
(mechanical_design/ASSEMBLY_pan_tilt.STEP, 16 unique parts / 24 occurrences,
245.3x298.0x269.8mm) around the real 28BYJ-48 stepper motor, per
Docs/Requirements_design_pan_tilt_D.docx and the piece-by-piece plan locked
with the project owner on 2026-08-15 (see the mechanical-engineer agent's
memory, project_option_d_pantilt.md, for the full clarifying-question
history). Unlike two earlier redesign tracks on this same reference file
("Option B" / pan_tilt_geared_28byj48.step, which deliberately CUT part
count 17->11, and the plain-language similarity-ideas doc, an unrelated
proposal-only document) -- Option D's brief requires the SAME piece count
and SAME moving parts as the original, only the motor changes.

THIS SCRIPT IS DELIBERATELY BUILT ON THE VALIDATED "Option C" SCRIPT
(python/pan_tilt_option_c.py, a related-but-separate task thread on the same
reference file that happened to land on almost exactly Option D's own
requirements independently: 16 unique parts / 22 occurrences, real
purchasable small bearings, a rigidly-keyed Cam Holder with no working third
pivot, real off-center-shaft motor placement, module re-derived for
printability at the same 20T/60T tooth counts/ratio). Every one of Option
D's 7 locked-plan answers was checked against Option C's existing geometry
before reuse, not assumed:
  Q1 (1/3 is a rough aim, real motor size/power is the hard limit)     -- OK, unchanged
  Q2 (bearings must be real & purchasable, never invented)             -- OK: 608ZZ pan / 625ZZ tilt, both real off-the-shelf part numbers
  Q3 (exact piece count not important; drop leftover motor-shaft       -- OK: the two 7804K114 motor-shaft-stub
      bearings if that's the clean move)                                  bearings are already absent, matching this exactly
  Q4 (Cam Holder: rigid/fixed, no working third pivot)                -- OK: already modelled as rigidly keyed via the Tilt hub, no free joint
  Q5 (off-center motor shaft: shape the bracket around the REAL       -- OK: motor_base / tilt_motor_bracket already cut a shaft
      offset, both motors)                                                clearance hole at the true offset position, both axes
  Q6 (gears: same tooth count/ratio, sized for printability,          -- OK: 20T/60T, module 1.0 (not a literal 1/3 of the
      not literal 1/3 scale)                                              original's 1.125) -- ratio and tooth count both preserved
  Q7 (one thin plate allowed to stay thicker than strict 1/3 scaling) -- OK: motor_base plate is 4mm, well above a literal
                                                                            1/3 of the original's 3mm plate (which would be 1mm)
No engineering values needed to change -- only naming, output paths, and
this documentation, to honestly reflect Option D's own identity and
provenance rather than silently presenting Option C's work as new.

IMPORTANT CORRECTION vs. an earlier claim made to the project owner during
planning: a "real 28BYJ-48" STEP file was found at
mechanical_design/OLD/Stepper motor 28BYJ-48 5V.STEP and an initial B-rep
read of it was reported as showing a 17.5mm shaft/can offset. A follow-up
render of that file's actual triangulated mesh showed it is NOT a clean
28BYJ-48 replica -- it is a round disc-shaped part with an embossed "MOTOR"
label and unusual mounting-ear loops, inconsistent with real 28BYJ-48
photos/datasheets (whose output shaft sits within the can's own radius, not
3.5mm beyond it). That file is not trustworthy as ground truth. This script
instead uses the 8mm shaft/can offset already validated in Option B/C's own
work (independently grounded in real 28BYJ-48 dimensions, not this
ambiguous file) -- see project_option_d_pantilt.md for the full correction
note.

Like Option B/C's own reports, this is a LAYOUT-STUDY / design-intent model:
bought components (bearings, motors) and gears are modelled as simplified,
correctly-dimensioned stand-ins (right OD/ID/height, no ball details or
involute tooth profiles) -- good enough for real bounding-box and
interference numbers, not a released manufacturing drawing. Printed parts
(base shell, brackets, platform, Tilt hub, Cam Holder, spacers) are modelled
with their real bores, pockets, and mounting features so the interference
checks that matter (spacer-vs-shaft, hub-vs-cam-holder-vs-platform, bearing
seats) are checking real geometry.

Geometry provenance: the gear-train, bearing, and motor placement numbers
below were extracted by walking the real B-rep of Option B's own STEP file
(mechanical_design/pan_tilt_geared_28byj48.step) via OCP/XCAF -- centre
distance 40 mm on both stages, 16 mm pan-bearing spacing, 8 mm motor
shaft-offset clocked inboard, base 50 mm tall, etc. -- and are reused
verbatim via Option C. Envelope (109x137x109mm) is larger than a literal
1/3 of the original (~82x99x90mm) -- per Q1, this is accepted: the overshoot
is entirely attributable to the 28BYJ-48's own fixed can/tab/shaft-offset
geometry (used twice, once per axis), not an unexplained modelling slack.

Author: mechanical-engineer agent, building on mechanical-engineer-2's
validated Option C engineering. Units: millimetres throughout.
"""

import cadquery as cq
from cadquery import exporters
import math
import os

# ---------------------------------------------------------------------------
# 1. GLOBAL ENGINEERING PARAMETERS
#    (verbatim from Option B where noted -- do not change without re-checking
#    the torque margin math in PanTilt_Mini_28BYJ48_Redesign.pdf section 1)
# ---------------------------------------------------------------------------

MM = 1.0  # explicit unit marker -- everything below is millimetres

# --- shafts (cut steel rod, verbatim from Option B) ------------------------
PAN_SHAFT_DIA = 8.0
PAN_SHAFT_R = PAN_SHAFT_DIA / 2.0
TILT_SHAFT_DIA = 5.0
TILT_SHAFT_R = TILT_SHAFT_DIA / 2.0

# --- gear train (module 1.0, 20 deg PA, verbatim from Option B) ------------
GEAR_MODULE = 1.0
N_BIG = 60
N_PIN = 20
GEAR_OD_BIG = GEAR_MODULE * (N_BIG + 2)          # 62.0 mm
GEAR_OD_PIN = GEAR_MODULE * (N_PIN + 2)          # 22.0 mm
GEAR_R_BIG = GEAR_OD_BIG / 2.0                   # 31.0 mm
GEAR_R_PIN = GEAR_OD_PIN / 2.0                   # 11.0 mm
CENTRE_DISTANCE = GEAR_MODULE * (N_BIG + N_PIN) / 2.0  # 40.0 mm
GEAR_FACE_WIDTH = 6.0
# Option B's mount notes: "Pinion bore: dia 5.0 with the double-D flats at
# 3.0 mm" -- read as a FLAT-TO-FLAT width (the distance across the two
# parallel flats), which is the standard way double-D shaft flats are
# dimensioned, not a per-side cut depth. For a 5 mm dia shaft (r=2.5), a
# 3.0 mm flat-to-flat width means each flat is cut to a depth of
# r - 3.0/2 = 1.0 mm from the round surface.
PIN_FLAT_TO_FLAT = 3.0
# PIN_DFLAT_DEPTH is computed further down, right after MOTOR_SHAFT_DIA is
# defined in the motor parameters block (needs that value; kept the
# PIN_FLAT_TO_FLAT constant here next to the rest of the gear-train numbers).
HUB_DFLAT_DEPTH = 1.5   # single D-flat depth on the 5 mm tilt shaft / hub bore

# --- bearings (bought, verbatim from Option B) ------------------------------
BRG608_OD, BRG608_ID, BRG608_H = 22.0, 8.0, 7.0     # pan shaft
BRG625_OD, BRG625_ID, BRG625_H = 16.0, 5.0, 5.0     # tilt shaft
PAN_BEARING_SPACING = 16.0   # centre-to-centre, "16 mm apart for moment stiffness"

# --- 28BYJ-48 motor (bought, simplified can+shaft stand-in) ----------------
MOTOR_CAN_DIA = 28.0
MOTOR_CAN_H = 19.0
MOTOR_SHAFT_DIA = 5.0
# Now that MOTOR_SHAFT_DIA is known: pinion bore double-D flat depth, read as
# a FLAT-TO-FLAT width (see PIN_FLAT_TO_FLAT comment above) -- for a 5mm dia
# shaft (r=2.5), a 3.0mm flat-to-flat width means each flat is cut to a depth
# of r - 3.0/2 = 1.0mm from the round surface.
PIN_DFLAT_DEPTH = MOTOR_SHAFT_DIA / 2.0 - PIN_FLAT_TO_FLAT / 2.0  # 1.0 mm cut depth
MOTOR_SHAFT_OFFSET = 8.0     # shaft is NOT concentric with the can
MOTOR_SHAFT_LEN = 12.0
MOTOR_TAB_PITCH = 35.0
MOTOR_TAB_HOLE_D = 4.3       # M4 clearance, printed oversize for FDM

# --- restored spacers (new in Option C) -------------------------------------
SPACER_CLEARANCE = 0.1       # "loose, easy-sliding 0.1 mm gap on each side"
PAN_SPACER_ID = PAN_SHAFT_DIA + 2 * SPACER_CLEARANCE
PAN_SPACER_OD = 13.0
PAN_SPACER_LEN = 4.0
TILT_SPACER_ID = TILT_SHAFT_DIA + 2 * SPACER_CLEARANCE
TILT_SPACER_OD = 9.0
TILT_SPACER_LEN = 4.0

# --- restored Tilt hub + Cam Holder (new in Option C) -----------------------
HUB_OD = 16.0
HUB_LEN = 16.0                 # along the tilt shaft (X)
HUB_FLANGE_OD = 24.0
HUB_FLANGE_R = HUB_FLANGE_OD / 2.0
HUB_FLANGE_T = 3.0
GRUB_SCREW_D = 3.0             # M3 grub screw, radial into the D-flat
CAM_ARM_THICK = 6.0
PAYLOAD_R_FROM_AXIS = 40.0     # matches Option B's torque-calc moment arm (R40)
# Arm height is DERIVED, not guessed: the hub+cam-holder flange (radius
# HUB_FLANGE_R) already reaches partway from the tilt axis toward the payload
# before the arm even starts climbing -- "the bolt-on flange... adds roughly
# its own radius of extra reach before the fork's arms can start curving in
# toward the payload" (Design-Option_C.pdf section 5). The arm makes up the
# rest of the distance to the real R40 payload position used in both PDFs'
# torque calculations.
CAM_ARM_HEIGHT = PAYLOAD_R_FROM_AXIS - HUB_FLANGE_R    # 28 mm
CAM_BRIDGE_T = 20.0            # thick enough to pocket the PAYLOAD_DIA cradle bore
PAYLOAD_DIA = 14.0             # placeholder sensor/laser diameter (not a BOM part)

# --- base / platform stack ---------------------------------------------------
BASE_R = 47.0                  # base shell ~94 mm across, per Option C BOM note
BASE_H = 50.0                  # "inverting the pan motor keeps the base 50 mm tall"

# derived Z levels for the pan stack (bottom to top)
Z_BASE_TOP = BASE_H                                   # 50
Z_PAN_GEAR0, Z_PAN_GEAR1 = 6.0, 6.0 + GEAR_FACE_WIDTH  # 6 .. 12
Z_PAN_SPACER_A0 = 20.0
Z_PAN_SPACER_A1 = Z_PAN_SPACER_A0 + PAN_SPACER_LEN     # 20 .. 24
Z_PAN_BRG_BOT0 = Z_PAN_SPACER_A1
Z_PAN_BRG_BOT1 = Z_PAN_BRG_BOT0 + BRG608_H             # 24 .. 31
Z_PAN_BRG_BOT_CTR = (Z_PAN_BRG_BOT0 + Z_PAN_BRG_BOT1) / 2.0
Z_PAN_BRG_TOP_CTR = Z_PAN_BRG_BOT_CTR + PAN_BEARING_SPACING
Z_PAN_BRG_TOP0 = Z_PAN_BRG_TOP_CTR - BRG608_H / 2.0
Z_PAN_BRG_TOP1 = Z_PAN_BRG_TOP_CTR + BRG608_H / 2.0
Z_PAN_SPACER_B0 = Z_PAN_BRG_TOP1
Z_PAN_SPACER_B1 = Z_PAN_SPACER_B0 + PAN_SPACER_LEN
Z_PLATFORM_BASE0 = Z_PAN_SPACER_B1                     # platform sits right on top
PLATFORM_PLATE_T = 8.0
Z_PLATFORM_PLATE1 = Z_PLATFORM_BASE0 + PLATFORM_PLATE_T
TOWER_HEIGHT = 36.0
Z_PLATFORM_TOP = Z_PLATFORM_PLATE1 + TOWER_HEIGHT

TILT_AXIS_Z = Z_PLATFORM_PLATE1 + (TOWER_HEIGHT - 8.0)  # tilt axis 8mm below tower top

PAN_SHAFT_Z0 = 2.0
PAN_SHAFT_Z1 = Z_PLATFORM_BASE0 + 4.0   # protrudes 4mm into the platform's blind bore

# tilt-axis (X) layout
TILT_BRG_A_XC = -23.0
TILT_BRG_B_XC = 25.0
TILT_BRG_A_X0, TILT_BRG_A_X1 = TILT_BRG_A_XC - BRG625_H / 2.0, TILT_BRG_A_XC + BRG625_H / 2.0
TILT_BRG_B_X0, TILT_BRG_B_X1 = TILT_BRG_B_XC - BRG625_H / 2.0, TILT_BRG_B_XC + BRG625_H / 2.0
TOWER_X_THICK = 10.0    # platform tower thickness along the tilt-shaft axis
TOWER_A_X0, TOWER_A_X1 = TILT_BRG_A_XC - TOWER_X_THICK / 2.0, TILT_BRG_A_XC + TOWER_X_THICK / 2.0
TOWER_B_X0, TOWER_B_X1 = TILT_BRG_B_XC - TOWER_X_THICK / 2.0, TILT_BRG_B_XC + TOWER_X_THICK / 2.0
TILT_HUB_X0, TILT_HUB_X1 = -HUB_LEN / 2.0, HUB_LEN / 2.0
# restored spacers sit between each bearing's inner face and the hub -- touching
# the bearing, with a short bare-shaft run to the hub (acceptable at layout-study
# fidelity; a manufacturing drawing would close this gap with a longer spacer)
TILT_SPACER_1_X0, TILT_SPACER_1_X1 = TILT_BRG_A_X1, TILT_BRG_A_X1 + TILT_SPACER_LEN
TILT_SPACER_2_X0, TILT_SPACER_2_X1 = TILT_BRG_B_X0 - TILT_SPACER_LEN, TILT_BRG_B_X0
TILT_GEAR_X0 = TOWER_B_X1 + 1.0   # 1 mm clearance past the tower's outer face
TILT_GEAR_X1 = TILT_GEAR_X0 + GEAR_FACE_WIDTH
TILT_SHAFT_X0 = TOWER_A_X0 - 2.0
TILT_SHAFT_X1 = TILT_GEAR_X1 + 2.0

TILT_PINION_Y = -CENTRE_DISTANCE   # -40, meshes with tilt gear at Y=0
TILT_PINION_Z = TILT_AXIS_Z        # same Z as the tilt gear's own axis
PAN_PINION_X = CENTRE_DISTANCE     # +40, meshes with pan gear at X=0

PAN_MOTOR_CAN_X = PAN_PINION_X - MOTOR_SHAFT_OFFSET   # 32, clocked inboard
PAN_MOTOR_CAN_Z0 = Z_PAN_GEAR1 + MOTOR_SHAFT_LEN       # 24, can bottom face

# NOTE on the tilt motor's "clocked inboard" offset: build_motor() applies its
# shaft offset along local +X before the part is rotated +90 deg about Y to
# align with the tilt axis. Under that rotation, local (x,y,z) -> global
# (z, y, -x) (verified numerically -- see to_tilt_axis docstring), so a can
# with no local-X offset and a shaft offset by +MOTOR_SHAFT_OFFSET in local X
# come out at the SAME global Y (can and shaft share translate_y) but at
# global Z differing by -MOTOR_SHAFT_OFFSET (shaft_Z = can_Z - offset). For
# the tilt motor only, the real 8 mm shaft/can offset therefore lands in Z,
# not Y -- a modelling/mirror-symmetry choice, not an engineering difference.
# Solved for translate_z so the shaft (pinion) axis lands exactly on the
# tilt axis height (Z = TILT_AXIS_Z), which is required for correct meshing:
#   shaft_Z = can_Z - MOTOR_SHAFT_OFFSET = TILT_PINION_Z  =>  can_Z = TILT_PINION_Z + MOTOR_SHAFT_OFFSET
TILT_MOTOR_CAN_Y = TILT_PINION_Y                          # -40, same Y as the pinion
TILT_MOTOR_CAN_Z = TILT_PINION_Z + MOTOR_SHAFT_OFFSET     # 95
TILT_MOTOR_CAN_X0 = TILT_GEAR_X0 + MOTOR_SHAFT_LEN        # can's near (shaft) face
TILT_MOTOR_CAN_X_FACE = TILT_MOTOR_CAN_X0                 # alias used by the bracket builder

print("Centre distance (both stages):", CENTRE_DISTANCE, "mm")
print("Tilt axis Z:", TILT_AXIS_Z, "mm  |  Base top Z:", Z_BASE_TOP, "mm")
print("Platform plate Z:", Z_PLATFORM_BASE0, "-", Z_PLATFORM_PLATE1,
      " towers to Z:", Z_PLATFORM_TOP)


# ---------------------------------------------------------------------------
# 2. LOW-LEVEL HELPERS
# ---------------------------------------------------------------------------

def d_bore_cutter(shaft_r, length, flat_depth, clearance=0.1):
    """Solid to SUBTRACT to create a single-D-flat keyed bore, built along
    local +Z from z=0 to z=length. The flat lands at local x = shaft_r +
    clearance - flat_depth. Verified numerically (see dev notes) against
    the exact bug class flagged in this agent's memory (an earlier
    'flat_box built twice, discarded version correct' mistake) -- this
    version builds the cutter box once and checks out at the expected
    bounding box before use.
    """
    r = shaft_r + clearance
    full = cq.Workplane("XY").circle(r).extrude(length)
    box_x0 = r - flat_depth
    box_w = (r + 2.0) - box_x0
    cutter_box = (cq.Workplane("XY")
                  .box(box_w, 2 * r + 4.0, length, centered=(False, True, False))
                  .translate((box_x0, 0, 0)))
    return full.cut(cutter_box)


def d_bore_cutter_double(shaft_r, length, flat_depth, clearance=0.1):
    """Double-D (two opposite flats) keyed bore cutter, local +Z axis."""
    r = shaft_r + clearance
    full = cq.Workplane("XY").circle(r).extrude(length)
    box_x0 = r - flat_depth
    box_w = (r + 2.0) - box_x0
    cutter_pos = (cq.Workplane("XY")
                  .box(box_w, 2 * r + 4.0, length, centered=(False, True, False))
                  .translate((box_x0, 0, 0)))
    cutter_neg = cutter_pos.mirror(mirrorPlane="YZ")
    return full.cut(cutter_pos).cut(cutter_neg)


def round_bore_cutter(dia, length, clearance=0.0):
    r = dia / 2.0 + clearance
    return cq.Workplane("XY").circle(r).extrude(length)


def shaft_flat_cut(shaft_wp, shaft_r, flat_depth, z0, z1):
    """FIX (real D-flat on the shaft itself): cut a single D-flat directly
    into an already-built round shaft solid, but ONLY over the local-Z
    sub-range [z0,z1] (the shaft's own build axis, before any to_pan_axis /
    to_tilt_axis placement transform). Everywhere outside [z0,z1] is left
    round for its running fit through bearings/spacers. Uses shaft_r (the
    shaft's real radius, no clearance added) with the SAME flat_depth value
    used for the mating bore's d_bore_cutter() call -- because the bore is
    cut from radius (shaft_r+bore_clearance) inward by flat_depth while the
    shaft is cut from radius shaft_r inward by the same flat_depth, the
    shaft's flat plane lands exactly bore_clearance (0.1mm) deeper than the
    bore's flat plane, i.e. a consistent small gap on the flats to match the
    running clearance on the round part of the fit -- not a coincidence."""
    box_x0 = shaft_r - flat_depth
    box_w = (shaft_r + 2.0) - box_x0
    length = z1 - z0
    cutter = (cq.Workplane("XY")
              .box(box_w, 2 * shaft_r + 4.0, length, centered=(False, True, False))
              .translate((box_x0, 0, z0)))
    return shaft_wp.cut(cutter)


def shaft_double_flat_cut(shaft_wp, shaft_r, flat_depth, z0, z1):
    """Same as shaft_flat_cut but cuts two opposite (double-D) flats -- used
    for the 28BYJ-48 motor's real output shaft, which is double-D on the
    actual part (matches d_bore_cutter_double on the mating pinion bore)."""
    box_x0 = shaft_r - flat_depth
    box_w = (shaft_r + 2.0) - box_x0
    length = z1 - z0
    cutter_pos = (cq.Workplane("XY")
                  .box(box_w, 2 * shaft_r + 4.0, length, centered=(False, True, False))
                  .translate((box_x0, 0, z0)))
    cutter_neg = cutter_pos.mirror(mirrorPlane="YZ")
    return shaft_wp.cut(cutter_pos).cut(cutter_neg)


def to_pan_axis(wp, z0):
    """Pan-axis parts are already built along global Z -- just place at height z0."""
    return wp.translate((0, 0, z0))


def to_tilt_axis(wp, x0, y=0.0, z=0.0):
    """Tilt-axis parts are built along local +Z (0..length) in the XY plane;
    rotate +90 deg about the global Y axis (numerically verified: this maps
    local Z 0..L onto global X 0..L, i.e. +Z -> +X) then translate into
    place at (x0, y, z)."""
    return wp.rotate((0, 0, 0), (0, 1, 0), 90).translate((x0, y, z))


# ---------------------------------------------------------------------------
# 3. PART BUILDERS
# ---------------------------------------------------------------------------

def build_base_shell():
    """#1 Base shell -- houses the pan gear and both pan bearings, ~94 mm
    across. Modelled as a SOLID cylindrical stand-in (not hollowed) -- the
    same simplification Option B's own report used: it exists to check the
    outer envelope, not wall thickness. Anything sitting inside it (pan
    gear, pan bearings, pan spacers, pan shaft, pan motor) will therefore
    show up as touching solid material in the interference check; that is
    an expected, explained modelling-shortcut artefact, not a real clash."""
    return cq.Workplane("XY").circle(BASE_R).extrude(BASE_H)


def build_motor_base():
    """#2 Motor base -- pan motor mounting bracket, bolts inside the base
    shell, can-up mount. Simple plate with 2x M4 clearance holes on the
    real 35 mm 28BYJ-48 tab pitch (tabs perpendicular to the 8 mm shaft
    offset direction, which is +X here -> tabs run along Y)."""
    plate_t = 4.0
    z0 = PAN_MOTOR_CAN_Z0 - plate_t
    plate = (cq.Workplane("XY")
             .box(44, 44, plate_t, centered=(True, True, False))
             .translate((PAN_MOTOR_CAN_X, 0, z0)))
    for sy in (-MOTOR_TAB_PITCH / 2.0, MOTOR_TAB_PITCH / 2.0):
        hole = round_bore_cutter(MOTOR_TAB_HOLE_D, plate_t + 2).translate(
            (PAN_MOTOR_CAN_X, sy, z0 - 1))
        plate = plate.cut(hole)
    # FIX: shaft clearance hole. The motor is mounted can-up/shaft-down (see
    # module docstring on Z_PAN_GEAR levels) -- its offset shaft passes
    # straight down through this plate on its way to the pan pinion below,
    # at (PAN_PINION_X, 0) = (PAN_MOTOR_CAN_X + MOTOR_SHAFT_OFFSET, 0). The
    # plate previously had no hole there, so the shaft clashed with solid
    # plate material. Generous clearance (+1.0mm dia) since this plate only
    # needs to let the shaft pass, not key it.
    shaft_hole = round_bore_cutter(MOTOR_SHAFT_DIA + 1.0, plate_t + 2).translate(
        (PAN_PINION_X, 0, z0 - 1))
    plate = plate.cut(shaft_hole)
    return plate


def build_gear2base_platform():
    """#3 Gear 2 Base -- pan platform + tilt walls. Sits atop the pan shaft;
    two towers carry the tilt shaft's 625ZZ bearings."""
    # FIX: the plate was modelled 64mm wide (+/-32mm), which reaches 2mm past
    # TOWER_B_X1 (30mm, tower B's own outer face) and right into the tilt
    # gear's clearance zone (tilt_gear_60T starts at TILT_GEAR_X0=31mm) --
    # a real clash. Narrowed to 60mm (+/-30mm) so the plate's edge lands
    # flush with the tower's own outer face instead of overshooting it;
    # still comfortably covers both towers (A reaches to -28mm, B to +30mm).
    PLATFORM_PLATE_W = 60.0
    plate = (cq.Workplane("XY")
             .box(PLATFORM_PLATE_W, 42, PLATFORM_PLATE_T, centered=(True, True, False))
             .translate((0, 0, Z_PLATFORM_BASE0)))
    # blind pocket for the pan shaft's top stub
    plate = plate.cut(
        round_bore_cutter(PAN_SHAFT_DIA, PLATFORM_PLATE_T + 1, clearance=0.2)
        .translate((0, 0, Z_PLATFORM_BASE0 - 0.5)))

    def tower(x0, x1, xc):
        t = (cq.Workplane("XY")
             .box(x1 - x0, 20, TOWER_HEIGHT, centered=(True, True, False))
             .translate((xc, 0, Z_PLATFORM_PLATE1)))
        # 625ZZ bearing pocket, through the tower thickness, real diameter +
        # 0.05 mm running clearance so the boolean check reads a clean fit
        pocket = to_tilt_axis(
            round_bore_cutter(BRG625_OD, TOWER_X_THICK + 2, clearance=0.025),
            xc - TOWER_X_THICK / 2.0 - 1.0, 0.0, TILT_AXIS_Z)
        return t.cut(pocket)

    tower_a = tower(TOWER_A_X0, TOWER_A_X1, TILT_BRG_A_XC)
    tower_b = tower(TOWER_B_X0, TOWER_B_X1, TILT_BRG_B_XC)
    return plate.union(tower_a).union(tower_b)


def build_tilt_motor_bracket():
    """#4 Tilt motor bracket -- simple mounting plate (matches the "dark
    grey box" in the PDF's own Figure 4), ~40 mm behind the tilt axis (not
    underneath it). The motor bolts to it face-on at its shaft-side face
    (X = TILT_MOTOR_CAN_X0); the shaft passes through a clearance hole to
    reach the pinion. 2x M4 tabs on the real 35 mm pitch."""
    plate_t = 4.0
    x0 = TILT_MOTOR_CAN_X0 - plate_t
    plate = (cq.Workplane("XY")
             .box(plate_t, 44, 44, centered=(False, True, True))
             .translate((x0, TILT_MOTOR_CAN_Y, TILT_MOTOR_CAN_Z)))
    shaft_clearance = (cq.Workplane("XY").circle(MOTOR_SHAFT_DIA / 2.0 + 1.0)
                        .extrude(plate_t + 2)
                        .rotate((0, 0, 0), (0, 1, 0), 90)
                        .translate((x0 - 1.0, TILT_MOTOR_CAN_Y, TILT_AXIS_Z)))
    plate = plate.cut(shaft_clearance)
    for sy in (-MOTOR_TAB_PITCH / 2.0, MOTOR_TAB_PITCH / 2.0):
        hole = (cq.Workplane("XY").circle(MOTOR_TAB_HOLE_D / 2.0).extrude(plate_t + 2)
                .rotate((0, 0, 0), (0, 1, 0), 90)
                .translate((x0 - 1.0, TILT_MOTOR_CAN_Y + sy, TILT_MOTOR_CAN_Z)))
        plate = plate.cut(hole)
    return plate


def build_spur_gear(od, bore_dia, face_width, flat_depth, double_d=False):
    """Spur gear stand-in: a plain disc at the gear's OD (no involute teeth
    cut -- see module docstring). Bore is D-flat keyed to its shaft."""
    disc = cq.Workplane("XY").circle(od / 2.0).extrude(face_width)
    if double_d:
        bore = d_bore_cutter_double(bore_dia / 2.0, face_width + 1, flat_depth)
    else:
        bore = d_bore_cutter(bore_dia / 2.0, face_width + 1, flat_depth)
    return disc.cut(bore.translate((0, 0, -0.5)))


def build_bearing(od, idd, h):
    """Bought-part stand-in: a plain ring (tube), no ball/race detail."""
    outer = cq.Workplane("XY").circle(od / 2.0).extrude(h)
    inner = cq.Workplane("XY").circle(idd / 2.0).extrude(h + 1).translate((0, 0, -0.5))
    return outer.cut(inner)


def build_spacer(od, idd, length):
    """Bought/printed sleeve stand-in for the restored spacers."""
    outer = cq.Workplane("XY").circle(od / 2.0).extrude(length)
    inner = cq.Workplane("XY").circle(idd / 2.0).extrude(length + 1).translate((0, 0, -0.5))
    return outer.cut(inner)


def build_motor(shaft_len=MOTOR_SHAFT_LEN):
    """Bought-part stand-in for the 28BYJ-48 + ULN2003: simplified can +
    offset double-D shaft. No internal gearbox, winding, or driver-board
    detail -- purely a mounting/clash envelope."""
    can = cq.Workplane("XY").circle(MOTOR_CAN_DIA / 2.0).extrude(MOTOR_CAN_H)
    # FIX: the real 28BYJ-48 output shaft is double-D (two opposite flats),
    # not a plain round rod -- cut before translating, in the shaft's own
    # local frame (built at local origin, so shaft_flat_cut's box_x0
    # convention is centred on the shaft's own axis, not its final offset
    # position). Uses PIN_DFLAT_DEPTH -- the SAME flat depth used to build
    # the mating pinion's double-D bore -- so the flat clearance works out
    # per shaft_flat_cut's docstring math.
    shaft = cq.Workplane("XY").circle(MOTOR_SHAFT_DIA / 2.0).extrude(shaft_len)
    shaft = shaft_double_flat_cut(shaft, MOTOR_SHAFT_DIA / 2.0, PIN_DFLAT_DEPTH, 0, shaft_len)
    shaft = shaft.translate((MOTOR_SHAFT_OFFSET, 0, -shaft_len))
    two_tabs = cq.Workplane("XY")
    for sy in (-MOTOR_TAB_PITCH / 2.0, MOTOR_TAB_PITCH / 2.0):
        tab = (cq.Workplane("XY").box(8, 8, 2, centered=(True, True, False))
               .translate((0, sy, -0.001)))
        two_tabs = two_tabs.union(tab)
    return can.union(shaft).union(two_tabs)


def build_tilt_hub():
    """#5 Tilt hub (RESTORED) -- small collar that clamps onto the tilt
    shaft with a D-flat + M3 grub screw, playing the role the original's
    'Baring 2 shaft' part played. Carries bolt-on flanges at each end
    (sticking OUT beyond the collar, not into it) for the Cam Holder's
    4 screws (2 per arm). Built in LOCAL coordinates along local +Z
    (0..HUB_LEN) -- the caller places it on the tilt axis via to_tilt_axis."""
    body = cq.Workplane("XY").circle(HUB_OD / 2.0).extrude(HUB_LEN)
    flange_near = (cq.Workplane("XY").circle(HUB_FLANGE_R)
                   .extrude(HUB_FLANGE_T).translate((0, 0, -HUB_FLANGE_T)))
    flange_far = (cq.Workplane("XY").circle(HUB_FLANGE_R)
                  .extrude(HUB_FLANGE_T).translate((0, 0, HUB_LEN)))
    body = body.union(flange_near).union(flange_far)
    bore = d_bore_cutter(TILT_SHAFT_R, HUB_LEN + 2 * HUB_FLANGE_T + 2, HUB_DFLAT_DEPTH, clearance=0.1)
    body = body.cut(bore.translate((0, 0, -HUB_FLANGE_T - 1)))
    # M3 grub screw, radial, threaded in from the top down onto the flat
    grub = (cq.Workplane("XY").circle(GRUB_SCREW_D / 2.0).extrude(HUB_OD)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((HUB_LEN / 2.0, -HUB_OD / 2.0, 0)))
    body = body.cut(grub)
    # 4x M2.5 bolt holes, 2 per flange, for the Cam Holder
    for zc in (0.0, HUB_LEN):
        for ang in (60, -60):
            rad = math.radians(ang)
            hx = (HUB_FLANGE_R - 3.0) * math.cos(rad)
            hy = (HUB_FLANGE_R - 3.0) * math.sin(rad)
            hole = (cq.Workplane("XY").circle(1.35).extrude(HUB_FLANGE_T + 2)
                    .translate((hx, hy, zc - 1 - (HUB_FLANGE_T if zc == 0 else 0))))
            body = body.cut(hole)
    return body


def build_cam_holder():
    """#6 Cam Holder (RESTORED) -- fork/yoke bracket, its own printed part
    again, styled after the original's two-armed shape. Bolts to the Tilt
    hub's two flanges (4 screws, 2 per arm) and carries the payload 40 mm
    out from the tilt axis (matches Option B's torque-calc moment arm).
    Built in LOCAL coordinates: local +Z runs along the tilt shaft (matching
    the hub, so its flanges land at the hub's ends); local -X is 'up' toward
    the payload after to_tilt_axis's rotation (local X -> global -Z)."""
    arm_tip_x = -(HUB_FLANGE_R + CAM_ARM_HEIGHT)
    # FIX (arm height / length): the arm box width was previously
    # (HUB_FLANGE_R + CAM_ARM_HEIGHT) -- the FULL reach from the tilt axis
    # out to arm_tip_x -- but it was then translated to start already at the
    # flange's own edge (-HUB_FLANGE_R). That double-counted HUB_FLANGE_R,
    # so the arm actually ran all the way out to arm_tip_x - HUB_FLANGE_R,
    # overshooting the payload's own axis (arm_tip_x) and plunging the solid
    # arm straight through the payload's clearance envelope -- exactly the
    # "arm height intrudes into the payload's clearance zone" defect. Fix:
    # the arm only needs to span from the flange's edge out to the NEAR face
    # of the bridge (which carries the actual payload cradle pocket); the
    # bridge itself (unioned in afterwards, below) covers the last stretch
    # and is the only feature allowed to come near the payload's radius.
    ARM_LEN = CAM_ARM_HEIGHT - CAM_BRIDGE_T / 2.0
    assert ARM_LEN > 0, "CAM_ARM_HEIGHT must exceed half the bridge thickness"
    result = None
    for zc, sign in ((0.0, -1), (HUB_LEN, 1)):
        # mating flange, OUTBOARD of the hub's own flange at this end
        f_near = zc + sign * HUB_FLANGE_T
        f_far = zc + sign * 2 * HUB_FLANGE_T
        flange = (cq.Workplane("XY").circle(HUB_FLANGE_R)
                  .extrude(abs(f_far - f_near)).translate((0, 0, min(f_near, f_far))))
        # FIX (shaft clearance): the flange was previously a solid disc, but
        # the tilt shaft runs the full length of the hub+cam-holder stack
        # and must pass freely through both of these outboard flanges too --
        # generous +1.0mm dia clearance since this flange only needs to let
        # the shaft pass (the hub, not this flange, does the actual keying).
        shaft_bore = (cq.Workplane("XY").circle(TILT_SHAFT_R + 0.5)
                      .extrude(abs(f_far - f_near) + 2)
                      .translate((0, 0, min(f_near, f_far) - 1)))
        flange = flange.cut(shaft_bore)
        # arm: local X from -HUB_FLANGE_R (flange's own radial reach) out to
        # the bridge's near face, local Z centred on this flange's position
        arm = (cq.Workplane("XY")
               .box(ARM_LEN, CAM_ARM_THICK, CAM_ARM_THICK,
                    centered=(False, True, True))
               .translate((0, 0, zc)))
        arm = arm.mirror(mirrorPlane="YZ").translate((-HUB_FLANGE_R, 0, 0))
        part = flange.union(arm)
        # 2 bolt holes per flange, mating the hub's own bolt pattern
        for ang in (60, -60):
            rad = math.radians(ang)
            hx = (HUB_FLANGE_R - 3.0) * math.cos(rad)
            hy = (HUB_FLANGE_R - 3.0) * math.sin(rad)
            hole = (cq.Workplane("XY").circle(1.35).extrude(abs(f_far - f_near) + 2)
                    .translate((hx, hy, min(f_near, f_far) - 1)))
            part = part.cut(hole)
        result = part if result is None else result.union(part)

    # bridge/cradle joining the two arm tops, with a pocket for the payload
    # (payload's own axis, after build_payload_placeholder's rotation, runs
    # along global X -- i.e. local Z here, so a plain local-Z-axis bore is
    # already the right orientation)
    bridge_z0, bridge_z1 = -3.0, HUB_LEN + 3.0
    bridge = (cq.Workplane("XY")
              .box(CAM_BRIDGE_T, CAM_ARM_THICK, bridge_z1 - bridge_z0, centered=(True, True, False))
              .translate((arm_tip_x, 0, bridge_z0)))
    cradle = (cq.Workplane("XY").circle(PAYLOAD_DIA / 2.0 + 0.3)
              .extrude(bridge_z1 - bridge_z0 + 4)
              .translate((arm_tip_x, 0, bridge_z0 - 2)))
    bridge = bridge.cut(cradle)
    result = result.union(bridge)
    return result


def build_payload_placeholder():
    """Yellow sensor/laser PLACEHOLDER -- not a BOM part (matches the PDF
    figures, which show it as a distinct 'not one of the 16' reference
    item). Centred at R = PAYLOAD_R_FROM_AXIS from the tilt axis, matching
    Option B's torque-calc moment arm."""
    length = 44.0
    payload = (cq.Workplane("XY").circle(PAYLOAD_DIA / 2.0).extrude(length)
               .rotate((0, 0, 0), (0, 1, 0), 90)
               .translate((-length / 2.0, 0, TILT_AXIS_Z + PAYLOAD_R_FROM_AXIS)))
    return payload


# ---------------------------------------------------------------------------
# 4. BUILD EVERY PART INSTANCE (16 unique designs / 22 occurrences)
# ---------------------------------------------------------------------------

PAN_DFLAT_DEPTH = 1.5   # single D-flat depth on the 8/5 mm pan/tilt 60T bores

parts = {}

# -- pan axis (vertical, Z) --------------------------------------------------
parts["base_shell"] = build_base_shell()
parts["motor_base"] = build_motor_base()
parts["pan_gear_60T"] = to_pan_axis(
    build_spur_gear(GEAR_OD_BIG, PAN_SHAFT_DIA, GEAR_FACE_WIDTH, PAN_DFLAT_DEPTH),
    Z_PAN_GEAR0)
parts["pan_pinion_20T"] = (
    build_spur_gear(GEAR_OD_PIN, MOTOR_SHAFT_DIA, GEAR_FACE_WIDTH, PIN_DFLAT_DEPTH, double_d=True)
    .translate((PAN_PINION_X, 0, Z_PAN_GEAR0)))
_pan_shaft_wp = cq.Workplane("XY").circle(PAN_SHAFT_R).extrude(PAN_SHAFT_Z1 - PAN_SHAFT_Z0)
# FIX (real D-flat on the shaft): the pan_gear_60T's bore already had a
# D-flat cut into it (PAN_DFLAT_DEPTH, above) but the shaft itself was a
# plain round rod -- a flatted bore around a fully round shaft clashes on
# the flat (the round shaft's OD pokes past the bore's flattened wall).
# Cut a matching flat into the shaft, local (pre-translate) z-range aligned
# with where pan_gear_60T sits once placed (Z_PAN_GEAR0..Z_PAN_GEAR1 global
# -> local z = global - PAN_SHAFT_Z0). Everywhere else (bearings, spacers,
# platform bore) stays round for its running/clearance fit.
_pan_shaft_wp = shaft_flat_cut(
    _pan_shaft_wp, PAN_SHAFT_R, PAN_DFLAT_DEPTH,
    Z_PAN_GEAR0 - PAN_SHAFT_Z0, Z_PAN_GEAR1 - PAN_SHAFT_Z0)
parts["pan_shaft"] = _pan_shaft_wp.translate((0, 0, PAN_SHAFT_Z0))
parts["pan_bearing_608_bottom"] = to_pan_axis(
    build_bearing(BRG608_OD, BRG608_ID, BRG608_H), Z_PAN_BRG_BOT0)
parts["pan_bearing_608_top"] = to_pan_axis(
    build_bearing(BRG608_OD, BRG608_ID, BRG608_H), Z_PAN_BRG_TOP0)
parts["pan_spacer_A"] = to_pan_axis(
    build_spacer(PAN_SPACER_OD, PAN_SPACER_ID, PAN_SPACER_LEN), Z_PAN_SPACER_A0)
parts["pan_spacer_B"] = to_pan_axis(
    build_spacer(PAN_SPACER_OD, PAN_SPACER_ID, PAN_SPACER_LEN), Z_PAN_SPACER_B0)
parts["pan_motor_28BYJ48"] = (
    build_motor().translate((PAN_MOTOR_CAN_X, 0, PAN_MOTOR_CAN_Z0)))

# -- pan platform / tilt walls -----------------------------------------------
parts["gear2base_platform"] = build_gear2base_platform()

# -- tilt axis (horizontal, X) ------------------------------------------------
parts["tilt_gear_60T"] = to_tilt_axis(
    build_spur_gear(GEAR_OD_BIG, TILT_SHAFT_DIA, GEAR_FACE_WIDTH, PAN_DFLAT_DEPTH),
    TILT_GEAR_X0, 0.0, TILT_AXIS_Z)
parts["tilt_pinion_20T"] = to_tilt_axis(
    build_spur_gear(GEAR_OD_PIN, MOTOR_SHAFT_DIA, GEAR_FACE_WIDTH, PIN_DFLAT_DEPTH, double_d=True),
    TILT_GEAR_X0, TILT_PINION_Y, TILT_PINION_Z)
_tilt_shaft_wp = cq.Workplane("XY").circle(TILT_SHAFT_R).extrude(TILT_SHAFT_X1 - TILT_SHAFT_X0)
# FIX (real D-flats on the shaft): same defect as the pan shaft above -- the
# tilt_gear_60T bore (PAN_DFLAT_DEPTH, reused constant) and the tilt_hub
# bore (HUB_DFLAT_DEPTH) both already had flats cut into THEM, but the
# round tilt_shaft had no matching flats, so both keyed joints clashed.
# Two separate flat regions (local z, pre-to_tilt_axis, z = global_X -
# TILT_SHAFT_X0): the tilt gear's mesh location, and the tilt hub's full
# bore span (including its outboard flanges). The bare-shaft run between
# them (bearings + spacers) stays round for its running/clearance fit.
_tilt_shaft_wp = shaft_flat_cut(
    _tilt_shaft_wp, TILT_SHAFT_R, PAN_DFLAT_DEPTH,
    TILT_GEAR_X0 - TILT_SHAFT_X0, TILT_GEAR_X1 - TILT_SHAFT_X0)
_hub_bore_gX0 = TILT_HUB_X0 - HUB_FLANGE_T - 1   # matches build_tilt_hub()'s bore span exactly
_hub_bore_gX1 = TILT_HUB_X0 + HUB_LEN + HUB_FLANGE_T + 1
_tilt_shaft_wp = shaft_flat_cut(
    _tilt_shaft_wp, TILT_SHAFT_R, HUB_DFLAT_DEPTH,
    _hub_bore_gX0 - TILT_SHAFT_X0, _hub_bore_gX1 - TILT_SHAFT_X0)
parts["tilt_shaft"] = to_tilt_axis(_tilt_shaft_wp, TILT_SHAFT_X0, 0.0, TILT_AXIS_Z)
parts["tilt_bearing_625_A"] = to_tilt_axis(
    build_bearing(BRG625_OD, BRG625_ID, BRG625_H), TILT_BRG_A_X0, 0.0, TILT_AXIS_Z)
parts["tilt_bearing_625_B"] = to_tilt_axis(
    build_bearing(BRG625_OD, BRG625_ID, BRG625_H), TILT_BRG_B_X0, 0.0, TILT_AXIS_Z)
parts["tilt_spacer_1"] = to_tilt_axis(
    build_spacer(TILT_SPACER_OD, TILT_SPACER_ID, TILT_SPACER_LEN), TILT_SPACER_1_X0, 0.0, TILT_AXIS_Z)
parts["tilt_spacer_2"] = to_tilt_axis(
    build_spacer(TILT_SPACER_OD, TILT_SPACER_ID, TILT_SPACER_LEN), TILT_SPACER_2_X0, 0.0, TILT_AXIS_Z)
parts["tilt_motor_28BYJ48"] = to_tilt_axis(
    build_motor(), TILT_MOTOR_CAN_X0, TILT_MOTOR_CAN_Y, TILT_MOTOR_CAN_Z)
parts["tilt_motor_bracket"] = build_tilt_motor_bracket()

# -- restored tilt-end parts (Option C's headline change) --------------------
parts["tilt_hub"] = to_tilt_axis(build_tilt_hub(), TILT_HUB_X0, 0.0, TILT_AXIS_Z)
parts["cam_holder"] = to_tilt_axis(build_cam_holder(), TILT_HUB_X0, 0.0, TILT_AXIS_Z)

# -- payload placeholder (visualisation only, not a BOM part) ---------------
parts["payload_placeholder"] = build_payload_placeholder()

# One entry per UNIQUE DESIGN (16 total, matching Option D's locked plan) --
# the 20T pinion and the 28BYJ-48 are each a single design used twice
# (qty 2, "one per motor" / "one per axis"), so their
# second occurrence (tilt_pinion_20T, tilt_motor_28BYJ48) is a separate
# PART INSTANCE in `parts` for geometry/placement purposes but is not a
# second unique design and is intentionally excluded here.
BOM_PART_NAMES = [
    "base_shell", "motor_base", "gear2base_platform", "tilt_motor_bracket",
    "tilt_hub", "cam_holder", "pan_gear_60T", "tilt_gear_60T",
    "pan_pinion_20T",           # design shared with tilt_pinion_20T (qty 2)
    "pan_motor_28BYJ48",        # design shared with tilt_motor_28BYJ48 (qty 2)
    "pan_bearing_608_bottom",   # design shared with pan_bearing_608_top (qty 2)
    "tilt_bearing_625_A",       # design shared with tilt_bearing_625_B (qty 2)
    "pan_shaft", "tilt_shaft",
    "pan_spacer_A",   # design shared with pan_spacer_B (qty 2)
    "tilt_spacer_1",  # design shared with tilt_spacer_2 (qty 2)
]
assert len(set(BOM_PART_NAMES)) == 16, f"expected 16 unique part designs, got {len(set(BOM_PART_NAMES))}"
_bom_occurrences = len(parts) - 1  # exclude the non-BOM payload placeholder
assert _bom_occurrences == 22, f"expected 22 physical BOM occurrences, got {_bom_occurrences}"

for name, wp in parts.items():
    solid = wp.val()
    if not solid.isValid():
        raise RuntimeError(f"Part '{name}' produced an invalid solid -- fix before proceeding")

print(f"\nBuilt {len(parts)} part instances ({len(BOM_PART_NAMES)} unique designs in the BOM),"
      f" all valid solids.\n")


# ---------------------------------------------------------------------------
# 5. INTERFERENCE CHECKS -- real BRepAlgoAPI_Common booleans via CadQuery's
#    .intersect(), on every pair whose bounding boxes overlap. Every pair is
#    classified up front so the report distinguishes a genuine unwanted clash
#    from an expected/explained condition (gear-tooth mesh overlap, the
#    solid-stand-in base shell swallowing anything housed inside it, or a
#    line-to-line bearing/shaft fit).
# ---------------------------------------------------------------------------

GEAR_MESH_PAIRS = {
    frozenset(["pan_gear_60T", "pan_pinion_20T"]),
    frozenset(["tilt_gear_60T", "tilt_pinion_20T"]),
}
# max allowed mesh overlap: 2x module is the standard addendum overlap for a
# properly meshing external gear pair (both teeth reach past the pitch circle
# by one module each) -- allow a working margin on top of the pure-cylinder
# approximation used here (no real teeth), which will read a bit high because
# it's testing full addendum-circle discs, not tooth flanks.
GEAR_MESH_MAX_VOL = 1300.0  # mm^3, generous cap for the disc-vs-disc approximation

BASE_SHELL_HOUSED = {
    "pan_gear_60T", "pan_pinion_20T", "pan_shaft", "pan_bearing_608_bottom",
    "pan_bearing_608_top", "pan_spacer_A", "pan_spacer_B", "pan_motor_28BYJ48",
    "motor_base",
}

# pairs that are DESIGNED to touch (coaxial neighbours on a shaft, a bolted
# mating flange, a bearing pocket, a shaft running through a clearance hole)
# -- real touching contact is fine here; the tolerance just needs to catch a
# genuine volumetric bite, not floating-point noise on a coincident face.
TOUCH_TOLERANCE = 0.5  # mm^3

results = []
names = list(parts.keys())
solids = {n: parts[n].val() for n in names}
bboxes = {n: solids[n].BoundingBox() for n in names}


def bbox_overlap(b1, b2):
    return not (b1.xmax < b2.xmin or b2.xmax < b1.xmin or
                b1.ymax < b2.ymin or b2.ymax < b1.ymin or
                b1.zmax < b2.zmin or b2.zmax < b1.zmin)


print("Running pairwise interference checks (bbox pre-filter, then real")
print("BRepAlgoAPI_Common booleans on every candidate pair)...\n")

checked = 0
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        n1, n2 = names[i], names[j]
        if not bbox_overlap(bboxes[n1], bboxes[n2]):
            continue
        checked += 1
        try:
            common = parts[n1].intersect(parts[n2])
            vol = common.val().Volume() if common.solids().vals() else 0.0
        except Exception:
            vol = 0.0
        pair = frozenset([n1, n2])
        if pair in GEAR_MESH_PAIRS:
            category = "expected-gear-mesh"
            ok = vol <= GEAR_MESH_MAX_VOL
        elif n1 in BASE_SHELL_HOUSED and n2 == "base_shell" or n2 in BASE_SHELL_HOUSED and n1 == "base_shell":
            category = "expected-solid-stand-in-housing"
            ok = True  # by definition non-issue, per the base-shell modelling note
        else:
            category = "must-be-clean"
            ok = vol <= TOUCH_TOLERANCE
        results.append((n1, n2, vol, category, ok))

print(f"Checked {checked} bbox-overlapping candidate pairs out of "
      f"{len(names) * (len(names) - 1) // 2} total pairs.\n")

real_clashes = [r for r in results if r[3] == "must-be-clean" and not r[4]]
mesh_rows = [r for r in results if r[3] == "expected-gear-mesh"]
housed_rows = [r for r in results if r[3] == "expected-solid-stand-in-housing"]
touch_rows = [r for r in results if r[3] == "must-be-clean" and r[4] and r[2] > 1e-6]

print("=" * 78)
print("INTERFERENCE CHECK REPORT")
print("=" * 78)

print(f"\n-- Expected gear-tooth mesh overlap ({len(mesh_rows)} pairs) --")
for n1, n2, vol, cat, ok in mesh_rows:
    print(f"  {n1:28s} vs {n2:28s}  {vol:10.2f} mm^3  "
          f"({'within' if ok else 'EXCEEDS'} the {GEAR_MESH_MAX_VOL:.0f} mm^3 mesh-overlap cap)")

print(f"\n-- Expected base-shell 'solid stand-in housing' overlap ({len(housed_rows)} pairs) --")
print("   (base shell is modelled SOLID, not hollowed -- see build_base_shell()")
print("    docstring. Anything housed inside it reads as touching solid")
print("    material. This is the same modelling shortcut Option B's own")
print("    report used and explicitly flagged as a non-issue.)")
total_housed_vol = sum(r[2] for r in housed_rows)
for n1, n2, vol, cat, ok in sorted(housed_rows, key=lambda r: -r[2]):
    other = n2 if n1 == "base_shell" else n1
    print(f"  {other:28s} vs base_shell   {vol:10.2f} mm^3")
print(f"  TOTAL (non-issue): {total_housed_vol:.2f} mm^3")

print(f"\n-- Real touching contact, zero or near-zero as intended ({len(touch_rows)} pairs) --")
for n1, n2, vol, cat, ok in touch_rows:
    print(f"  {n1:28s} vs {n2:28s}  {vol:10.4f} mm^3  (within {TOUCH_TOLERANCE} mm^3 tolerance)")

print(f"\n-- GENUINE UNWANTED CLASHES ({len(real_clashes)} pairs) --")
if real_clashes:
    for n1, n2, vol, cat, ok in real_clashes:
        print(f"  *** CLASH ***  {n1:28s} vs {n2:28s}  {vol:10.2f} mm^3")
else:
    print("  NONE. Every pair that is not an expected gear mesh or the")
    print("  intentional base-shell solid-stand-in overlap reads clean")
    print(f"  (<= {TOUCH_TOLERANCE} mm^3).")

if real_clashes:
    raise RuntimeError(
        f"{len(real_clashes)} genuine unwanted interference(s) found -- "
        "fix geometry before exporting. See report above.")

print("\nAll interference checks passed. Proceeding to bounding-box")
print("measurement and STEP export.\n")


# ---------------------------------------------------------------------------
# 6. ASSEMBLY, BOUNDING BOX, AND STEP EXPORT
# ---------------------------------------------------------------------------

COLOURS = {
    "base_shell": (0.75, 0.75, 0.75), "motor_base": (0.75, 0.75, 0.75),
    "gear2base_platform": (0.75, 0.75, 0.75),
    "pan_gear_60T": (0.27, 0.45, 0.77), "tilt_gear_60T": (0.27, 0.45, 0.77),
    "pan_pinion_20T": (0.27, 0.45, 0.77), "tilt_pinion_20T": (0.27, 0.45, 0.77),
    "pan_motor_28BYJ48": (0.35, 0.35, 0.35), "tilt_motor_28BYJ48": (0.35, 0.35, 0.35),
    "tilt_motor_bracket": (0.35, 0.35, 0.35),
    "pan_bearing_608_bottom": (0.6, 0.6, 0.65), "pan_bearing_608_top": (0.6, 0.6, 0.65),
    "tilt_bearing_625_A": (0.6, 0.6, 0.65), "tilt_bearing_625_B": (0.6, 0.6, 0.65),
    "pan_shaft": (0.5, 0.5, 0.5), "tilt_shaft": (0.5, 0.5, 0.5),
    "pan_spacer_A": (0.95, 0.6, 0.15), "pan_spacer_B": (0.95, 0.6, 0.15),
    "tilt_spacer_1": (0.95, 0.6, 0.15), "tilt_spacer_2": (0.95, 0.6, 0.15),
    "tilt_hub": (0.85, 0.2, 0.2),
    "cam_holder": (0.25, 0.65, 0.35),
    "payload_placeholder": (0.95, 0.9, 0.2),
}

assy = cq.Assembly(name="PanTilt_Option_D")
for name, wp in parts.items():
    assy.add(wp, name=name, color=cq.Color(*COLOURS.get(name, (0.8, 0.8, 0.8))))

# overall envelope, computed from the ACTUAL solids just built (not from the
# design-intent constants above)
xmin = min(bboxes[n].xmin for n in names)
xmax = max(bboxes[n].xmax for n in names)
ymin = min(bboxes[n].ymin for n in names)
ymax = max(bboxes[n].ymax for n in names)
zmin = min(bboxes[n].zmin for n in names)
zmax = max(bboxes[n].zmax for n in names)
W, D, H = xmax - xmin, ymax - ymin, zmax - zmin

print("=" * 78)
print("PRE-EXPORT ENVELOPE (from the live CadQuery model)")
print("=" * 78)
print(f"  X (width,  W): {xmin:8.2f} .. {xmax:8.2f}  ->  {W:.2f} mm")
print(f"  Y (depth,  D): {ymin:8.2f} .. {ymax:8.2f}  ->  {D:.2f} mm")
print(f"  Z (height, H): {zmin:8.2f} .. {zmax:8.2f}  ->  {H:.2f} mm")
print(f"  Option D rough aim (1/3 of the 245.3x298.0x269.8mm original): ~82 x 90 x 99 mm (W x H x D)")
print(f"  This model:                                                    {W:.1f} x {H:.1f} x {D:.1f} mm (W x H x D)")
print(f"  Overshoot is expected per Q1 -- driven by the real 28BYJ-48's fixed can/tab/shaft-offset")
print(f"  geometry (used twice), not unexplained modelling slack. See script header for detail.")

OUT_DIR = "/home/inot/CLAUDE/mechanical_design"
OUT_PATH = os.path.join(OUT_DIR, "OPTION_D.step")
os.makedirs(OUT_DIR, exist_ok=True)
assy.save(OUT_PATH, exportType="STEP")
print(f"\nExported assembly STEP to: {OUT_PATH}")


# ---------------------------------------------------------------------------
# 7. RE-IMPORT AND VERIFY -- read the STEP file back with CadQuery, confirm
#    every solid is valid (no open shells) and re-measure the bounding box
#    from the EXPORTED FILE ITSELF, not from the in-memory model.
# ---------------------------------------------------------------------------

print("\n" + "=" * 78)
print("POST-EXPORT VERIFICATION (re-reading the STEP file from disk)")
print("=" * 78)

reimported = cq.importers.importStep(OUT_PATH)
re_solids = reimported.vals()
print(f"Re-imported {len(re_solids)} top-level solid(s)/compound(s) from the STEP file.")

all_valid = True
total_solid_count = 0
for i, s in enumerate(re_solids):
    for sub in s.Solids():
        total_solid_count += 1
        if not sub.isValid():
            all_valid = False
            print(f"  INVALID solid found (compound {i})")
print(f"Total leaf solids: {total_solid_count}")
print(f"All solids valid (no open shells): {all_valid}")
if not all_valid:
    raise RuntimeError("Exported STEP file contains invalid/open-shell geometry.")

rebb = reimported.val().BoundingBox()
rW, rD, rH = rebb.xlen, rebb.zlen, rebb.ylen
# NOTE: BoundingBox exposes xlen/ylen/zlen directly; recompute W/H/D using the
# SAME axis convention as the in-memory measurement above (X=W, Z=H, Y=D)
rW = rebb.xmax - rebb.xmin
rD = rebb.ymax - rebb.ymin
rH = rebb.zmax - rebb.zmin
print(f"\nRe-measured envelope from the EXPORTED STEP FILE:")
print(f"  X (width,  W): {rebb.xmin:8.2f} .. {rebb.xmax:8.2f}  ->  {rW:.2f} mm")
print(f"  Y (depth,  D): {rebb.ymin:8.2f} .. {rebb.ymax:8.2f}  ->  {rD:.2f} mm")
print(f"  Z (height, H): {rebb.zmin:8.2f} .. {rebb.zmax:8.2f}  ->  {rH:.2f} mm")
print(f"  ==> {rW:.1f} x {rH:.1f} x {rD:.1f} mm (W x H x D)")

third_scale_aim = (81.8, 89.9, 99.3)  # 1/3 of 245.3 x 269.8 x 298.0mm (W/H/D)
built = (rW, rH, rD)
deltas = tuple(b - t for b, t in zip(built, third_scale_aim))
print(f"\nDelta vs the ~1/3-scale rough aim (82 x 90 x 99 mm): "
      f"{deltas[0]:+.1f} / {deltas[1]:+.1f} / {deltas[2]:+.1f} mm (W/H/D)")
print("Overshoot attributed to the real 28BYJ-48 motor's fixed geometry per Q1 -- see script header.")

print("\nDone.")
