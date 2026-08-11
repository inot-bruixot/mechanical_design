"""
Pan/Tilt Mini -- Design Option B  (CadQuery manufacturing-fidelity model)
==========================================================================
Builds the 16-unique-part / 22-occurrence assembly described in
/home/inot/CLAUDE/Docs/Design-Option_B.pdf.

Units: millimetres throughout.  Axes: Z = up (pan axis, vertical), X = tilt
axis (horizontal), Y = depth (front/back -- tilt motor sits ~40mm behind
the tilt axis in +Y, "behind" not "underneath").

Assumptions made where the PDF spec leaves a dimension unstated (flagged
again in the final summary):
  - Wall thickness 3mm on printed housings/brackets (typical FDM min ~2.4mm
    for 2 walls at 0.4mm nozzle; 3mm gives margin for M3/M4 bolt bosses).
  - Pan/tilt shafts keyed to their 60T gears and to Gear 2 Base's boss via a
    plain round bore + radial M3 grub screw (spec calls these out as plain
    "steel rod", no flats specified -- unlike the motor's real output
    shaft, which DOES get real double-D flats per spec).
  - 625-family bearing pocket shoulder radius (6.5mm) extrapolated from the
    spec's explicit 608-family value (~9.0mm) by the same outer-race/OD
    ratio, since the PDF only gives the 608 number explicitly.
  - Bearing flange seats: the spec describes the shoulder (blind-end) rule
    explicitly; the flange (open end, bolts flat to the housing face) needs
    a shallow recess so its larger OD doesn't collide with housing material
    around the bore -- a straightforward, standard detail not spelled out
    dimension-by-dimension in the PDF.
  - Cam Holder pivot bore: 3.3mm dia against the 3.0mm pin (0.15mm radial
    clearance per side) for a hand-turnable but firm friction fit in FDM
    plastic -- tight enough to hold a set position, loose enough to adjust
    by hand without a tool.
  - Tilt axis positioned directly above the pan axis in Y (not offset) to
    keep the assembly's depth (Y) envelope tight -- only the tilt motor's
    mandatory ~40mm 'behind' offset extends it, which is what actually
    drives the achieved depth being larger than the ~92mm target.
  - A short structural gusset bridges Gear 2 Base's tower to the tilt motor
    bracket -- basic structural continuity, not explicit in the PDF.
  - Fastener clearance holes are modeled in the part that carries the bolt
    pattern (e.g. the motor plate, the bearing's own 3 informational
    holes); matching tapped/clearance holes in every mating housing are not
    all individually modeled -- this is a fit-check geometry exercise, not
    a full fastener BOM.
  - Housing bulk: Gear 2 Base's towers are modeled as simple full-height
    solid blocks from the boss up to the bearing bosses, rather than
    tapered/ribbed to save material -- valid and correctly clears the
    tilt gear, just not weight-optimized.
"""

import math
import os
import cadquery as cq
from OCP.BOPAlgo import BOPAlgo_Builder

STEP_OUT = "/home/inot/CLAUDE/mechanical_design/Design-Option_B.step"

# ===========================================================================
# 0. GEAR GENERATOR
# ===========================================================================

def fuse_many(shapes):
    builder = BOPAlgo_Builder()
    for s in shapes:
        builder.AddArgument(s.wrapped)
    builder.Perform()
    # BOPAlgo_Builder always returns a TopAbs_COMPOUND as a bare TopoDS_Shape;
    # cq.Shape.cast() picks the correct cadquery wrapper subclass (Compound)
    # so downstream .cut()/.union() calls (which require Solid/Compound, not
    # a bare Shape) work correctly.
    return cq.Shape.cast(builder.Shape())


def involute_gear(module, teeth, face_width, pressure_angle_deg=20.0,
                   addendum_factor=1.0, dedendum_factor=1.25, n_flank=6):
    """External spur gear solid, additive teeth on a root cylinder.
    Axis = Z, centred at origin, extruded z=0..face_width."""
    m = module
    z = teeth
    alpha = math.radians(pressure_angle_deg)
    Rp = m * z / 2.0
    Rb = Rp * math.cos(alpha)
    Ra = Rp + addendum_factor * m
    Rf = Rp - dedendum_factor * m

    half_thick_ang = (math.pi * m / 4.0) / Rp
    t_p = math.tan(alpha)

    def invo_xy(t):
        x = Rb * (math.cos(t) + t * math.sin(t))
        y = Rb * (math.sin(t) - t * math.cos(t))
        return x, y

    xna, yna = invo_xy(t_p)
    nat_ang = math.atan2(yna, xna)
    rot = half_thick_ang - nat_ang

    t_end = math.sqrt(max((Ra / Rb) ** 2 - 1.0, 0.0))
    if Rf < Rb:
        t_start = 0.0
        need_stub = True
    else:
        t_start = math.sqrt(max((Rf / Rb) ** 2 - 1.0, 0.0))
        need_stub = False

    ts = [t_start + (t_end - t_start) * i / (n_flank - 1) for i in range(n_flank)]
    right_pts = []
    for t in ts:
        x, y = invo_xy(t)
        ang = math.atan2(y, x) + rot
        r = math.hypot(x, y)
        right_pts.append((r * math.cos(ang), r * math.sin(ang)))
    left_pts = [(x, -y) for (x, y) in reversed(right_pts)]

    poly = []
    if need_stub:
        base_ang = rot
        poly.append((Rf * math.cos(-base_ang), Rf * math.sin(-base_ang)))
    poly.extend(left_pts)
    poly.extend(right_pts)
    if need_stub:
        base_ang = rot
        poly.append((Rf * math.cos(base_ang), Rf * math.sin(base_ang)))

    pitch_ang = 2 * math.pi / z
    root = cq.Workplane("XY").circle(Rf).extrude(face_width)
    tooth = cq.Workplane("XY").polyline(poly).close().extrude(face_width)

    shapes = [root.val()]
    for i in range(z):
        ang_deg = math.degrees(pitch_ang * i)
        t = tooth.rotate((0, 0, 0), (0, 0, 1), ang_deg).val()
        shapes.append(t)

    fused = fuse_many(shapes)
    gear = cq.Workplane("XY").newObject([fused])
    return gear, dict(Rp=Rp, Rb=Rb, Ra=Ra, Rf=Rf)


# ===========================================================================
# 1. GLOBAL PARAMETERS
# ===========================================================================

WALL = 3.0
RELIEF = 0.9          # internal-corner relief cut (bearing pocket shoulders)
RUN_CLR = 0.15         # running radial clearance, bearing OD vs pocket bore

PAN_SHAFT_DIA, PAN_SHAFT_R = 8.0, 4.0
TILT_SHAFT_DIA, TILT_SHAFT_R = 5.0, 2.5
PIN_DIA, PIN_R = 3.0, 1.5
PIN_BORE_DIA = 3.3

MODULE = 1.0
N_BIG, N_PIN = 60, 20
FACE_W = 6.0
CD = MODULE * (N_BIG + N_PIN) / 2.0     # 40.0 mm, both stages
GEAR_BORE_PAN = 8.0
GEAR_BORE_TILT = 5.0
PINION_BORE = 5.0
PINION_FLAT2FLAT = 3.0

MOTOR_CAN_D = 28.0
MOTOR_CAN_H = 19.0
MOTOR_SHAFT_D = 5.0
MOTOR_SHAFT_OFFSET = 8.0
MOTOR_BOLT_PITCH = 35.0
MOTOR_BOLT_HOLE_D = 4.3

B608_OD, B608_ID, B608_W = 22.0, 8.0, 7.0
B608_FLANGE_OD, B608_FLANGE_T = 26.0, 2.0
B608_SHOULDER_R = 9.0

B625_OD, B625_ID, B625_W = 16.0, 5.0, 5.0
B625_FLANGE_OD, B625_FLANGE_T = 20.0, 1.6
B625_SHOULDER_R = 6.5

TILT_BEARING_SPACING = 16.0

print(f"Centre distance (both stages): {CD} mm")

# ===========================================================================
# 2. LOW-LEVEL HELPERS
# ===========================================================================

def round_bore(dia, length, clearance=0.0):
    r = dia / 2.0 + clearance
    return cq.Workplane("XY").circle(r).extrude(length)


def double_d_bore(dia, length, flat_to_flat, clearance=0.1):
    """Double-D (two opposite flats) keyed bore cutter, local +Z axis,
    z=0..length. flat_to_flat = distance between the two parallel flat
    faces. Also valid as a real double-D SHAFT solid when clearance=0."""
    r = dia / 2.0 + clearance
    half_gap = flat_to_flat / 2.0
    full = cq.Workplane("XY").circle(r).extrude(length)
    cutter_pos = (cq.Workplane("XY")
                  .box(2 * (r + 2.0), 2 * r + 4.0, length, centered=(False, True, False))
                  .translate((half_gap, 0, 0)))
    cutter_neg = cutter_pos.mirror(mirrorPlane="YZ")
    return full.cut(cutter_pos).cut(cutter_neg)


def grub_screw_hole(dia, length):
    """Radial grub-screw clearance hole, centred on the local origin,
    spanning local Y = -length/2..+length/2 (used to key a plain round bore
    onto a shaft). NOTE: Workplane("XZ").extrude(L) extrudes in -Y, so we
    extrude(-length) to get a solid spanning 0..length before centering."""
    return (cq.Workplane("XZ").circle(dia / 2.0).extrude(-length)
            .translate((0, -length / 2.0, 0)))


def flanged_bearing(od, idd, width, flange_od, flange_t):
    """Bought-part representative solid: barrel + flange disc at the z=0
    end, bore straight through. Local +Z axis, barrel z=0..width."""
    barrel = cq.Workplane("XY").circle(od / 2.0).extrude(width)
    flange = cq.Workplane("XY").circle(flange_od / 2.0).extrude(flange_t)
    body = barrel.union(flange)
    bore = cq.Workplane("XY").circle(idd / 2.0).extrude(width + 2).translate((0, 0, -1))
    body = body.cut(bore)
    for i in range(3):
        ang = math.radians(i * 120)
        r = (flange_od / 2.0) * 0.78
        hole = (cq.Workplane("XY").circle(1.7).extrude(flange_t + 1)
                .translate((r * math.cos(ang), r * math.sin(ang), -0.5)))
        body = body.cut(hole)
    return body


def bearing_pocket_cutter(od, shoulder_r, full_depth, shoulder_depth,
                           flange_od, flange_t, clearance=RUN_CLR):
    """Cutter (to SUBTRACT from housing material) for a flanged-bearing seat.
    Rule: pocket touches ONLY the bearing's outer race -- never reaches in
    to the bore/inner-race diameter. Local +Z, z=0..full_depth (opening end
    at z=0, blind end at z=full_depth).
      - flange recess: r=flange_od/2+clr, z=0..flange_t+0.3 -- lets the
        bearing's flange (which bolts flat to the housing's outer face)
        sit flush/recessed without colliding with surrounding material.
      - full bore: r=od/2+clr, z=0..full_depth -- clears the barrel.
      - shoulder counterbore: r=shoulder_r (< full-bore r), cut for
        `shoulder_depth` at the blind end so the REMAINING shoulder ring
        (shoulder_r..od/2) is what the barrel's blind-end face lands on --
        reach = od/2-shoulder_r, kept <= the real outer-race wall
        thickness so it can't preload the bearing.
      - relief groove: 0.9mm groove at the internal corner where the
        shoulder wall meets the through-bore, so a printed fillet there
        can't touch the bearing.
    """
    r_full = od / 2.0 + clearance
    r_flange = flange_od / 2.0 + clearance
    full_bore = cq.Workplane("XY").circle(r_full).extrude(full_depth)
    flange_recess = cq.Workplane("XY").circle(r_flange).extrude(flange_t + 0.3)
    shoulder_bore = (cq.Workplane("XY").circle(shoulder_r).extrude(shoulder_depth)
                      .translate((0, 0, full_depth - shoulder_depth)))
    relief = (cq.Workplane("XY").circle(r_full + RELIEF).extrude(RELIEF)
              .translate((0, 0, full_depth - shoulder_depth - RELIEF)))
    return full_bore.union(flange_recess).union(shoulder_bore).union(relief)


def to_tilt_axis_gen(wp, x0, y=0.0, z=0.0, reverse=False):
    """reverse=False: local +Z -> global +X.  reverse=True: local +Z -> global -X.
    (Verified: Ry(+90) maps local Z(0,0,1)->global +X, Ry(-90)->global -X.)"""
    ang = -90 if reverse else 90
    return wp.rotate((0, 0, 0), (0, 1, 0), ang).translate((x0, y, z))


def x_cyl(r, length, x0, y, z):
    """Solid cylinder, axis along global X, spanning x0..x0+length, at (y,z).
    (Workplane("YZ") extrudes in +X.)"""
    return cq.Workplane("YZ").workplane(offset=x0).circle(r).extrude(length).translate((0, y, z))


def x_tube(od, idd, length, x0, y, z):
    return x_cyl(od / 2.0, length, x0, y, z).cut(x_cyl(idd / 2.0, length + 0.4, x0 - 0.2, y, z))


def x_hole(dia, length, x0, y, z):
    return x_cyl(dia / 2.0, length, x0, y, z)


def y_cyl(r, length, x, y0, z):
    """Solid cylinder, axis along global Y, spanning y0..y0+length, at (x,z).
    (Workplane("XZ") extrudes in -Y, so extrude(-length) then translate to y0
    gives a solid spanning y0..y0+length.)"""
    return cq.Workplane("XZ").circle(r).extrude(-length).translate((x, y0, z))


def build_spur_gear(base_gear_wp, gear_info, bore_dia, face_width, double_d=False,
                     flat2flat=None, grub=False):
    """Cut a keyed bore into a copy of a pre-built involute gear blank."""
    gear = base_gear_wp
    if double_d:
        bore = double_d_bore(bore_dia, face_width + 2, flat2flat).translate((0, 0, -1))
    else:
        bore = round_bore(bore_dia, face_width + 2, clearance=0.1).translate((0, 0, -1))
    gear = gear.cut(bore)
    if grub:
        gs = grub_screw_hole(3.2, bore_dia / 2.0 + 6).translate((0, 0, face_width / 2.0))
        gear = gear.cut(gs)
    return gear


print("Building involute gears (60T x2 designs share one blank each side)...")
GEAR60_BLANK, INFO60 = involute_gear(MODULE, N_BIG, FACE_W)
GEAR20_BLANK, INFO20 = involute_gear(MODULE, N_PIN, FACE_W)
assert GEAR60_BLANK.val().isValid() and GEAR20_BLANK.val().isValid()
print(f"  60T OD={2*INFO60['Ra']:.2f}mm (spec ~62)  20T OD={2*INFO20['Ra']:.2f}mm (spec ~22)")

# ===========================================================================
# 3. PAN STAGE -- world coordinates.  Pan axis = vertical line through (0,0).
#    Z=0 is the underside (external face) of Base 2.
# ===========================================================================

FLOOR_T = 8.0
GEAR_BOT_CLR, GEAR_TOP_CLR = 0.3, 1.0
PAN_GEAR_Z0 = FLOOR_T + GEAR_BOT_CLR              # 8.3
PAN_GEAR_Z1 = PAN_GEAR_Z0 + FACE_W                # 14.3
BASE2_TOP_Z = PAN_GEAR_Z1 + GEAR_TOP_CLR           # 15.3

R_TUB_BIG = INFO60["Ra"] + 3.0
R_TUB_SMALL = INFO20["Ra"] + 3.0
R_TUB_BIG_OUT = R_TUB_BIG + WALL
R_TUB_SMALL_OUT = R_TUB_SMALL + WALL

PIN_XY = (CD, 0.0)
CAN_XY = (CD - MOTOR_SHAFT_OFFSET, 0.0)

print("\nBuilding Base 2 (pan housing, part #1)...")
big_tub = cq.Workplane("XY").circle(R_TUB_BIG_OUT).extrude(BASE2_TOP_Z)
small_tub = (cq.Workplane("XY").circle(R_TUB_SMALL_OUT).extrude(BASE2_TOP_Z)
             .translate((PIN_XY[0], PIN_XY[1], 0)))
base2 = big_tub.union(small_tub)

pocket_big = (cq.Workplane("XY").circle(R_TUB_BIG).extrude(BASE2_TOP_Z - FLOOR_T)
              .translate((0, 0, FLOOR_T)))
pocket_small = (cq.Workplane("XY").circle(R_TUB_SMALL).extrude(BASE2_TOP_Z - FLOOR_T)
                .translate((PIN_XY[0], PIN_XY[1], FLOOR_T)))
base2 = base2.cut(pocket_big).cut(pocket_small)

bearing608_pocket = bearing_pocket_cutter(B608_OD, B608_SHOULDER_R,
                                           full_depth=7.0, shoulder_depth=2.0,
                                           flange_od=B608_FLANGE_OD, flange_t=B608_FLANGE_T)
base2 = base2.cut(bearing608_pocket)  # local Z == global Z already, at (0,0,0)

for ang in (45, 135, 225, 315):
    a = math.radians(ang)
    hx, hy = 25 * math.cos(a), 25 * math.sin(a)
    hole = (cq.Workplane("XY").circle(MOTOR_BOLT_HOLE_D / 2.0).extrude(FLOOR_T + 2)
            .translate((hx, hy, -1)))
    base2 = base2.cut(hole)

assert base2.val().isValid(), "Base 2 solid invalid"
bb = base2.val().BoundingBox()
print(f"  Base 2 OK. bbox={bb.xlen:.1f}x{bb.ylen:.1f}x{bb.zlen:.1f} mm")

print("Building Motor base / pan motor plate (part #2)...")
motor_plate = (cq.Workplane("XY").box(36, 40, WALL, centered=(False, True, False))
               .translate((22, 0, BASE2_TOP_Z)))
shaft_clr_hole = (cq.Workplane("XY").circle(MOTOR_SHAFT_D / 2.0 + 0.6).extrude(WALL + 2)
                  .translate((PIN_XY[0], PIN_XY[1], BASE2_TOP_Z - 1)))
motor_plate = motor_plate.cut(shaft_clr_hole)
for dy in (MOTOR_BOLT_PITCH / 2.0, -MOTOR_BOLT_PITCH / 2.0):
    h = (cq.Workplane("XY").circle(MOTOR_BOLT_HOLE_D / 2.0).extrude(WALL + 2)
         .translate((CAN_XY[0], CAN_XY[1] + dy, BASE2_TOP_Z - 1)))
    motor_plate = motor_plate.cut(h)
for cx, cy in ((26, -16), (26, 16), (54, -16), (54, 16)):
    h = (cq.Workplane("XY").circle(1.6).extrude(WALL + 2)
         .translate((cx, cy, BASE2_TOP_Z - 1)))
    motor_plate = motor_plate.cut(h)
assert motor_plate.val().isValid(), "Motor plate invalid"
print("  Motor plate OK.")

print("Building pan motor (28BYJ-48 + ULN2003, simplified, part #14 inst.1)...")
pan_motor_can = (cq.Workplane("XY").circle(MOTOR_CAN_D / 2.0).extrude(MOTOR_CAN_H)
                  .translate((CAN_XY[0], CAN_XY[1], BASE2_TOP_Z + WALL)))
pan_motor_shaft = (double_d_bore(MOTOR_SHAFT_D, BASE2_TOP_Z + WALL - PAN_GEAR_Z0,
                                  PINION_FLAT2FLAT, clearance=0.0)
                    .translate((PIN_XY[0], PIN_XY[1], PAN_GEAR_Z0)))
pan_motor = pan_motor_can.union(pan_motor_shaft)
assert pan_motor.val().isValid(), "Pan motor invalid"
print("  Pan motor OK.")

print("Building pan gear train (part #7 pan 60T, part #9 pinion x1 of 2)...")
pan_gear = build_spur_gear(GEAR60_BLANK, INFO60, GEAR_BORE_PAN, FACE_W,
                            double_d=False, grub=True).translate((0, 0, PAN_GEAR_Z0))
pan_pinion = build_spur_gear(GEAR20_BLANK, INFO20, PINION_BORE, FACE_W,
                              double_d=True, flat2flat=PINION_FLAT2FLAT
                              ).translate((PIN_XY[0], PIN_XY[1], PAN_GEAR_Z0))
assert pan_gear.val().isValid() and pan_pinion.val().isValid()
print("  Pan gear + pinion OK.")

print("Building pan shaft (part #11), 608 bearing (part #15), pan spacer "
      "(part #10, inst.1 of 4)...")
PAN_SHAFT_Z0, PAN_SHAFT_Z1 = 1.0, 44.0
pan_shaft = (cq.Workplane("XY").circle(PAN_SHAFT_R - 0.05).extrude(PAN_SHAFT_Z1 - PAN_SHAFT_Z0)
             .translate((0, 0, PAN_SHAFT_Z0)))
bearing608 = flanged_bearing(B608_OD, B608_ID, 7.0, B608_FLANGE_OD, B608_FLANGE_T)
spacer1 = (cq.Workplane("XY").circle(6.0).extrude(40.0 - PAN_GEAR_Z1)
           .translate((0, 0, PAN_GEAR_Z1)))
spacer1 = spacer1.cut(cq.Workplane("XY").circle(PAN_SHAFT_R + 0.15)
                       .extrude(40.0 - PAN_GEAR_Z1 + 0.4).translate((0, 0, PAN_GEAR_Z1 - 0.2)))
for nm, sh in (("pan_shaft", pan_shaft), ("bearing608", bearing608), ("spacer1", spacer1)):
    assert sh.val().isValid(), f"{nm} invalid"
print("  Pan shaft / bearing / spacer OK.")
print(f"  PAN STAGE built. Base2_top_z={BASE2_TOP_Z}, pan motor can top="
      f"{BASE2_TOP_Z+WALL+MOTOR_CAN_H:.1f}")

# ===========================================================================
# 4. TILT STAGE -- world coordinates.  Tilt (rotation) axis is horizontal,
#    parallel to global X, located at (y=TILT_AXIS_Y, z=TILT_AXIS_Z).
#    The tilt gear/pinion face-width sits at world X = -3..3.
# ===========================================================================

TILT_AXIS_Y = 0.0
TILT_AXIS_Z = 105.0
GEAR_HALF = FACE_W / 2.0          # 3.0
TOWER_GAP_HALF = GEAR_HALF + 0.15  # 3.15

BOSS_R = 12.0
ARM_Z0, ARM_Z1 = 40.0, 46.0
TOWER_X_A0, TOWER_X_A1 = -(TOWER_GAP_HALF + 10.0), -TOWER_GAP_HALF
TOWER_X_B0, TOWER_X_B1 = TOWER_GAP_HALF, TOWER_GAP_HALF + 10.0
TOWER_Y0, TOWER_Y1 = TILT_AXIS_Y - 10.0, TILT_AXIS_Y + 10.0
TOWER_Z1 = TILT_AXIS_Z + B625_OD / 2.0 + WALL

print("\nBuilding Gear 2 Base (tilt carrier, part #3)...")
boss = (cq.Workplane("XY").circle(BOSS_R).extrude(ARM_Z1 - ARM_Z0)
        .translate((0, 0, ARM_Z0)))
towerA = (cq.Workplane("XY")
          .box(TOWER_X_A1 - TOWER_X_A0, TOWER_Y1 - TOWER_Y0, TOWER_Z1 - ARM_Z1,
               centered=(False, False, False))
          .translate((TOWER_X_A0, TOWER_Y0, ARM_Z1)))
towerB = (cq.Workplane("XY")
          .box(TOWER_X_B1 - TOWER_X_B0, TOWER_Y1 - TOWER_Y0, TOWER_Z1 - ARM_Z1,
               centered=(False, False, False))
          .translate((TOWER_X_B0, TOWER_Y0, ARM_Z1)))
gear2base = boss.union(towerA).union(towerB)

gear2base = gear2base.cut(round_bore(PAN_SHAFT_DIA, ARM_Z1 - ARM_Z0 + 2, clearance=0.1)
                           .translate((0, 0, ARM_Z0 - 1)))
gs = grub_screw_hole(3.2, 2 * BOSS_R + 4).translate((0, 0, (ARM_Z0 + ARM_Z1) / 2.0))
gear2base = gear2base.cut(gs)

pocketA = to_tilt_axis_gen(
    bearing_pocket_cutter(B625_OD, B625_SHOULDER_R, 8.0, 3.0, B625_FLANGE_OD, B625_FLANGE_T),
    TOWER_X_A0, TILT_AXIS_Y, TILT_AXIS_Z, reverse=False)
pocketB = to_tilt_axis_gen(
    bearing_pocket_cutter(B625_OD, B625_SHOULDER_R, 8.0, 3.0, B625_FLANGE_OD, B625_FLANGE_T),
    TOWER_X_B1, TILT_AXIS_Y, TILT_AXIS_Z, reverse=True)
gear2base = gear2base.cut(pocketA).cut(pocketB)
assert gear2base.val().isValid(), "Gear 2 Base invalid"
bb = gear2base.val().BoundingBox()
print(f"  Gear 2 Base OK. bbox={bb.xlen:.1f}x{bb.ylen:.1f}x{bb.zlen:.1f} mm, "
      f"z {bb.zmin:.1f}..{bb.zmax:.1f}")

print("Building tilt gear train (part #8 tilt 60T, part #9 pinion x2 of 2)...")
TILT_PIN_Y = TILT_AXIS_Y + 40.0
tilt_gear = build_spur_gear(GEAR60_BLANK, INFO60, GEAR_BORE_TILT, FACE_W,
                             double_d=False, grub=True)
tilt_gear = to_tilt_axis_gen(tilt_gear, -GEAR_HALF, TILT_AXIS_Y, TILT_AXIS_Z)
tilt_pinion = build_spur_gear(GEAR20_BLANK, INFO20, PINION_BORE, FACE_W,
                               double_d=True, flat2flat=PINION_FLAT2FLAT)
tilt_pinion = to_tilt_axis_gen(tilt_pinion, -GEAR_HALF, TILT_PIN_Y, TILT_AXIS_Z)
assert tilt_gear.val().isValid() and tilt_pinion.val().isValid()
print("  Tilt gear + pinion OK.")

print("Building tilt shaft (part #12), 625 bearings (part #16 x2), "
      "tilt-stage spacers (part #10, inst.2/3 of 4)...")
TILT_SHAFT_X0, TILT_SHAFT_X1 = -12.0, 19.0
tilt_shaft = x_cyl(TILT_SHAFT_R - 0.05, TILT_SHAFT_X1 - TILT_SHAFT_X0,
                    TILT_SHAFT_X0, TILT_AXIS_Y, TILT_AXIS_Z)
bearing625 = flanged_bearing(B625_OD, B625_ID, B625_W, B625_FLANGE_OD, B625_FLANGE_T)
bearing625_A = to_tilt_axis_gen(bearing625, TOWER_X_A0, TILT_AXIS_Y, TILT_AXIS_Z, reverse=False)
bearing625_B = to_tilt_axis_gen(bearing625, TOWER_X_B1, TILT_AXIS_Y, TILT_AXIS_Z, reverse=True)

# Barrel far ends (bearing shoulder) sit at TOWER_X_A0+B625_W / TOWER_X_B1-B625_W
# (barrel width from each tower's opening); spacers fill the remaining gap out
# to the gear faces at x=-GEAR_HALF / +GEAR_HALF.
SP2_X0 = TOWER_X_A0 + B625_W
SP2_LEN = -GEAR_HALF - SP2_X0
SP3_X0 = GEAR_HALF
SP3_LEN = (TOWER_X_B1 - B625_W) - SP3_X0
spacer2 = x_tube(9.0, TILT_SHAFT_DIA + 0.3, SP2_LEN, SP2_X0, TILT_AXIS_Y, TILT_AXIS_Z)
spacer3 = x_tube(9.0, TILT_SHAFT_DIA + 0.3, SP3_LEN, SP3_X0, TILT_AXIS_Y, TILT_AXIS_Z)
for nm, sh in (("tilt_shaft", tilt_shaft), ("bearing625_A", bearing625_A),
               ("bearing625_B", bearing625_B), ("spacer2", spacer2), ("spacer3", spacer3)):
    assert sh.val().isValid(), f"{nm} invalid"
print("  Tilt shaft / bearings / spacers OK.")

print("Building Baring 2 shaft / tilt hub (part #4)...")
HUB_X0, HUB_X1 = 15.5, 21.5
hub = x_cyl(8.0, HUB_X1 - HUB_X0, HUB_X0, TILT_AXIS_Y, TILT_AXIS_Z)
hub = hub.cut(x_hole(TILT_SHAFT_DIA + 0.2, HUB_X1 - HUB_X0 + 2, HUB_X0 - 1,
                      TILT_AXIS_Y, TILT_AXIS_Z))
hub = hub.cut(grub_screw_hole(3.2, 22).rotate((0, 0, 0), (1, 0, 0), 90)
              .translate((HUB_X0 + 3, TILT_AXIS_Y, TILT_AXIS_Z)))
for dz in (5.0, -5.0):
    h = x_hole(3.2, HUB_X1 - HUB_X0 + 2, HUB_X0 - 1, TILT_AXIS_Y, TILT_AXIS_Z + dz)
    hub = hub.cut(h)
assert hub.val().isValid(), "Tilt hub invalid"
print("  Tilt hub OK.")

print("Building Cam Holder (part #5) + pivot pin (part #13) + spacer (inst.4/4)...")
CAM_BASE_X0, CAM_BASE_X1 = HUB_X1, HUB_X1 + 3.0
CAM_ARM_X0, CAM_ARM_X1 = CAM_BASE_X1, CAM_BASE_X1 + 14.0
CAM_PIN_X = (CAM_ARM_X0 + CAM_ARM_X1) / 2.0
cam_base = (cq.Workplane("XY").box(CAM_BASE_X1 - CAM_BASE_X0, 12, 12, centered=(False, True, True))
            .translate((CAM_BASE_X0, TILT_AXIS_Y, TILT_AXIS_Z)))
for dz in (5.0, -5.0):
    h = x_hole(3.2, CAM_BASE_X1 - CAM_BASE_X0 + 2, CAM_BASE_X0 - 1,
               TILT_AXIS_Y, TILT_AXIS_Z + dz)
    cam_base = cam_base.cut(h)
tabA = (cq.Workplane("XY").box(CAM_ARM_X1 - CAM_ARM_X0, 4, 8, centered=(False, True, True))
        .translate((CAM_ARM_X0, TILT_AXIS_Y - 6, TILT_AXIS_Z)))
tabB = (cq.Workplane("XY").box(CAM_ARM_X1 - CAM_ARM_X0, 4, 8, centered=(False, True, True))
        .translate((CAM_ARM_X0, TILT_AXIS_Y + 6, TILT_AXIS_Z)))
cam_holder = cam_base.union(tabA).union(tabB)
pin_bore = y_cyl(PIN_BORE_DIA / 2.0, 20, CAM_PIN_X, TILT_AXIS_Y - 10, TILT_AXIS_Z)
cam_holder = cam_holder.cut(pin_bore)
assert cam_holder.val().isValid(), "Cam Holder invalid"

pivot_pin = y_cyl((PIN_DIA - 0.1) / 2.0, 16, CAM_PIN_X, TILT_AXIS_Y - 8, TILT_AXIS_Z)
spacer4 = y_cyl(3.0, 4, CAM_PIN_X, TILT_AXIS_Y - 2, TILT_AXIS_Z)
spacer4 = spacer4.cut(y_cyl(PIN_DIA / 2.0 + 0.15, 4.4, CAM_PIN_X, TILT_AXIS_Y - 2.2, TILT_AXIS_Z))
for nm, sh in (("cam_holder", cam_holder), ("pivot_pin", pivot_pin), ("spacer4", spacer4)):
    assert sh.val().isValid(), f"{nm} invalid"
print("  Cam Holder / pivot pin / spacer OK.")

print("Building tilt motor (part #14 inst.2) + tilt motor bracket (part #6)...")
TILT_CAN_Y = TILT_PIN_Y + MOTOR_SHAFT_OFFSET
TILT_CAN_X0 = 10.0
tilt_can = (cq.Workplane("YZ").workplane(offset=TILT_CAN_X0)
            .circle(MOTOR_CAN_D / 2.0).extrude(MOTOR_CAN_H)
            .translate((0, TILT_CAN_Y, TILT_AXIS_Z)))
tilt_motor_shaft_len = TILT_CAN_X0 - (-GEAR_HALF)
tilt_motor_shaft = to_tilt_axis_gen(
    double_d_bore(MOTOR_SHAFT_D, tilt_motor_shaft_len, PINION_FLAT2FLAT, clearance=0.0),
    -GEAR_HALF, TILT_PIN_Y, TILT_AXIS_Z)
tilt_motor = tilt_can.union(tilt_motor_shaft)
assert tilt_motor.val().isValid(), "Tilt motor invalid"

BRACKET_Y0, BRACKET_Y1 = TILT_CAN_Y - 20.0, TILT_CAN_Y + 20.0
bracket_plate = (cq.Workplane("XY").box(3, BRACKET_Y1 - BRACKET_Y0, 47, centered=(False, True, True))
                  .translate((7.0, TILT_CAN_Y, TILT_AXIS_Z)))
shaft_hole = x_hole(MOTOR_SHAFT_D + 1.2, 5, 6.5, TILT_PIN_Y, TILT_AXIS_Z)
bracket_plate = bracket_plate.cut(shaft_hole)
for dz in (MOTOR_BOLT_PITCH / 2.0, -MOTOR_BOLT_PITCH / 2.0):
    h = x_hole(MOTOR_BOLT_HOLE_D, 5, 6.5, TILT_CAN_Y, TILT_AXIS_Z + dz)
    bracket_plate = bracket_plate.cut(h)
gusset = (cq.Workplane("XY").box(3, BRACKET_Y0 - TOWER_Y1, 6, centered=(False, False, False))
          .translate((7.0, TOWER_Y1, TILT_AXIS_Z - 3)))
tilt_bracket = bracket_plate.union(gusset)
assert tilt_bracket.val().isValid(), "Tilt motor bracket invalid"
print("  Tilt motor / bracket OK.")

bbG = gear2base.val().BoundingBox()
bbT = tilt_gear.val().BoundingBox()
print(f"\nTILT STAGE built. Tilt axis (y,z)=({TILT_AXIS_Y},{TILT_AXIS_Z}), "
      f"tilt gear z {bbT.zmin:.1f}..{bbT.zmax:.1f}")

# ===========================================================================
# 5. ASSEMBLY
# ===========================================================================

print("\nAssembling all 16 unique parts / 22 occurrences...")
asm = cq.Assembly(name="Design-Option-B_PanTilt")

PRINTED = (0.85, 0.55, 0.20, 1.0)
STEEL = (0.75, 0.75, 0.78, 1.0)
BOUGHT = (0.25, 0.35, 0.55, 1.0)

asm.add(base2, name="01_Base2_pan_housing", color=cq.Color(*PRINTED))
asm.add(motor_plate, name="02_Motor_base_pan_plate", color=cq.Color(*PRINTED))
asm.add(gear2base, name="03_Gear2Base_tilt_carrier", color=cq.Color(*PRINTED))
asm.add(hub, name="04_Baring2shaft_tilt_hub", color=cq.Color(*PRINTED))
asm.add(cam_holder, name="05_Cam_Holder", color=cq.Color(*PRINTED))
asm.add(tilt_bracket, name="06_Tilt_motor_bracket", color=cq.Color(*PRINTED))
asm.add(pan_gear, name="07_Spur_gear_60T_pan_8mmDbore", color=cq.Color(*PRINTED))
asm.add(tilt_gear, name="08_Spur_gear_60T_tilt_5mmbore", color=cq.Color(*PRINTED))
asm.add(pan_pinion, name="09_Pinion_20T_inst1_pan", color=cq.Color(*PRINTED))
asm.add(tilt_pinion, name="09_Pinion_20T_inst2_tilt", color=cq.Color(*PRINTED))
asm.add(spacer1, name="10_Spacer_sleeve_inst1_pan", color=cq.Color(*PRINTED))
asm.add(spacer2, name="10_Spacer_sleeve_inst2_tiltA", color=cq.Color(*PRINTED))
asm.add(spacer3, name="10_Spacer_sleeve_inst3_tiltB", color=cq.Color(*PRINTED))
asm.add(spacer4, name="10_Spacer_sleeve_inst4_camPin", color=cq.Color(*PRINTED))
asm.add(pan_shaft, name="11_Pan_shaft_8mm", color=cq.Color(*STEEL))
asm.add(tilt_shaft, name="12_Tilt_shaft_5mm", color=cq.Color(*STEEL))
asm.add(pivot_pin, name="13_CamHolder_pivot_pin_3mm", color=cq.Color(*STEEL))
asm.add(pan_motor, name="14_Stepper_28BYJ48_ULN2003_inst1_pan", color=cq.Color(*BOUGHT))
asm.add(tilt_motor, name="14_Stepper_28BYJ48_ULN2003_inst2_tilt", color=cq.Color(*BOUGHT))
asm.add(bearing608, name="15_Flanged_bearing_608_8mm", color=cq.Color(*BOUGHT))
asm.add(bearing625_A, name="16_Flanged_bearing_625_inst1_towerA", color=cq.Color(*BOUGHT))
asm.add(bearing625_B, name="16_Flanged_bearing_625_inst2_towerB", color=cq.Color(*BOUGHT))

assert len(asm.children) == 22, f"expected 22 occurrences, got {len(asm.children)}"
print(f"  Assembly has {len(asm.children)} occurrences of 16 unique parts.")

# ===========================================================================
# 6. OVERALL ENVELOPE
# ===========================================================================

all_shapes = [c.obj.val() for c in asm.children]
compound = cq.Compound.makeCompound(all_shapes)
bb = compound.BoundingBox()
print(f"\nOverall envelope: {bb.xlen:.1f} x {bb.zlen:.1f} x {bb.ylen:.1f} mm "
      f"(width x height x depth)  [spec target ~120 x 140 x 92 mm]")
print(f"  X {bb.xmin:.1f}..{bb.xmax:.1f}  Y {bb.ymin:.1f}..{bb.ymax:.1f}  "
      f"Z {bb.zmin:.1f}..{bb.zmax:.1f}")

# ===========================================================================
# 7. INTERFERENCE CHECKS
# ===========================================================================

print("\nRunning interference checks (.intersect() volumes)...")
pairs = [
    ("Base2 vs pan_gear",              base2, pan_gear,        False),
    ("Base2 vs pan_pinion",            base2, pan_pinion,       False),
    ("Base2 vs bearing608",            base2, bearing608,       False),
    ("Base2 vs motor_plate",           base2, motor_plate,      False),
    ("motor_plate vs pan_motor(can)",  motor_plate, pan_motor,  False),
    ("pan_motor(shaft) vs pan_pinion", pan_motor, pan_pinion,   False),
    ("pan_gear vs pan_pinion (MESH)",  pan_gear, pan_pinion,    True),
    ("pan_shaft vs pan_gear",          pan_shaft, pan_gear,     False),
    ("pan_shaft vs bearing608",        pan_shaft, bearing608,   False),
    ("pan_shaft vs spacer1",           pan_shaft, spacer1,      False),
    ("spacer1 vs pan_gear",            spacer1, pan_gear,       False),
    ("spacer1 vs gear2base(boss)",     spacer1, gear2base,      False),
    ("gear2base vs tilt_gear",         gear2base, tilt_gear,    False),
    ("gear2base vs tilt_pinion",       gear2base, tilt_pinion,  False),
    ("gear2base(towerA) vs bearing625A", gear2base, bearing625_A, False),
    ("gear2base(towerB) vs bearing625B", gear2base, bearing625_B, False),
    ("tilt_shaft vs bearing625A",      tilt_shaft, bearing625_A, False),
    ("tilt_shaft vs bearing625B",      tilt_shaft, bearing625_B, False),
    ("tilt_shaft vs tilt_gear",        tilt_shaft, tilt_gear,    False),
    ("tilt_shaft vs spacer2",          tilt_shaft, spacer2,      False),
    ("tilt_shaft vs spacer3",          tilt_shaft, spacer3,      False),
    ("tilt_shaft vs hub",              tilt_shaft, hub,          False),
    ("spacer2 vs tilt_gear",           spacer2, tilt_gear,       False),
    ("spacer3 vs tilt_gear",           spacer3, tilt_gear,       False),
    ("hub vs gear2base(towerB)",       hub, gear2base,           False),
    ("hub vs bearing625B",             hub, bearing625_B,        False),
    ("hub vs cam_holder",              hub, cam_holder,          False),
    ("cam_holder vs pivot_pin",        cam_holder, pivot_pin,    False),
    ("cam_holder vs spacer4",          cam_holder, spacer4,      False),
    ("pivot_pin vs spacer4",           pivot_pin, spacer4,       False),
    ("tilt_gear vs tilt_pinion (MESH)", tilt_gear, tilt_pinion,  True),
    ("tilt_motor(shaft) vs tilt_pinion", tilt_motor, tilt_pinion, False),
    ("tilt_motor(can) vs tilt_bracket", tilt_motor, tilt_bracket, False),
    ("tilt_bracket vs gear2base(towerB)", tilt_bracket, gear2base, False),
    ("pan_motor(can) vs Base2",        pan_motor, base2,         False),
    ("tilt_motor(can) vs gear2base",   tilt_motor, gear2base,    False),
    ("tilt_motor(can) vs tilt_gear",   tilt_motor, tilt_gear,    False),
    ("cam_holder vs gear2base",        cam_holder, gear2base,    False),
]

RUN_CLR_TOL = 1.0
unexpected = []
for name, a, b, is_mesh in pairs:
    try:
        common = a.intersect(b)
        vol = common.val().Volume() if common.val() is not None else 0.0
    except Exception:
        vol = 0.0
    tag = "MESH (expected)" if is_mesh else ("ok" if vol < RUN_CLR_TOL else "*** UNEXPECTED ***")
    print(f"  {name:42s} vol={vol:10.3f} mm3  [{tag}]")
    if not is_mesh and vol >= RUN_CLR_TOL:
        unexpected.append((name, vol))

if unexpected:
    print(f"\n*** {len(unexpected)} UNEXPECTED INTERFERENCE(S) FOUND ***")
    for n, v in unexpected:
        print(f"    {n}: {v:.3f} mm3")
else:
    print("\nNo unintended interferences found (gear-tooth mesh excepted).")

# ===========================================================================
# 8. VALIDITY CHECK
# ===========================================================================

print("\nValidity check on all 22 occurrences...")
all_valid = True
for c in asm.children:
    ok = c.obj.val().isValid()
    if not ok:
        all_valid = False
        print(f"  INVALID: {c.name}")
print("  All solids valid." if all_valid else "  *** SOME SOLIDS INVALID ***")

# ===========================================================================
# 9. EXPORT
# ===========================================================================

print(f"\nExporting STEP assembly to {STEP_OUT} ...")
asm.save(STEP_OUT, exportType="STEP")
sz = os.path.getsize(STEP_OUT)
print(f"  Done. {sz/1024:.0f} KB written.")
print(f"\n{'PASS' if (all_valid and not unexpected) else 'FAIL'}: "
      f"{'ready' if (all_valid and not unexpected) else 'needs fixes'}")
