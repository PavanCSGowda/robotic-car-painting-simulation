# ==============================================================================
# Final 4-Stage Full-Body Paint Cell (Direct Base Targets + 1880mm MoveL Sweet Spot)
# Candidate: Pavan C S | Assessment: Yuktii AI Labs | Robot: ABB IRB 5500-27
# ==============================================================================
from robolink import *
import math

RDK = Robolink()
RDK.Render(False)

# 1. Connect to ABB Robot and Attached Paint Sprayer Tool
robot = RDK.Item('', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("ABB IRB 5500-27 robot not found in station!")

tool = None
for item in robot.Childs():
    if item.Type() == ITEM_TYPE_TOOL:
        tool = item
        break

if tool is None or not tool.Valid():
    raise Exception("Please ensure Generic Paint Sprayer tool is attached!")

# Clean up previous programs, frames, and targets
cleanup_names = [
    'Paint_Full_Cycle', 'Prog_1_Driver_0deg', 'Prog_2_Front_90deg',
    'Prog_3_Passenger_180deg', 'Prog_4_Rear_270deg',
    'Frame_S1_Driver', 'Frame_S2_Front', 'Frame_S3_Passenger', 'Frame_S4_Rear',
    'Car_Frame', 'Booth_Frame', 'Rail_Frame'
]
for name in cleanup_names:
    old = RDK.Item(name)
    if old.Valid():
        old.Delete()

for t in RDK.ItemList(ITEM_TYPE_TARGET):
    t.Delete()

MatClass = robot.Pose().__class__
pi = math.pi

def transl(x, y, z):
    return MatClass([
        [1.0, 0.0, 0.0, float(x)],
        [0.0, 1.0, 0.0, float(y)],
        [0.0, 0.0, 1.0, float(z)],
        [0.0, 0.0, 0.0, 1.0]
    ])

# ==============================================================================
# 2. Exact 25-Point SolveFK Grid (Identical to Working image_f870c6.jpg!)
# ==============================================================================
robot_base = robot.Parent()
robot.setPoseFrame(robot_base)
robot.setPoseTool(tool)
tool_pose = tool.PoseTool()

J2_GRID = [-55.0, -30.0, -5.0, 20.0, 45.0]
J3_GRID = [-42.0, -22.0, -2.0, 18.0, 38.0]

fk_samples = []
for j2_val in J2_GRID:
    for j3_val in J3_GRID:
        j5_val = -(j2_val + j3_val) + 5.0
        tcp = robot.SolveFK([0.0, j2_val, j3_val, 0.0, j5_val, 0.0]) * tool_pose
        fk_samples.append((j2_val, j3_val, tcp[0, 3], tcp[1, 3], tcp[2, 3]))

tcp_j1_test = robot.SolveFK([10.0, -5.0, -2.0, 0.0, 12.0, 0.0]) * tool_pose
tcp_j1_zero = robot.SolveFK([ 0.0, -5.0, -2.0, 0.0, 12.0, 0.0]) * tool_pose
J1_SIGN = 1.0 if (tcp_j1_test[0, 3] < tcp_j1_zero[0, 3]) else -1.0

def solve_exact_from_grid(x_goal, y_goal, z_goal, nozzle_down_deg):
    dpitch = math.radians(nozzle_down_deg - 5.0)
    r_target = math.hypot(x_goal, y_goal) + 160.0 * (1.0 - math.cos(dpitch))
    z_target = z_goal + 210.0 * math.sin(dpitch)

    best_dist = 1e9
    j2_est, j3_est = -5.0, -2.0
    x0_est, y0_est = 174.26, 1880.0
    for (g_j2, g_j3, gx0, gy0, gz0) in fk_samples:
        gr = math.hypot(gx0, gy0)
        d = (gr - r_target)**2 + (gz0 - z_target)**2
        if d < best_dist:
            best_dist = d
            j2_est, j3_est = g_j2, g_j3
            x0_est, y0_est = gx0, gy0

    idx2 = min(len(J2_GRID) - 2, max(0, J2_GRID.index(j2_est)))
    idx3 = min(len(J3_GRID) - 2, max(0, J3_GRID.index(j3_est)))

    def get_cell(i2, i3):
        return fk_samples[i2 * len(J3_GRID) + i3]

    for _ in range(12):
        for k in range(len(J2_GRID) - 1):
            if J2_GRID[k] <= j2_est <= J2_GRID[k+1]:
                idx2 = k
                break
        for k in range(len(J3_GRID) - 1):
            if J3_GRID[k] <= j3_est <= J3_GRID[k+1]:
                idx3 = k
                break

        j2_0, j3_0, x00, y00, z00 = get_cell(idx2, idx3)
        j2_1, _,    x10, y10, z10 = get_cell(idx2 + 1, idx3)
        _,    j3_1, x01, y01, z01 = get_cell(idx2, idx3 + 1)
        _,    _,    x11, y11, z11 = get_cell(idx2 + 1, idx3 + 1)

        r00, r10 = math.hypot(x00, y00), math.hypot(x10, y10)
        r01, r11 = math.hypot(x01, y01), math.hypot(x11, y11)

        u = (j2_est - j2_0) / (j2_1 - j2_0)
        v = (j3_est - j3_0) / (j3_1 - j3_0)

        r_curr = (1-u)*(1-v)*r00 + u*(1-v)*r10 + (1-u)*v*r01 + u*v*r11
        z_curr = (1-u)*(1-v)*z00 + u*(1-v)*z10 + (1-u)*v*z01 + u*v*z11
        x0_est = (1-u)*(1-v)*x00 + u*(1-v)*x10 + (1-u)*v*x01 + u*v*x11
        y0_est = (1-u)*(1-v)*y00 + u*(1-v)*y10 + (1-u)*v*y01 + u*v*y11

        er = r_target - r_curr
        ez = z_target - z_curr
        if abs(er) < 0.3 and abs(ez) < 0.3:
            break

        dr_du = ((1-v)*(r10 - r00) + v*(r11 - r01)) / (j2_1 - j2_0)
        dz_du = ((1-v)*(z10 - z00) + v*(z11 - z01)) / (j2_1 - j2_0)
        dr_dv = ((1-u)*(r01 - r00) + u*(r11 - r10)) / (j3_1 - j3_0)
        dz_dv = ((1-u)*(z01 - z00) + u*(z11 - z10)) / (j3_1 - j3_0)

        det = dr_du * dz_dv - dr_dv * dz_du
        if abs(det) > 1e-3:
            j2_est += 0.8 * (er * dz_dv - dr_dv * ez) / det
            j3_est += 0.8 * (dr_du * ez - er * dz_du) / det

        j2_est = max(-55.0, min(45.0, j2_est))
        j3_est = max(-42.0, min(38.0, j3_est))

    ang_zero = math.degrees(math.atan2(x0_est, y0_est))
    ang_goal = math.degrees(math.atan2(x_goal, y_goal))
    j1 = J1_SIGN * (ang_zero - ang_goal)

    j1 = max(-85.0, min(85.0, j1))
    j5 = max(-65.0, min(65.0, -(j2_est + j3_est) + nozzle_down_deg))
    return [round(j1, 2), round(j2_est, 2), round(j3_est, 2), 0.0, round(j5, 2), 0.0]

# ==============================================================================
# 3. Geometry Setup: Locked Sweet-Spot Distances
# ==============================================================================
DOOR_TCP_Y   = 1880.0
CAR_SIDE_Y   = 3010.0  # 0° & 180° Side Poses: Car side surface at Y=2060, TCP at Y=1880
CAR_END_Y    = 4300.0  # 90° & 270° End Poses: Bumper at 4300-2400 = 1900, Bonnet/Dikki at 1900..2500!

def quad_to_tris(p0, p1, p2, p3):
    return [p0, p1, p2, p0, p2, p3]

def get_car_half_width(z, w_bottom=1900.0, w_top=1520.0, z_cabin_split=920.0):
    if z <= z_cabin_split:
        return w_bottom / 2.0
    factor = (z - z_cabin_split) / (1600.0 - z_cabin_split)
    return (w_bottom / 2.0) * (1.0 - factor) + (w_top / 2.0) * factor

def make_car_along_x(xz_loop, w_bottom, w_top, z_cabin_split=920):
    tris = []
    n = len(xz_loop)
    for i in range(n):
        x1, z1 = xz_loop[i]
        x2, z2 = xz_loop[(i + 1) % n]
        hw1 = get_car_half_width(z1, w_bottom, w_top, z_cabin_split)
        hw2 = get_car_half_width(z2, w_bottom, w_top, z_cabin_split)
        p_f1, p_b1 = [x1, -hw1, z1], [x1, hw1, z1]
        p_f2, p_b2 = [x2, -hw2, z2], [x2, hw2, z2]
        tris.extend(quad_to_tris(p_f1, p_b1, p_b2, p_f2))

    xc, zc = 0.0, 600.0
    hw_c = w_bottom / 2.0
    for i in range(n):
        x1, z1 = xz_loop[i]
        x2, z2 = xz_loop[(i + 1) % n]
        hw1 = get_car_half_width(z1, w_bottom, w_top, z_cabin_split)
        hw2 = get_car_half_width(z2, w_bottom, w_top, z_cabin_split)
        tris.extend([[xc, -hw_c, zc], [x2, -hw2, z2], [x1, -hw1, z1]])
        tris.extend([[xc,  hw_c, zc], [x1,  hw1, z1], [x2,  hw2, z2]])
    return tris

def make_wheel_along_x(x_center, y_in, y_out, z_center, radius=350, segments=24):
    tris = []
    for i in range(segments):
        a1 = 2.0 * pi * i / segments
        a2 = 2.0 * pi * (i + 1) / segments
        x1, z1 = x_center + radius * math.cos(a1), z_center + radius * math.sin(a1)
        x2, z2 = x_center + radius * math.cos(a2), z_center + radius * math.sin(a2)
        tris.extend(quad_to_tris([x1, y_in, z1], [x1, y_out, z1], [x2, y_out, z2], [x2, y_in, z2]))
        tris.extend([[x_center, y_in, z_center], [x2, y_in, z2], [x1, y_in, z1]])
        tris.extend([[x_center, y_out, z_center], [x1, y_out, z1], [x2, y_out, z2]])
    return tris

booth_frame = RDK.AddFrame('Booth_Frame')
booth_frame.setPose(transl(0, 1800, 0))
floor_tris = quad_to_tris([-6000, -4000, 0], [6000, -4000, 0], [6000, 4000, 0], [-6000, 4000, 0])
floor_obj = RDK.AddShape(floor_tris)
floor_obj.setName('Paint_Booth_Floor_12x8m')
floor_obj.setParent(booth_frame)
floor_obj.setColor([0.22, 0.24, 0.27, 0.4])

rail_frame = RDK.AddFrame('Rail_Frame')
rail_frame.setPose(transl(0, 0, 0))
rail_tris = quad_to_tris([-3500, -450, 5], [3500, -450, 5], [3500, 450, 5], [-3500, 450, 5])
rail_obj = RDK.AddShape(rail_tris)
rail_obj.setName('Linear_Track_Bed_7m')
rail_obj.setParent(rail_frame)
rail_obj.setColor([0.75, 0.60, 0.15, 1.0])

car_frame = RDK.AddFrame('Car_Frame')
car_frame.setPose(transl(0, CAR_SIDE_Y, 0))

skid_tris = quad_to_tris([-2500, -1100, 8], [2500, -1100, 8], [2500, 1100, 8], [-2500, 1100, 8])
skid_obj = RDK.AddShape(skid_tris)
skid_obj.setName('Rotary_Skid_Platform')
skid_obj.setParent(car_frame)
skid_obj.setColor([0.35, 0.38, 0.42, 1.0])

sedan_profile = [
    (-2400, 220), (-2400, 520), (-2250, 680), (-1250, 920),
    (-550, 1560), ( 350, 1600), ( 950, 1540), ( 1650, 980),
    ( 2320, 900), ( 2400, 750), ( 2400, 240), ( 1200, 200), (-1200, 200)
]

body_tris = make_car_along_x(sedan_profile, w_bottom=1900.0, w_top=1520.0, z_cabin_split=920.0)
car_body = RDK.AddShape(body_tris)
car_body.setName('Sedan_Body_4800x1900x1600')
car_body.setParent(car_frame)
car_body.setColor([0.18, 0.45, 0.78, 1.0])

glass_tris = []
glass_tris.extend(quad_to_tris([-1230, -760, 930], [-1230, 760, 930], [-560, 680, 1550], [-560, -680, 1550]))
glass_tris.extend(quad_to_tris([ 960, -680, 1530], [ 960, 680, 1530], [ 1630, 760, 990], [ 1630, -760, 990]))

z_win_bot, z_win_top = 980.0, 1450.0
hw_bot = get_car_half_width(z_win_bot) + 3.0
hw_top = get_car_half_width(z_win_top) + 3.0

for side_sign in [-1.0, 1.0]:
    y_wb = side_sign * hw_bot
    y_wt = side_sign * hw_top
    glass_tris.extend(quad_to_tris([-1040.0, y_wb, z_win_bot], [30.0, y_wb, z_win_bot], [30.0, y_wt, z_win_top], [-520.0, y_wt, z_win_top]))
    glass_tris.extend(quad_to_tris([ 210.0, y_wb, z_win_bot], [1380.0, y_wb, z_win_bot], [920.0, y_wt, z_win_top], [ 210.0, y_wt, z_win_top]))

glass_obj = RDK.AddShape(glass_tris)
glass_obj.setName('Sedan_Windshields_Glass')
glass_obj.setParent(car_frame)
glass_obj.setColor([0.12, 0.15, 0.18, 1.0])

wheels_tris = []
for x_axle in [-1450.0, 1450.0]:
    wheels_tris.extend(make_wheel_along_x(x_axle, -960.0, -720.0, 350.0, radius=350.0))
    wheels_tris.extend(make_wheel_along_x(x_axle,  720.0,  960.0, 350.0, radius=350.0))

wheels_obj = RDK.AddShape(wheels_tris)
wheels_obj.setName('Sedan_Wheels_4x')
wheels_obj.setParent(car_frame)
wheels_obj.setColor([0.15, 0.15, 0.15, 1.0])

# ==============================================================================
# 4. Target Generation Parented Directly to robot_base (Zero MoveL Wrist Flip!)
# ==============================================================================
def add_synced_pt(name, x, y, z, nozzle_deg):
    joints = solve_exact_from_grid(x, y, z, nozzle_deg)
    tcp_pose = robot.SolveFK(joints) * tool_pose
    t = RDK.AddTarget(name, robot_base, robot)
    t.setPose(tcp_pose)
    t.setJoints(joints)
    t.setAsJointTarget()
    return t

def add_linear_pass(prog, prefix, x_start, x_end, y_start, y_end, z_start, z_end, nozzle_deg, n_pts=3, use_movej_all=False):
    """
    Point 1 (i == 0) uses MoveJ for row entry; Points 2..N use MoveL for painting!
    Because all targets are in robot_base (just like image_f870c6.jpg), MoveL
    never flips the wrist and never shifts the yellow line away from the arrows!
    """
    for i in range(n_pts):
        f = float(i) / float(n_pts - 1) if n_pts > 1 else 0.0
        x = x_start + f * (x_end - x_start)
        y = y_start + f * (y_end - y_start)
        z = z_start + f * (z_end - z_start)
        pt = add_synced_pt(f"{prefix}_{i+1}", x, y, z, nozzle_deg)
        if use_movej_all or i == 0:
            prog.MoveJ(pt)
        else:
            prog.MoveL(pt)

t_home = add_synced_pt('Target_Home', 0.0, DOOR_TCP_Y - 220.0, 950.0, 5.0)

def build_driver_0_program(prog_name, tag):
    """
    STAGE 1 (0° Driver Side):
    """
    p = RDK.AddProgram(prog_name, robot)
    p.setSpeed(150)
    p.setSpeedJoints(14)
    p.MoveJ(t_home)

    # Zone 1: Front Nose Skirt & Upper Front Fender
    add_linear_pass(p, f'{tag}_Nose1',   -2250, -1860, DOOR_TCP_Y, DOOR_TCP_Y, 320, 320, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Nose2',   -1860, -2250, DOOR_TCP_Y, DOOR_TCP_Y, 450, 450, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Nose3',   -2250, -1860, DOOR_TCP_Y, DOOR_TCP_Y, 580, 580, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Fender1', -2100, -1020, DOOR_TCP_Y, DOOR_TCP_Y, 760, 860, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Fender2', -1020, -2100, DOOR_TCP_Y, DOOR_TCP_Y, 910, 810, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Hood1',   -2000, -1180, DOOR_TCP_Y + 160.0, DOOR_TCP_Y + 160.0, 820, 950, 26.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Hood2',   -1180, -2000, DOOR_TCP_Y + 280.0, DOOR_TCP_Y + 280.0, 970, 840, 30.0, n_pts=2)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_Door', -1000.0, DOOR_TCP_Y - 180.0, 550.0, 5.0))

    # Zone 2: 7-Pass Doors Between Wheels (X = -980 to +1050 mm)
    door_z = [300.0, 405.0, 510.0, 615.0, 720.0, 825.0, 930.0]
    for idx, zv in enumerate(door_z):
        xs, xe = (-980.0, 1050.0) if idx % 2 == 0 else (1050.0, -980.0)
        add_linear_pass(p, f'{tag}_Door{idx+1}', xs, xe, DOOR_TCP_Y, DOOR_TCP_Y, zv, zv, 5.0, n_pts=3)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_Pillar', -1050.0, DOOR_TCP_Y - 180.0, 980.0, 10.0))

    # Zone 3: Greenhouse Pillars
    add_linear_pass(p, f'{tag}_APil', -1120, -540, DOOR_TCP_Y, DOOR_TCP_Y + 145.0,  950, 1490, 22.0, n_pts=2, use_movej_all=True)
    add_linear_pass(p, f'{tag}_Cant',  -520,  920, DOOR_TCP_Y + 165.0, DOOR_TCP_Y + 165.0, 1510, 1510, 28.0, n_pts=3)
    add_linear_pass(p, f'{tag}_CPil',   920, 1520, DOOR_TCP_Y + 145.0, DOOR_TCP_Y, 1490,  980, 22.0, n_pts=2, use_movej_all=True)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_BPil', 500.0, DOOR_TCP_Y - 200.0, 1150.0, 10.0))
    add_linear_pass(p, f'{tag}_BPil',   120,  120, DOOR_TCP_Y, DOOR_TCP_Y + 145.0,  960, 1460, 16.0, n_pts=2, use_movej_all=True)

    # Zone 4: Overhead Left Half-Roof
    add_linear_pass(p, f'{tag}_Roof1', -650,  750, DOOR_TCP_Y + 360.0, DOOR_TCP_Y + 360.0, 1680, 1680, 48.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Roof2',  750, -650, DOOR_TCP_Y + 480.0, DOOR_TCP_Y + 480.0, 1680, 1680, 52.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Roof3', -650,  750, DOOR_TCP_Y + 600.0, DOOR_TCP_Y + 600.0, 1680, 1680, 56.0, n_pts=2)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_Rear', 1050.0, DOOR_TCP_Y - 180.0, 1050.0, 10.0))

    # Zone 5: Left Trunk, Upper Quarter & Rear Skirt
    add_linear_pass(p, f'{tag}_Trnk1', 1150, 2000, DOOR_TCP_Y + 280.0, DOOR_TCP_Y + 280.0, 1000, 930, 28.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Qtr1',  1050, 2100, DOOR_TCP_Y, DOOR_TCP_Y, 800, 820, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Qtr2',  2100, 1050, DOOR_TCP_Y, DOOR_TCP_Y, 890, 910, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_RSk1',  1860, 2220, DOOR_TCP_Y, DOOR_TCP_Y, 580, 580, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_RSk2',  2220, 1860, DOOR_TCP_Y, DOOR_TCP_Y, 450, 450, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_RSk3',  1860, 2220, DOOR_TCP_Y, DOOR_TCP_Y, 320, 320, 5.0, n_pts=2)

    p.MoveJ(t_home)
    return p

def build_passenger_180_program(prog_name, tag):
    """
    STAGE 3 (180° Passenger Side):
    Uses the exact same robot_base parent frame and MoveJ-entry / MoveL-stroke
    pattern as Stage 1!
    """
    p = RDK.AddProgram(prog_name, robot)
    p.setSpeed(150)
    p.setSpeedJoints(14)
    p.MoveJ(t_home)

    # Zone 1: Left Side at 180° (Rear Skirt, Quarter Panel & Right Half-Trunk)
    add_linear_pass(p, f'{tag}_RSk1',  -2220, -1860, DOOR_TCP_Y, DOOR_TCP_Y, 320, 320, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_RSk2',  -1860, -2220, DOOR_TCP_Y, DOOR_TCP_Y, 450, 450, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_RSk3',  -2220, -1860, DOOR_TCP_Y, DOOR_TCP_Y, 580, 580, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Qtr1',  -2100, -1050, DOOR_TCP_Y, DOOR_TCP_Y, 820, 800, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Qtr2',  -1050, -2100, DOOR_TCP_Y, DOOR_TCP_Y, 910, 890, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Trnk1', -2000, -1150, DOOR_TCP_Y + 280.0, DOOR_TCP_Y + 280.0, 930, 1000, 28.0, n_pts=2)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_Door', -1000.0, DOOR_TCP_Y - 180.0, 550.0, 5.0))

    # Zone 2: 7-Pass Doors Between Wheels (X = -980 to +1050 mm — 100% Wheel-Free!)
    door_z = [300.0, 405.0, 510.0, 615.0, 720.0, 825.0, 930.0]
    for idx, zv in enumerate(door_z):
        xs, xe = (-980.0, 1050.0) if idx % 2 == 0 else (1050.0, -980.0)
        add_linear_pass(p, f'{tag}_Door{idx+1}', xs, xe, DOOR_TCP_Y, DOOR_TCP_Y, zv, zv, 5.0, n_pts=3)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_Pillar', -1050.0, DOOR_TCP_Y - 180.0, 980.0, 10.0))

    # Zone 3: Greenhouse Pillars for 180° (C-Pillar on Left -> Cantrail -> A-Pillar on Right)
    add_linear_pass(p, f'{tag}_CPil', -1480, -900, DOOR_TCP_Y, DOOR_TCP_Y + 145.0,  980, 1490, 22.0, n_pts=2, use_movej_all=True)
    add_linear_pass(p, f'{tag}_Cant',  -880,  540, DOOR_TCP_Y + 165.0, DOOR_TCP_Y + 165.0, 1510, 1510, 28.0, n_pts=3)
    add_linear_pass(p, f'{tag}_APil',   560, 1100, DOOR_TCP_Y + 145.0, DOOR_TCP_Y, 1490,  950, 22.0, n_pts=2, use_movej_all=True)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_BPil', 400.0, DOOR_TCP_Y - 200.0, 1150.0, 10.0))
    add_linear_pass(p, f'{tag}_BPil',   -80,  -80, DOOR_TCP_Y, DOOR_TCP_Y + 145.0,  960, 1460, 16.0, n_pts=2, use_movej_all=True)

    # Zone 4: Overhead Right Half-Roof
    add_linear_pass(p, f'{tag}_Roof1', -650,  750, DOOR_TCP_Y + 360.0, DOOR_TCP_Y + 360.0, 1680, 1680, 48.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Roof2',  750, -650, DOOR_TCP_Y + 480.0, DOOR_TCP_Y + 480.0, 1680, 1680, 52.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Roof3', -650,  750, DOOR_TCP_Y + 600.0, DOOR_TCP_Y + 600.0, 1680, 1680, 56.0, n_pts=2)

    p.MoveJ(add_synced_pt(f'{tag}_Ret_Front', 1050.0, DOOR_TCP_Y - 180.0, 1050.0, 10.0))

    # Zone 5: Right Side at 180° (Right Half-Hood, Front Fender & Front Nose Skirt)
    add_linear_pass(p, f'{tag}_Hood1',  1180, 2000, DOOR_TCP_Y + 280.0, DOOR_TCP_Y + 280.0, 970, 840, 30.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Hood2',  2000, 1180, DOOR_TCP_Y + 160.0, DOOR_TCP_Y + 160.0, 820, 950, 26.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Fend1',  1050, 2100, DOOR_TCP_Y, DOOR_TCP_Y, 860, 760, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Fend2',  2100, 1050, DOOR_TCP_Y, DOOR_TCP_Y, 810, 910, 8.0, n_pts=3)
    add_linear_pass(p, f'{tag}_Nose1',  1860, 2250, DOOR_TCP_Y, DOOR_TCP_Y, 580, 580, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Nose2',  2250, 1860, DOOR_TCP_Y, DOOR_TCP_Y, 450, 450, 5.0, n_pts=2)
    add_linear_pass(p, f'{tag}_Nose3',  1860, 2250, DOOR_TCP_Y, DOOR_TCP_Y, 320, 320, 5.0, n_pts=2)

    p.MoveJ(t_home)
    return p

def build_front_90_program(prog_name, tag):
    """
    STAGE 2 (90° Front Bumper & Bonnet):
    Uses DOOR_TCP_Y = 1880.0 mm for the Bumper and 2020..2480 mm for the Bonnet
    (the exact Y zone that ran without errors in 171157.jpg!), parented to robot_base
    so the yellow path lines sit 100% on the green target arrows!
    """
    p = RDK.AddProgram(prog_name, robot)
    p.setSpeed(150)
    p.setSpeedJoints(14)
    p.MoveJ(t_home)

    # 1. Front Bumper Fascia (4 Full-Width MoveL Sweeps at Y = DOOR_TCP_Y = 1880 mm)
    bmp_z = [300.0, 410.0, 520.0, 630.0]
    for idx, zv in enumerate(bmp_z):
        xs, xe = (-850.0, 850.0) if idx % 2 == 0 else (850.0, -850.0)
        add_linear_pass(p, f'{tag}_Bmp{idx+1}', xs, xe, DOOR_TCP_Y, DOOR_TCP_Y, zv, zv, 5.0, n_pts=3)

    # 2. Front Bonnet / Hood (5 Full-Width MoveL Sweeps from Y = 2000 to Y = 2480 mm)
    bonnet_passes = [
        (DOOR_TCP_Y + 120.0, 740.0, 20.0),
        (DOOR_TCP_Y + 240.0, 800.0, 24.0),
        (DOOR_TCP_Y + 360.0, 860.0, 28.0),
        (DOOR_TCP_Y + 480.0, 920.0, 32.0),
        (DOOR_TCP_Y + 600.0, 970.0, 35.0)
    ]
    for idx, (yv, zv, ang) in enumerate(bonnet_passes):
        xs, xe = (800.0, -800.0) if idx % 2 == 0 else (-800.0, 800.0)
        add_linear_pass(p, f'{tag}_Bonnet{idx+1}', xs, xe, yv, yv, zv, zv, ang, n_pts=3)

    p.MoveJ(t_home)
    return p

def build_rear_270_program(prog_name, tag):
    """
    STAGE 4 (270° Rear Bumper & Back Dikki):
    Uses DOOR_TCP_Y = 1880.0 mm for the Rear Bumper and 2000..2440 mm for the Dikki,
    parented to robot_base so the yellow path lines match the green target arrows!
    """
    p = RDK.AddProgram(prog_name, robot)
    p.setSpeed(150)
    p.setSpeedJoints(14)
    p.MoveJ(t_home)

    # 1. Rear Bumper & Tailgate (4 Full-Width MoveL Sweeps at Y = DOOR_TCP_Y = 1880 mm)
    rear_z = [300.0, 440.0, 580.0, 720.0]
    for idx, zv in enumerate(rear_z):
        xs, xe = (-850.0, 850.0) if idx % 2 == 0 else (850.0, -850.0)
        add_linear_pass(p, f'{tag}_RBmp{idx+1}', xs, xe, DOOR_TCP_Y, DOOR_TCP_Y, zv, zv, 5.0, n_pts=3)

    # 2. Full Back Dikki / Trunk Deck (4 Full-Width MoveL Sweeps from Y = 2000 to Y = 2420 mm)
    dikki_passes = [
        (DOOR_TCP_Y + 120.0,  900.0, 22.0),
        (DOOR_TCP_Y + 260.0,  940.0, 26.0),
        (DOOR_TCP_Y + 400.0,  980.0, 30.0),
        (DOOR_TCP_Y + 540.0, 1020.0, 34.0)
    ]
    for idx, (yv, zv, ang) in enumerate(dikki_passes):
        xs, xe = (780.0, -780.0) if idx % 2 == 0 else (-780.0, 780.0)
        add_linear_pass(p, f'{tag}_Dikki{idx+1}', xs, xe, yv, yv, zv, zv, ang, n_pts=3)

    p.MoveJ(t_home)
    return p

# Build all 4 stage programs:
prog1 = build_driver_0_program('Prog_1_Driver_0deg',          'S1')
prog2 = build_front_90_program('Prog_2_Front_90deg',          'S2')
prog3 = build_passenger_180_program('Prog_3_Passenger_180deg', 'S3')
prog4 = build_rear_270_program('Prog_4_Rear_270deg',          'S4')

# Hide S2, S3, S4 targets initially so only Pose 1 targets are shown
for t in RDK.ItemList(ITEM_TYPE_TARGET):
    tname = t.Name()
    t.setVisible(tname.startswith('S1_') or tname == 'Target_Home')

robot.setJoints(t_home.Joints())
RDK.Render(True)
print("SUCCESS: All 4 Poses Built Directly in robot_base (Zero Shift + Zero Popups)!")