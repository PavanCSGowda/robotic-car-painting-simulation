from robolink import *
import math
import time
import tkinter as tk
from tkinter import simpledialog

RDK = Robolink()
car_frame = RDK.Item('Car_Frame', ITEM_TYPE_FRAME)
MatClass = car_frame.Pose().__class__

CAR_SIDE_Y = 3010.0  # Locked for 0° and 180°
CAR_END_Y  = 4300.0  # Locked for 90° and 270° (Places Bumper at Y=1900, TCP at 1880!)

def show_only_prefix(prefix):
    """Shows ONLY targets starting with 'S1_', 'S2_', 'S3_', or 'S4_' so screen is clean!"""
    RDK.Render(False)
    for t in RDK.ItemList(ITEM_TYPE_TARGET):
        tname = t.Name()
        t.setVisible(tname.startswith(prefix) or tname == 'Target_Home')
    RDK.Render(True)

def set_car_pose(deg, y_pos):
    ang = math.radians(deg)
    ca, sa = float(math.cos(ang)), float(math.sin(ang))
    car_frame.setPose(MatClass([
        [ca, -sa, 0.0, 0.0],
        [sa,  ca, 0.0, float(y_pos)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]))

def animate_turntable(start_deg, end_deg, y_start, y_end):
    for step in range(25):
        f = step / 24.0
        set_car_pose(start_deg + f * (end_deg - start_deg), y_start + f * (y_end - y_start))
        time.sleep(0.03)

def run_stage_blocking(prog_name):
    p = RDK.Item(prog_name, ITEM_TYPE_PROGRAM)
    if p.Valid():
        p.RunProgram()
        while p.Busy():
            time.sleep(0.1)

root = tk.Tk()
root.withdraw()
root.attributes("-topmost", True)
choice = simpledialog.askstring(
    "Select Pose to Run",
    "Enter Pose Number (0, 1, 2, 3, or 4):\n\n"
    "0 = Run ALL 4 Poses Together (Full 360° Car)\n"
    "1 = Pose 1 ONLY (0° Driver Side)\n"
    "2 = Pose 2 ONLY (90° Front Bumper & Bonnet)\n"
    "3 = Pose 3 ONLY (180° Passenger Side)\n"
    "4 = Pose 4 ONLY (270° Rear Bumper & Back Dikki)",
    initialvalue="0",
    parent=root
)
root.destroy()

if choice is not None:
    mode = str(choice).strip()

    if mode == "1":
        show_only_prefix('S1_')
        set_car_pose(0.0, CAR_SIDE_Y)
        run_stage_blocking('Prog_1_Driver_0deg')

    elif mode == "2":
        show_only_prefix('S2_')
        set_car_pose(90.0, CAR_END_Y)
        run_stage_blocking('Prog_2_Front_90deg')

    elif mode == "3":
        show_only_prefix('S3_')
        set_car_pose(180.0, CAR_SIDE_Y)
        run_stage_blocking('Prog_3_Passenger_180deg')

    elif mode == "4":
        show_only_prefix('S4_')
        set_car_pose(270.0, CAR_END_Y)
        run_stage_blocking('Prog_4_Rear_270deg')

    elif mode == "0":
        show_only_prefix('S1_')
        set_car_pose(0.0, CAR_SIDE_Y)
        run_stage_blocking('Prog_1_Driver_0deg')

        animate_turntable(0.0, 90.0, CAR_SIDE_Y, CAR_END_Y)
        show_only_prefix('S2_')
        run_stage_blocking('Prog_2_Front_90deg')

        animate_turntable(90.0, 180.0, CAR_END_Y, CAR_SIDE_Y)
        show_only_prefix('S3_')
        run_stage_blocking('Prog_3_Passenger_180deg')

        animate_turntable(180.0, 270.0, CAR_SIDE_Y, CAR_END_Y)
        show_only_prefix('S4_')
        run_stage_blocking('Prog_4_Rear_270deg')

        animate_turntable(270.0, 360.0, CAR_END_Y, CAR_SIDE_Y)
        show_only_prefix('S1_')