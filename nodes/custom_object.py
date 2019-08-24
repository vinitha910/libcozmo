#!/usr/bin/env python3
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes

def cozmo_program(robot: cozmo.robot.Robot):
    box = robot.world.define_custom_box(
        CustomType00,
        Circles2, # Front
        Circles3, # Back
        Diamonds2, # Top
        Diamonds3, # Bottom
        Circles4, # Left
        Circles5, # Right
        228, # Depth, X_axis, in mm
        123, # Width, Y_axis, in mm
        53,  # Height, Z_axis, in mm
        58.5, # Marker_width, in mm
        58.5, # Marker_height, in mm
        True)
    if box is not None:
        print("Box defined successfully")

cozmo.run_program(cozmo_program)
