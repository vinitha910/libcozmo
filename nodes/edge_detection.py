#!/usr/bin/env python3
import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

def cozmo_program(robot: cozmo.robot.Robot):
    # robot.is_cliff_detected:
    done = False
    while not done:
        drive = robot.drive_wheel_motors(50, 50)
        if robot.is_cliff_detected:
            robot.stop_all_motors()
            done = True
    #if robot.is_cliff_detected:
    #    robot.stop(drive)
    
    robot.world.request_nav_memory_map(0.1)
    memory_map = robot.world.wait_for(cozmo.nav_memory_map.EvtNewNavMemoryMap)
    print(memory_map.nav_memory_map)
cozmo.run_program(cozmo_program)
