#!/usr/bin/env python3

"""
This file implements two different action spaces for cozmo to use:
    Generic Action Space
    Object Oriented Action Space
"""
import cozmo
from cozmo.util import distance_mm, pose_z_angle, radians

import math
import numpy as np

from action_util import create_choices

class Action(object):
    """
    An action class that stores the following information:
        linear velocity
        angular velocity
        duration
    """
    def __init__(self, lin_vel, ang_vel, duration):
        """
        Parameters
        ----------
        lin_vel : float
            linear velocity, in millimeters/s
        ang_vel : float
            angular velocity, in millimeters/s
        duration : float
            duration, in seconds
        """

        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.duration = duration

    def __str__(self):
        return "Linear Velocity: %s, Angular Velocity: %s, Duration: %s" % \
                (self.lin_vel, self.ang_vel, self.duration)

    def __repr__(self):
        return "Action(%s, %s, %s)" % (self.lin_vel, self.ang_vel, self.duration)

class GenericActionSpace(object):
    """
    An action space class that generates possible actions 
    for cozmo to execute given the following constraints:
        minimum, maximum, and number of samples for:
            linear velocity, in millimeters/s
            angular velocity, in millimeters/s
            duration, in seconds
    
    Note: Linear Velocity > 0 represents forward movement, 
                          < 0 represents backward movement,
                          == 0 represents no movement in this direction
          Angular Velocity > 0 represents clockwise rotation, 
                           < 0 represents counterclockwise, 
                           == 0 represents no movement in this direction
          
          Linear Velocity is defined as the velocity of cozmo's left and right wheel 
            moving in the same direction
          Angular Velocity is defined as the velocity of cozmo's left and right wheel
            moving in opposite directions, for example, for an angular velocity of 100 mm/s,
            this will correspond to the left wheel moving forward at 100 mm/s and the right 
            wheel moving backwards at 100 mm/s

    Methods
    -------
    apply_action(index)
        applies an action from the generated action space
    get_action_space()
        returns the action space
    view_action_space()
        output all the actions in the action space with the corresponding indices
    """
    def __init__(self, cozmo, lin_min, lin_max, lin_samples, ang_min, ang_max, ang_samples, dur_min, dur_max, dur_samples):
        """
        Parameters
        ----------
        cozmo : cozmo.robot
            the cozmo SDK robot handle
        lin_min : float
            the minimum linear velocity to be generated, in millimeters/s
        lin_max : float
            the maximum linear velocity to be generated, in millimeters/s
        lin_samples : int
            the number of samples to be generated for linear velocity
        ang_min : float
            the minimum angular velocity to be generated, in millimeters/s
        ang_max : float
            the maximum angular velocity to be generated, in millimeters/s
        ang_samples : int
            the number of samples to be generated for angular velocity
        dur_min : float
            the minimum duration to be generated, in seconds
        dur_max : float
            the maximum duration to be generated, in seconds
        dur_samples : int
            the number of samples to be generated for angular velocity
        """

        self.cozmo = cozmo
        self.lin_min = lin_min
        self.lin_max = lin_max
        self.lin_samples = lin_samples
        self.ang_min = ang_min
        self.ang_max = ang_max
        self.ang_samples = ang_samples
        self.dur_min = dur_min
        self.dur_max = dur_max
        self.dur_samples = dur_samples
        self._generate_actions()
        
    def apply_action(self, action_id):
        """
        Applies an action from the action space based on the id

        Parameters
        ----------
        action_id : int
            the id of the action in the action space

        Raises
        ------
        IndexError
            if the id is not a valid id in the action space
        """

        try:
            action = self.actions[action_id]
            print('Applying action: ', action)
            left_wheel = action.lin_vel + action.ang_vel
            right_wheel = action.lin_vel - action.ang_vel
            self.cozmo.drive_wheels(left_wheel, right_wheel, duration=action.duration)
        except IndexError:
            print(action_id, ' is an invalid id\n Enter an id between 0 and ', len(self.actions) - 1)
    
    def get_action_space(self):
        return self.actions

    def view_action_space(self):
        """
        Outputs all the actions from the action space with their corresponding indices
        """

        index = 0
        for action in self.actions:
            if index % 5 == 0:
                print()
            print(index, ' : ', action)
            index += 1

    def _generate_actions(self):
        """
        Helper function to generate the action space
        """

        self.actions = []
        lin_choices = create_choices(self.lin_min, self.lin_max, self.lin_samples, True)
        ang_choices = create_choices(self.ang_min, self.ang_max, self.ang_samples, True)
        dur_choices = create_choices(self.dur_min, self.dur_max, self.dur_samples, False)

        for dur_choice in dur_choices:
            for lin_choice in lin_choices:
                for ang_choice in ang_choices:
                    if not lin_choice == ang_choice == 0 or dur_choice == 0:
                        action = Action(lin_choice, ang_choice, dur_choice)
                        self.actions.append(action)

class ObjectOrientedActionSpace(object):
    """
    An action space class that generates possible actions 
    for cozmo to execute relative to an object, in this class
    we assume it to be a cube

    Given the number of cube offsets, the min, max values for 
    linear velocity and duration as well as the number of samples
    for each, this class will generate lin_samples * dur_samples * cube offsets * 4
    number of actions 
    
    Cube offset represents the horizontal offsets from the center of the edge of a cube

    Methods
    -------
    apply_action(index)
        applies an action from the generated action space
    view_action_space()
        output all the actions in the action space with the corresponding indices
    """
    def __init__(self, cozmo, cube, samples, lin_min, lin_max, lin_samples, dur_min, dur_max, dur_samples):
        """
        Parameters
        ----------
        cozmo : cozmo.robot
            the cozmo SDK robot handle
        cube : cozmo.objects.LightCube
            the object we are moving relative to
        samples : int
            the number of samples per side of the cube
        lin_min : float
            the minimum linear velocity to be generated, in millimeters/s
        lin_max : float
            the maximum linear velocity to be generated, in millimeters/s
        lin_samples : int
            the number of samples to be generated for linear velocity
        dur_min : float
            the minimum duration to be generated, in seconds
        dur_max : float
            the maximum duration to be generated, in seconds
        dur_samples : int
            the number of samples to be generated for angular velocity
        """

        self.cozmo = cozmo
        self.cube = cube
        self.samples = samples
        self.lin_min = lin_min
        self.lin_max = lin_max
        self.lin_samples = lin_samples
        self.dur_min = dur_min
        self.dur_max = dur_max
        self.dur_samples = dur_samples

        self.locations = []
        self.location_names = []
        self.actions = []
        self.action_names = []
        self._generate_actions()

    def apply_action(self, action_id):
        """
        Applies an action from the action space based on the id

        Parameters
        ----------
        action_id : int
            the id of the action in the action space

        Raises
        ------
        IndexError
            if the id is not a valid id in the action space
        """

        try:
            print('Moving to: ', self.action_names[action_id][0], ' and apply action: ', self.action_names[action_id][1])
            action = self.cozmo.go_to_pose(self.actions[action_id][0])
            action.wait_for_completed()
            
            action = self.actions[action_id][1]
            left_wheel = action.lin_vel + action.ang_vel
            right_wheel = action.lin_vel - action.ang_vel
            self.cozmo.drive_wheels(left_wheel, right_wheel, duration=action.duration)
        except IndexError:
            print(action_id, 'is an invalid id\nEnter an id between 0 and', len(self.actions) - 1)
    
    def view_action_space(self):
        """
        Outputs all the actions from the action space with their corresponding indices
        """
        idx = 0
        for action in self.action_names:
            if idx % 5 == 0:
                print()
            print(idx, ': ', action[0], ' ', action[1])
            idx += 1

    def _cube_offset(self, offset, angle):
        """
        Helper function to calculate offset distances from the cube

        Parameters
        ----------
        offset : float
            the amount to offset by, in millimeters
        angle : float
            the angle of the cube, in radians
        """
        return offset * math.cos(angle), offset * math.sin(angle)
    
    def _find_sides(self, angle):
        """
        Helper function to find the location of all 4 sides of the cube

        Parameters
        ----------
        angle : float
            the angle of the cube, in radians
        
        returns a sorted list of the angle of each of the 4 sides where index
            0 corresponds to front of cube
            1 corresponds to left of cube
            2 corresponds to back of cube
            3 corresponds to right of cube
        """

        sides = [angle]
        for _ in range(3):
            # Adding in clockwise order
            angle -= math.pi / 2
            if angle < -math.pi:
                angle = 2 * math.pi + angle
            sides.append(angle)
        return sides

    def _generate_actions(self, h_offset=40, v_offset=60):
        """
        Helper function to generate the action space
        """

        self._generate_offsets(h_offset, v_offset)
        gen_space = GenericActionSpace(self.cozmo, self.lin_min, self.lin_max, self.lin_samples, 0, 0, 0, self.dur_min, self.dur_max, self.dur_samples)
        actions = gen_space.get_action_space()

        for i in range(len(self.locations)):
            for action in actions:
                self.actions.append((self.locations[i], action))
                self.action_names.append((self.location_names[i], action))

    def _generate_offsets(self, h_offset, v_offset):
        """
        Helper function to generate cube offset positions

        Parameters
        ----------
        h_offset : float
            the max horizontal offset from the center of the edge of the cube, in millimeters
        v_offset : float
            the vertical offset away from the center of the cube, in millimeters
        """
        if self.samples == 1:
            choices = [0]
        else:
            choices = create_choices(-h_offset, h_offset, self.samples, True)
        
        x = self.cube.pose.position.x
        y = self.cube.pose.position.y
        z = self.cube.pose.position.z
        
        cube_rotation = self.cube.pose.rotation.angle_z.radians
        cube_sides = self._find_sides(cube_rotation)
        
        key_idx = 0
        num_actions = len(cube_sides) * len(choices)

        for side in cube_sides:
            if key_idx < num_actions / 4:
                cube_side = 'front of cube '
            elif key_idx < num_actions / 2:
                cube_side = 'left of cube '
            elif key_idx < num_actions * 3/4:
                cube_side = 'back of cube '
            else:
                cube_side = 'right of cube '

            for choice in choices:
                offset = 'with offset: ' + str(choice)
                x_offset, y_offset = self._cube_offset(v_offset, side)
                cy_offset, cx_offset = self._cube_offset(choice, side)
                
                location = pose_z_angle(x - x_offset + cx_offset, y - y_offset - cy_offset, z, radians(side))
                
                self.locations.append(location)
                self.location_names.append(cube_side + offset)
                key_idx += 1
    
def cozmo_run(robot: cozmo.robot):
    #action = GenericActionSpace(robot, 10, 100, 5, 10, 100, 5, 1, 5, 5)
    #action.view_action_space()
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cube = None
    try:
        cube = robot.world.wait_for_observed_light_cube(timeout=30)
    except asyncio.TimeoutError:
        print('no cube')
    finally:
        look_around.stop()
    
    oos = ObjectOrientedActionSpace(robot, cube, 3, 10, 100, 3, 1, 5, 3)
    print(cube.pose)
    oos.view_action_space()
    oos.apply_action(81)
    
if __name__ == '__main__':
    cozmo.run_program(cozmo_run)
