#!/usr/bin/env python3

"""
This file implements a generic action space for cozmo to use given certain contraints
"""
import cozmo
from cozmo.util import distance_mm

import math
import numpy as np

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

class ActionSpace(object):
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

        if action_id >= len(self.actions) or action_id < 0:
            raise IndexError(action_id, ' is an invalid id\n Enter an id between 0 and ', len(self.actions) - 1)

        action = self.actions[action_id]
        print('Applying action: ', action)
        left_wheel = action.lin_vel + action.ang_vel
        right_wheel = action.lin_vel - action.ang_vel
        self.cozmo.drive_wheels(left_wheel, right_wheel, duration=action.duration)
    
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
        lin_choices = self._create_choices(self.lin_min, self.lin_max, self.lin_samples, True)
        ang_choices = self._create_choices(self.ang_min, self.ang_max, self.ang_samples, True)
        dur_choices = self._create_choices(self.dur_min, self.dur_max, self.dur_samples, False)

        for dur_choice in dur_choices:
            for lin_choice in lin_choices:
                for ang_choice in ang_choices:
                    if not lin_choice == ang_choice == 0 or dur_choice == 0:
                        action = Action(lin_choice, ang_choice, dur_choice)
                        self.actions.append(action)

    def _create_choices(self, start, stop, num, include_zero):
        """
        Helper function to generate choices, generates [num] number of choices
        from [start] to [stop]

        Parameters
        ----------
        start : float
           the starting value of the sequence
        stop : float
            the end value of the sequence
        num : int
            number of samples to generate
        include_zero : bool
            True to add zero to choices, False to not
            This allows for 0 velocity in either the linear or angular direction

        Returns a list of choices
        """

        choices = np.linspace(start, stop, num)
        if include_zero:
            choices = np.unique(np.insert(choices, 0, 0, axis=0))
        return [self._truncate(val, 3) for val in list(choices)]


    def _truncate(self, number, digits) -> float:
        """
        Helper function to truncate floats to specified number of decimal places

        Parameters
        ----------
        number : float
            the number to truncate
        digits : int
            the number of decimal places to keep

        Returns the truncated number
        """
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper

def cozmo_run(robot: cozmo.robot):
    action = ActionSpace(robot, 10, 100, 5, 10, 100, 5, 1, 5, 5)
    action.view_action_space()
    action.apply_action(174)

if __name__ == '__main__':
    cozmo.run_program(cozmo_run)
