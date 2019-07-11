#!/usr/bin/env python3

import cozmo
import math
import numpy as np

class Action(object):
    def __init__(self, lin_vel, ang_vel, duration):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.duration = duration

    def __str__(self):
        return "Linear Velocity: %s, Angular Velocity: %s, Duration: %s" % \
                (self.lin_vel, self.ang_vel, self.duration)

    def __repr__(self):
        return "Action(%s, %s, %s)" % (self.lin_vel, self.ang_vel, self.duration)

class ActionSpace(object):
    def __init__(self, cozmo, lin_min, lin_max, lin_samples, ang_min, ang_max, ang_samples, dur_min, dur_max, dur_samples):
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
        self.generate_actions()
        
    def generate_actions(self):
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
        print(len(self.actions))
    
    # Combine both values to 
    def apply_action(self, index):
        if index >= len(self.actions) or index < 0:
            print(index, ' is an invalid index\n Enter an index between 0 and ', len(self.actions) - 1)
            return
        action = self.actions[index]
        print(action)
        self.cozmo.drive_wheels(action.lin_vel + action.ang_vel, action.lin_vel - action.ang_vel,duration=action.duration)

    def _create_choices(self, start, stop, num, include_zero):
        choices = np.linspace(start, stop, num)
        if include_zero:
            choices = np.unique(np.append(choices, 0))
        return [self._truncate(val, 3) for val in list(choices)]


    def _truncate(self, number, digits) -> float:
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper

def cozmo_run(robot: cozmo.robot):
    action = ActionSpace(robot, 10, 100, 5, 10, 100, 5, 1, 5, 5)
    action.apply_action(130)

if __name__ == '__main__':
    cozmo.run_program(cozmo_run)
