from math import pi 
import random, os, yaml
import numpy as np

num_robots = 60
min_val = 200
max_val = 700
for i in range(1, 16):
    scenario_num = i

    d = '/home/vinitha/workspaces/cozmo_ws/src/libcozmo/scenarios/'
    directory = d + 'scenario_' + str(scenario_num)
    if not os.path.exists(directory):
        os.makedirs(directory)

    start_poses = []
    goal_poses = []
    while len(start_poses) < num_robots:
        pose = np.array([random.randint(min_val, max_val), random.randint(min_val, max_val)])
        dists = np.array([np.linalg.norm(pose - p) for p in start_poses])
        in_range = dists > 20
        if in_range.all(): 
            start_poses.append(pose)

    while len(goal_poses) < num_robots:
        pose = np.array([random.randint(min_val, max_val), random.randint(min_val, max_val)])
        dists = np.array([np.linalg.norm(pose - p) for p in goal_poses])
        in_range = dists > 20
        if in_range.all(): 
            goal_poses.append(pose)

    p = 0
    for start, goal in zip(start_poses, goal_poses):
        p += 1
        scenario = {}
        scenario['priority'] = p  
        scenario['start_pose'] = {}
        scenario['start_pose']['x_mm'] = int(start[0])
        scenario['start_pose']['y_mm'] = int(start[1])
        scenario['start_pose']['theta_rad'] = random.uniform(0, 2*pi)
        scenario['goal_pose'] = {}
        scenario['goal_pose']['x_mm'] = int(goal[0])
        scenario['goal_pose']['y_mm'] = int(goal[1])
        scenario['goal_pose']['theta_rad'] = random.uniform(0, 2*pi)

        with open(directory + '/robot_' + str(p) + '.yaml', 'w') as yaml_file:
            yaml.dump(scenario, yaml_file, default_flow_style=False)