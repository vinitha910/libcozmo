from math import pi, sin, cos
import numpy as np
import json

def generate_waypoints(r, num_waypoints, outfile='waypoints.json'):
    eps = 2*pi/num_waypoints;
    thetas = [th for th in np.arange(0.0,2*pi,eps)]
    waypoints = [[r*cos(th),r*sin(th),th] for th in thetas] 
    
    with open(outfile, 'w') as f:
        json.dump(waypoints,f)

if __name__ == '__main__':
    
    import argparse
    parser = argparse.ArgumentParser(description="Generate waypoints around a circle") 
    
    parser.add_argument("--radius", type=int, default=None,
                        help="The radius of the circle")    
    parser.add_argument("--num_waypoints", type=int, default=None,
                        help="The number of waypoints to generate")
    parser.add_argument("--outfile", type=str, default='waypoints.json',
                        help="The name of the file to write the waypoints to")

    args = parser.parse_args()

    generate_waypoints(args.radius, args.num_waypoints, args.outfile)
