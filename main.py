import matplotlib.pyplot as plt
import numpy as np

import worldClass
from planner import *

def main():
    # GENERATE THE ENVIRONMENT
        # Pull the obstacles from a different file
        # Generate random start and goal points

    world = worldClass.World(state, size, obstacles)
        # input parameters can be state = 0 and size and obstacles, or state = 1,2,3... for predetermined size and obstacles
    world.generate_env()
        # Prints out the plot of the grid space and the polygonal obstacles (squares)
    world.generate_endpoints()
        # Sets values of world.start and world.goal internally
        # Randomly picks two "valid" endpoints for start and goal
        # h(start) - h(goal) > epsilon for some heuristic function h(x, y) and some epsilon > 0

    # START GLOBAL PLANNER
        # GP calculates a path from start to goal
        # Determines waypoints on the path that are the furthest from obstacles
            # Maybe uses voronoi to get "farthest" waypoints
        # Make sure no obstacles in between waypoints

    GP = global_planner("global_planner_name")
    world.run_global(GP)
    world.display(global_path = 1, waypoints = 1)

    # START LOCAL PLANNER
        # Assuming that the GP generated N waypoints
        # Start local planning for pairs of waypoints W(i) to W(i+1) for i = 1:N-1
        # If need be, discretize proportional to the distance between W(i) and W(i+1) with say ~100 grid points

    LP = local_planner("local_planner_name")
    waypoints = world.waypoints
    world.local_paths = {}
    for i in (range(len(waypoints)))[:-1]:
        w1, w2 = w[i], w[i+1]
        world.local_paths[(w1, w2)] = world.run_local(LP, w1, w2)
        # Let world.run_local() return all points on path INCLUDING start and goal
        world.display(global_path = 1, waypoints = 1, local_paths = 1)
    
    # Generate the full final path for the world object
    world.full_path = []
    for key in world.local_paths.key:
        mini_path = (world.local_paths[key])[:-1]
        # Omit just the goal from each path
        world.full_path.extend(mini_path)
    world.full_path.append(world.goal)

    # RUN COMBINATIONS : w Path Smoother?
        # GP:D*Lite + LP:Bug/APF
        # GP:RRT* + LP:APF
        # GP:(RRT + PRM/APF around obstacles) + LP:Bug
        
    print("Done!")


if __name__ == "__main__":
    main()