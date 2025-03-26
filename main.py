import matplotlib.pyplot as plt
import numpy as np

import worldClass
from planner import *

def main():
    
    # world = worldClass.World(state, size, obstacles)
    state = 0
    world = worldClass.World(state = state)
    
    world.generate_endpoints()
    world.display()
    print("Displaying the basic environment")

    """
    START GLOBAL PLANNER
        GP calculates a path from start to goal
        Determines waypoints on the path that are the furthest from obstacles
            Maybe uses voronoi to get "farthest" waypoints
        Make sure no obstacles in between waypoints
    """

    GP = global_planner("global_planner_name")
    world.run_global(GP)
    world.display(waypoints = True)
    print("Displaying environment with waypoints")

    """
    START LOCAL PLANNER
        Assuming that the GP generated N waypoints
        Start local planning for pairs of waypoints W(i) to W(i+1) for i = 1:N-1
        If need be, discretize proportional to the distance between W(i) and W(i+1) with say ~100 grid points
    """

    # Let world.run_local() return all points on path INCLUDING start and goal
    LP = local_planner("local_planner_name")
    waypoints = world.waypoints
    world.local_paths = {}
    for i in (range(len(waypoints[:-1]))):
        w1, w2 = waypoints[i], waypoints[i+1]
        world.local_paths[(w1, w2)] = world.run_local(LP, w1, w2)
        world.display(waypoints = True, local_paths = True, timer=2)
        print(f"Displaying environment with waypoints and local paths between w1 to w{i+2}")

    # # DEBUG : Dummy locally planned path test
    # LP = local_planner("local_planner_name")
    # world.waypoints = [(1, 1), (2, 2), (3, 3)]
    # for i in (range(len(world.waypoints[:-1]))):
    #     if i == 0:
    #         world.local_paths = {
    #             ((1, 1), (2, 2)):[(1, 1), (1.25, 1.25), (1.5, 1.5), (1.75, 1.75), (2, 2)],
    #         }
    #     elif i == 1:
    #         world.local_paths = {
    #             ((1, 1), (2, 2)):[(1, 1), (1.25, 1.25), (1.5, 1.5), (1.75, 1.75), (2, 2)],
    #             ((2, 2), (3, 3)):[(2, 2), (2.25, 2), (2.25, 2.5), (2.75, 2.5), (3, 3)],
    #         }
    #     world.display(waypoints = True, local_paths = True, timer = 2)
    #     print(f"Displaying environment with waypoints and local paths between w1 to w{i+2}")
    
    # Generate the full final path for the world object
    world.full_path = []
    for key in world.local_paths.keys():
        mini_path = (world.local_paths[key])[:-1]
        # Omit just the goal from each path
        world.full_path.extend(mini_path)
    world.full_path.append(world.goal)

    """
    RUN COMBINATIONS : w Path Smoother?
        GP:D*Lite + LP:Bug/APF
        GP:RRT* + LP:APF
        GP:(RRT + PRM/APF around obstacles) + LP:Bug
    """    
        
    print("Done!")


if __name__ == "__main__":
    main()