class World:
    def __init__(self, state=None, size=None, obstacles=None):
        """
        Constructor function.

        Args: 
            state (int) : value for predetermined environment state
            size (list, len = 2) : size of state space; {form : (M x N)}
            obstacles (list(list)) : list of each list of points defining each polygonal obstacle

        Properties:
            start (tuple) : start point coordinate
            goal (tuple) : goal point coordinate
            waypoints (list) : all waypoints generated by global planner
            local_paths (dict) : (w1, w2) -> list of points in path between w1 and w2
            full_path (list) : list of all points in final path between start and goal
        """

        # Edit these to include the constructor use cases
        self.state = state
        self.size = size
        self.obstacles = obstacles

        self.start = None
        self.goal = None
        self.waypoints = list()
        self.local_paths = dict()
        self.full_path = list()


    def generate_env():
        """
        Plots the state space and the obstacles.

        Args : None

        Returns : None
        """

        print("generate_env() called")
        pass

    def generate_endpoints():
        """
        Plots start and goal points in already generated state space.


        Args : None

        Returns : None
        """

        print("generate_endpoints() called")
        pass

    def run_global(GP):
        """
        Generates the global planner solution.

        Args:
            GP (function) : global planner function

        Returns:
            waypoints (list) : list of waypoints for local planner to follow
        """

        if callable(GP):
            print(f"run_global() called with Global Planner : {str(GP.__name__)}")
        else:
            print("run_global() called incorrectly -> GP is not a function")
        pass

    def run_local(LP, w1, w2):
        """
        Generates the local planner solution given 2 waypoints.

        Args:
            LP (function) : local planner function
            w1 (tuple) : waypoint 1 coordinate
            w2 (tuple) : waypoint 2 coordinate

        Returns:
            local_path (list) : points on the local planner solution
        """

        if callable(LP):
            print(f"run_local() called with Global Planner : {str(LP.__name__)} between {w1} and {w2}")
        else:
            print("run_local() called incorrectly -> LP is not a function")
        pass


    def display(global_path = 0, waypoints = 0, local_paths = 0):
        """
        Displays different aspects of the solution.

        Args:
            global_path (bool) : toggle for displaying global_path
            waypoints (bool) : toggle for displaying waypoints
            local_paths (bool) : toggle for displaying all local paths planned so far

        Returns : None
        """

        if global_path:
            # Do something to show the global path in the figure
            pass

        if waypoints:
            # Do something to plot the waypoint points in the figure
            pass

        if local_paths:
            # Do something to show the local paths in the figure as they are being drawn
            pass

    

