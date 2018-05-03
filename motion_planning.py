import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import matplotlib.pyplot as plt

from planning_utils import a_star, heuristic, create_grid, heuristic_sqrt
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

#from grid import create_grid_and_edges
from planning_utils import readLatLon
from planning_utils import prune_path


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Planning for a path before takeoff ...")
        TARGET_ALTITUDE = 100
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        #lat0 = 37.792480
        #lon0 = -122.397450

        with open('colliders.csv') as f:
            home_pos_data = f.readline().split(",")
        lat0 = float(home_pos_data[0].strip().split(" ")[1])
        lon0 = float(home_pos_data[1].strip().split(" ")[1])
        print(lat0, lon0)

        ## set home position to (lat0, lon0, 0) ##
        self.set_home_position(lat0, lon0, 0)

        ## retrieve current global position ##
        global_pos = [self._longitude, self._latitude, self._altitude]

        ## convert to current local position using global_to_local ##
        # Note current_local_pos is in NED
        local_pos = global_to_local(global_pos, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        print(data)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        # Note that the grid is in NED
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        print(grid)

        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)

        ## convert start position to current position rather than map center ##
        grid_start = (int(local_pos[0]-north_offset), int(local_pos[1]-east_offset))
        
        # Set goal as some arbitrary position on the grid
        # Changed this to any arbitrary position by adding random offsets to north and east
        # Note: the offsets are updated as per latest FNCD release

        ## adapt to set goal as latitude / longitude position and convert
        #target_global_position =  (-122.400305, 37.791436, 0.0)
        target_global_position =  (-122.400305, 37.791436, 0.0)
        target_local_position = global_to_local(target_global_position, self.global_home)

        grid_goal = (-north_offset + int(target_local_position[0]), -east_offset + int(target_local_position[1]))

        # Run A* to find a path from start to goal
        ## add diagonal motions with a cost of sqrt(2) to your A* implementation ##
        # or move to a different search space such as a graph (not done here)
        print('grid start and grid goal: ', grid_start, grid_goal)

        path, cost = a_star(grid, heuristic_sqrt, grid_start, grid_goal)
        print('Found path: len/cost: ', len(path), cost)
        print('Actual path: ', (path))
        
        # Let's plot the initial path generated by A*

        #plt.plot(grid_start[1], grid_start[0], 's')
        #plt.plot(grid_goal[1], grid_goal[0], 'x')

        #pp = np.array(path)
        #plt.plot(pp[:, 1], pp[: 0], 'g')

        #plt.xlabel['EAST']
        #plt.ylabel['NORTH']
        #plt.show()

        ## prune path to minimize number of waypoints
        ## (if you're feeling ambitious): Try a different approach altogether!

        pruned_path = prune_path(path)
        print('pruned path len: ', len(pruned_path))
        print('Pruned path: ', (pruned_path))

        # Let's plot the pruned path 

        #plt.plot(grid_start[1], grid_start[0], 'S')
        #plt.plot(grid_goal[1], grid_goal[0], 'X')

        #pp = np.array(pruned_path)
        #plt.plot(pp[:, 1], pp[: 0], 'g')

        #plt.xlabel['EAST']
        #plt.ylabel['NORTH']
        #plt.show()

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]

        # Set self.waypoints
        self.waypoints = waypoints

        ##  send waypoints to sim ##
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
