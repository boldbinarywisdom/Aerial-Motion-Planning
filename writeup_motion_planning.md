# -----------------------------
## Project: 3D Motion Planning
# -----------------------------

### Ashutosh Zade
### Project 2 submission write-up
### Motion Planning
### April 14th, 2018

# -----------------------------

---

## Purpose:

The purpose of this project is to plan a path in a city using FCND simulator. This is achieved by following steps:

(This section is from original template provided)

1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].

---

## Writeup / README (this file)


### Brief about the Starter Code

#### 1. What's provided in `motion_planning.py` and `planning_utils.py`

##### motion_planning.py

This starter file provides MotionPlanning class which implements following callback functions. 

1. local_postion_callback() - this function gets called once the drone's position changes
2. velociy_callback() - this function gets called when velocity of the drone changes
3. state_callback() - this gets called when state of the drone changes
4. state_transition functions allow drone to transition from one state to another. Drone states are defined here. 
5. Perhaps the most important function it provides in the Drone class is PlanPath. The function does following:
 a. reads and sets home position specified in collision map file -- note the position is read in lat, lon
 b. coverts current position to global position
 c. covernts global position to local position using global_to_local() mapping -- note the coverted position in NED
 d. generates a grid from obstacle map
 e. allows to set ga oal (goal can be changed in the code or script can be modified to accept new goal) 
 f. runs A* algorithm to derive best path (A* notes below)
 g. uses path pruning to prune the path (avoids stopping at intermediate waypoints unnecessary)
 h  covernts path to waypoints and returns waypoints list
6. The class also generates logs for the execution

##### plannint_utils.py

This file provides a_star() algorithm in additional to other utility functions such as 
 a. heuristic function
 b. prune_path function
 c. collinearity function used by prune_path
 
 A* -- this is a pathfinding algorithm which combines best of Dijkstra's and breadth first algorithm. It derives path based upon cost to move from starting point to the goal. It uses heuristics as a guide to move to the goal. 
 
 The A* implementation is modified to use diagonal motions on the grid that have cost of sqrt(2). The diagonal distance is also called as manhattan distance. 
 
 ##### diff between motion_planning.py and backyard_flyer.py
 
 1. motion_planning adds a new state called PLANNING as backyard_flyer was not required to move autonomously
 2. motion_planning plans a path before takeoff
 3. motion_planning sends waypoints calculated as path to the simulator whih allows drone to fly within the FCND simulator
 4. what's common between the two implementations is basic code that provides callback routines and connecting to the simulator
 


### Path Planning Algorithm

#### 1. Set your global home position

Read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. 

###### Lat and Lon are specified in the csv file header in the colliders file on the first line. The code snippet is as follows
    
      lat0, lon0 = readLatLon('colliders.csv')
      f = open('colliders.csv','r')
      line = f.readlines()[:2]
      lat0, lon0 = line[1].split(',')
    
###### The lat0 and lon0 are read in below variables as floating point entities from current version of colliders.csv file
    
     lat0 = 37.792480
     lon0 = -122.397450    

#### 2. Setting the current local position

The current local position is set as follows:

      self.set_home_position(lat0, lon0, 0)
      
      Note that we are only dealing with 2D map so third parameter (altitude - Z) is 0


#### 3. Setting grid start position from local position

This is another step in adding flexibility to the start location. As long as it works you're good to go!


#### 4. Setting grid goal position from geodetic coords


This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.


#### 5. Modified A* to include diagonal motion 

The code is modified in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2). This is an optimization which uses the priciple of pythogorian distance (the hypotenuse) is always shorter than sum of two sides in a right angle triangle. 

The optimization is used by passing a new heuristic function which calculates the diagonal distnace instead of linear distnace between two points say p1 and p2. See code for both the heuristic functions below for comparison. 

    def heuristic(position, goal_position):
        return np.linalg.norm(np.array(position) - np.array(goal_position))

    def heuristic_sqrt(position, goal_position):
        return np.sqrt(np.linalg.norm(np.array(position[0]) - np.array(goal_position[0])**2) + np.linalg.norm(np.array(position[    1]) - np.array(goal_position[1])**2))

##### invoke a_star() as below to use new heuristics

    path, _ = a_star(grid, heuristic_sqrt, grid_start, grid_goal)

#### 6. Culling the waypoints 

If three points (p1, p2, p3) lie on the same line then p2 can be dropped from the waypoint to achieve an optimization while travelling from p1 to p3. To test if three points are collinear is to test, collinearity test can be used. Below is the python code that is used. It uses, np.linalg.det() function to check if deterrant is less than the error margin to be on the line or tolerance value. 

    def collinearity_check(p1, p2, p3, epsilon=1e-6):
      m = np.concatenate((p1, p2, p3), 0)
      det = np.linalg.det(m)
      return abs(det) < epsilon


### Execute the flight

The waypoints are sent to the emulator and before changing the state to takeoff the found path is executed by the simulator. 

  
### Extra Challenges: Real World Planning (TBD -- 4/14/18)

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
