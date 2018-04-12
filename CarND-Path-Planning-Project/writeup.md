**Path Planning Project**

The goals / steps of this project are the following:
* design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic.
* The path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using 
localization, sensor fusion, and map data.

## Rubric Points

####1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* main.cpp: reads and sends data to simulator, handle data step by step 
* vehicle.cpp: path planner determine whether to speed up, slow down or change lanes
* global.cpp: helper functions 
* writeup.md: summarizing the results

####2. The top right screen of the simulator shows the current/best miles driven without incident.      
No collision, exceeding acceleration/jerk/speed occurs.

####3. The car drives according to the speed limit.     
Yes, no more than 50mph unless obstructed by traffic.

####4. Max Acceleration and Jerk are not Exceeded, car does not have collisions, car stays in its lane, 
except for the time between changing lanes, & car is able to change lanes   
Yes, 

### Reflection

### 1. how to generate paths.

Some strategies are adopted:  
  1. Previous trajectories are kept constant when designing current trajectories, in this way, the vehicle moves consistently. 
  2. Five points are used to fit a 3rd spline, the first two points are the last two points of the previous trajectory, and the other three
  points are located far away from the target lane, specifically 30m, 60m, 90m ahead. The target lane is given by the path planner module,
  implemented in the vehicle.cpp, which decides whether to keep lane or change lane. 
  3. The fitted spline is used to generate the current trajectory. Basically, for each point, given its x-value, a y-value is computed using 
  the equation of the spline. 
  4. Discretizing the current trajectory into several waypoints, the location (x,y) of these points are computed. The result of the waypoints 
  is the generated path. 
  
### More In-depth Materials

http://ais.informatik.uni-freiburg.de/teaching/ss10/robotics/slides/16-pathplanning.pdf   
http://correll.cs.colorado.edu/?p=965   
http://www.coppeliarobotics.com/helpFiles/en/pathPlanningModule.htm   
https://www.cs.cmu.edu/afs/cs/project/jair/pub/volume9/mazer98a-html/node2.html   
http://www.roborealm.com/help/Path_Planning.php   
https://wesscholar.wesleyan.edu/cgi/viewcontent.cgi?referer=&httpsredir=1&article=1856&context=etd_hon_theses   
http://www.scielo.org.co/pdf/inge/v21n2/v21n2a05.pdf    
https://medium.com/@mohankarthik/path-planning-in-highways-for-an-autonomous-vehicle-242b91e6387d   
https://www.sciencedirect.com/science/article/pii/S0968090X15003447   


