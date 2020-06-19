# CarND-Path-Planning-Project

The code tries to make the car switch to a faster moving lane if the traffic allows so. To find the optimal path dynamic programming method is used. 

1) Firstly a Matrix of size 100 x 3 is made which represents a section of Highway ahead of the grid of 3 lanes and 100 units in S direction in Ferent coordinates. Line : 100 -132

2) Using the sensor data and map data, the grid matrix is populated. 

2) To avoid collision each car gets the value 99 on the cost grid matrix. To penalise slower lanes additional cost is penalised on grid cells behind each individual traffic car. This cost is directly proportional to difference between Max speed limit and traffic car speed. Line : 132 -156

3) Every time the code runs, this matrix gets updated with new data. Objective of populating the grid matrix is to determine the fastest lane. 

4) Basic idea is to compute overall cost incurred after the grid section is traversed. To find the optimal path trajectory for the next 100 units in S direction, the cost computation is done backward. Basic idea behind this approach is that the minimum cost to move current state to the next state is given by the cost involved in taking the action to move to the next state + minimum cost associated with the trajectory from the next state onwards. Line : 156 -214

let's assume every cell of the grid is a state represented as (s,d), where s={0,1,2...98,99} and d={0,1,3}.
J(s+1,d2)* = Optimal cost of traversing the trajectory from this point onwards. size = 1 x 3 (for each lane)

Cost(s,d1 --> s+1,d2) = Total cost of moving from current state (s,d1) to possible next states (s+1,d2) 

J(s,d1) = Cost(s,d1 --> s+1,d2) + J(s+1 ,d2)* , computes all 3 values of d1={0,1,2}

Finally J(s,d1)* is determined by minimizing the values of J(s,d1). size = 1 x 3(for each lane)

Thus the optimal action is recorded against each J(s,d1)*
 
Below is an example of Cost grid Matrix, Next optimal lane state matrix and final trajectory :

1) Cost grid Matrix

    1. The traffic cars in front of the subject car are represented by 99 in each lane.
    2. Left most lane is clear. No Car and hence all zeros.
    3. Middle lane has a car at 8 units ahead of the current location with speed around 41 mph.
    4. Right most lane has a car at 3 units ahead with speed around 42.5 mph.
    5. Fastest lane should be with the lowest cost value.  

0 		99 		0 
0 	9.78104 	0 
0 	9.78104 	0                 ↑   S coordinates 
0 	9.78104 	0                 →   d coordinates
0 	9.78104 	99 
0 	9.78104 	7.47237 
0 	9.78104 	7.47237 
0 	9.78104 	7.47237 
0 	9.78104 	7.47237  <--current location of the subject car

2) Next optimal Lane

Each cell in the matrix below shows the next optimal lane position.

0   0 	2 
0 	0 	2 
0 	0 	2          ↑   S coordinates
0 	0 	2          →   d coordinates     
0 	0 	1 
0 	0 	2          0 --> Left Most lane
0 	0 	2          1 --> Middle Lane
0 	0	2          2 --> Right Most Lane
0	0	2 <--  Current location of the subject car

3) Final Trajectory 

1   0 	0 
1 	0 	0      
1 	0 	0          ↑   S coordinates
1 	0 	0          →   d coordinates
1 	0 	0          1 --> Indicates final trajectory
1 	0 	0 
1 	0 	0 
1 	0	0      Conclusion : Car should move to the left most lane to follow the optimal trejctory
0	1	0 <--  Starting current location (Middle lane)

Now since the next optimal action is known, that is whether to move straight or change the lane, decision has to be taken whether to change the lane. Will it be safe to change lanes? Therefore again Sensor data is referred to determine proximity with the other traffic cars specially the cars in the lane where the car intends to move. Accordingly, cars change lanes while keeping a safe distance with other cars. Line : 303 -317

Accelerate or Brake ?
Also if the lane can not be changed due close proximity to another car on the next  lane and there is a traffic vehicle in the same lane as the lane of the subject car, the car brakes to maintain an appropriate safe distance.Line : 264 -300

In most of the traffic conditions this code enables the car to successfully shift to the fastest lane.

Further the spline method is used to generate smooth path as recommended.
