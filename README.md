# ADOS
Main code for my article: A Distributed Adaptive Route Planning Optimization Algorithm for UAV Swarm, is now under review. 

Before running, a toolbox should be installed: https://github.com/mattools/matGeom

Some of the code information is as follows: 
- `Drone.m`, `Obstacle.m`: they are class of UAVs and obstacles, including some properties and methods;
- `H.m`, `heaviside.m`: some mathematical functions;
- `insertsectCuboid.m`, `insertsectCylinder.m`: to determine whether the ray intersects with a box/cylinder and return the intersection point;
- `isInCuboid.m`, `isInCylinder.m`: to determine whether the drone is in an obstacle;
- `calculateF.m`: to calculate value F;
- `main_cuboid.m`, `main_cylinder.m`: are the main programs, the former has a cuboid wall, while the latter has many cylinders.
- `plot_H_exp.m`, `viewline.m`: some functions to plot details;
- `optimizer.m`,`main_optimize.m`: used to optimize parameters in the model.

More details are accessible by contacting me: qcxu0220@163.com
