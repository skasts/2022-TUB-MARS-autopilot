# Covariance Adder

This package contains of a node that adds Covariance information to a PoseStamped msgs. This 
information is needed in order to fuse several PoseStamped msgs together, i.e. using a Kalman filter
as in `robot localization`. The node has a parameter for the diagonal entries of the covariance 
matrix. The default value is 
```
[0.01, 0.01, 0.01, 0.01, 0.01, 0.1]
```
To run the node 
```
ros2 run covariance_adder covariance_adder
```