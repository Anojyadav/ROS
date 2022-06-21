# Move Base

Creating a Ros node to send goals to Move Base using action client. Also, Visualize goal in rviz through Marker 

Ros version : Melodic

The Navigation Stack serves to drive a robot(mobile base) from one location to another while safely avoiding obstacles.

Pre-Requisites: 
1. Ros Melodic
2. Simualtion environment in gazebo
3. map of the environment (should be seen in rviz)

For Localization and navigation we will use SLAM

Note: Goals in the script are choosed according to environment used. You can also create another pkg to publish goals and write a Subscriber to subscribe to the goal points.  
