# proLAB_CedericoV1
This is the repo of Cedric's implementation of the PRO-LAB project.

First things first: This project was implemented on ROS2 Foxy with natively installed Ubuntu 20.04 Focal Fossa. Due to that, the Turtlebot3 Waffle was used instead of the Turtlebot4, so don't be confused by the name of the package.

It's basically a normal ros2_ws, nothing special about the structure. In the ws, there's the package "tb4_cedric_pkg_1". Let's go through its folders and files.


Folder "doku": Excel sheets for turning the evaluation data into graphs.

Folder "evaluation": Pictures of the plotted trajectories with different noise values. The names of the pictures can be read as the following: qXkx--rXkx.png --> this means that the Q_factor and the R_factor have a value of X.x, therefore the "k" just stands for "KOMMA" or ".".
E.g.: q0k01--r1k0 means Q_factor = 0.01 and R_Factor = 1.0
These factors are used for adjusting the actual value of the noise, as it can be read in the paper.

Folder "launch": Here are the following launchfiles: all_filters_comparison, my_turtlebot3_world, simulation; Let's explain these.
all_filters_comparison: This launchfile starts a self-made gazebo world with the tb3 waffle, puts noise on the odometry which is used by the filters, executes an autonomous driving script, activates all 4 filters (KF, EKF, EKF-LM, PF) and starts the recorder node. This was used to get to the evaluation of the filters depending on noise.
my_turtlebot3_world: This is a "middle-launchfile" which is executed to spawn the tb3 in my own world correctly. I didn't want to change the original launchfile of the gazebo-turtlebot-world.
simulation: With this launchfile, the filters were tested seperately during coding them.

