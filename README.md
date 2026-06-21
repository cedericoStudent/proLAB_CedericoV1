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


Folder "src": Here are the following cpp-nodes:
ekf_landmark: The Extended Kalman Filter with Landmark-detection.
evaluation_node: The node to calculate the RMSE between the trajectories given by all filters and the ground truth. Also to calculate the euclidean distance of the endpoints (Ground truth vs. filters). Output is in terminal
extended_kalman_filter_node: EKF without Landmark-detection.
joint_noise_node: To have more realistic conditions, gaussian noise was added to the encoders of the wheels in this node.
kalman_filter_node: Normal KF.
landmark_test_node: With this node, the landmark detection was tested.
multi_recorder_node: Recording all the trajectory points estimated by all 4 filters as well as the ground truth from "odom".
noisy_odometry_node: This calculates a noisy odometry based on the noise joint_states provided by the joint_noise_node. This publishes /odom_noisy.
particle_filter_node: PF.
recorder_node: This node was used to test the recording of the trajectories, also to test the plotting of them.
test_node: Was used for early testing since I don't have much experience with ROS2.
trajectory_generator_node: This controls the turtlebot. There's no closed loop trajectory/position control or something like that, but the velocity inputs to the turtlebot are implemented here. This ensures that the input and therefore the motion command stays constant throughout evaluation processes.


Folder "trajectories": At first, I wanted to store the trajectories there. Turned out this was a stupid idea with the tremendous amount of tests. Therefore, the trajectories get (over)written in your install-folder! You can ignore this folder.


Folder "worlds": Here I created and experimented with different types of gazebo-worlds:
cedric_landmark_world.world: This is my implemented world with the four walls and the original three cylinders. It is in the .world-format which doesn't work that well in ROS2 Foxy when it comes to including the turtlebot.
my_world.world: Early experiments, trying to create an empty world. It is in the .world-format which doesn't work that well in ROS2 Foxy when it comes to including the turtlebot.
waffel.model: That's it, that's my world with the four walls and only one reliably detectable landmark. Also, this is finally capable of spawning the turtlebot.


Other Files:
plot_path.py: This creates a plot of the trajectory with its covariance-ellipses estimated by one filter and the ground truth. Used for testing implementations of the filters.
plotter_multi_comparison.py: This plots all the trajectories with its covariance-ellipses estimated by all 4 filters as well as the ground truth.
plotter_pf_average.py: This plots a rolling average of the poses estimated by the PF. This was used at an early stage to reduse visible jittering in the estimated pose.
