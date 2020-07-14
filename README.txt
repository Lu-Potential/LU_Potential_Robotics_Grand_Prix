LU_Potential_Robotics_Grand_Prix
LU_Potential_Robotics final submission for the Berlin Grand Prix of IFAC 2020 of F1tenth race car competition by Team LU Potential of Lehigh University, Bethlehem, PA, USA. 

Team members: Xiyuan Zhu, Andrew Charway, and Yazhou Li.
Team Advisor: Rosa Zheng

The submission contains five files including this README.txt.

Assume that the host computer has installed Ubuntu 18.04, ROS Melodic, and Docker, and the July 2020 version of f1tenth_gym_ros. To run the race in a separate terminal:


1. Git clone the LU-Potential_Robotics_Grand_Prix package from https://github.com/Lu-Potential/LU_Potential_Robotics_Grand_Prix.git. Copy  
 Lehigh_Ver_4.py into the subfolder /src/f1tenth_gym_ros/scripts/ of your catkin workspace.

2. Make sure that Lehigh_Ver_4.py is executable.

3. Create a path in the Home folder called "~/rcws/logs/" and copy wp-berlin-max-2.csv, wp-berlin-max-3.csv, wp-smooth-skeleton.csv files into the '~/rcws/logs' folder.

4. Make sure in the params.yaml file, the odom, opp_odom, drive and scan topics are prefixed with "/lu/" which is our unique_team_id.

5. Under the catkin workspace, source devel/setup.bash, and run:
     rosrun f1tenth_gym_ros Lehigh_Ver_4.py
