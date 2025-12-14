# Unitree Go2 Simulator with Custom UKF Odometry

This repository contains the full ROS 2 Humble workspace source code for the final project submission, including the Unitree Go2 simulator package and the custom Unscented Kalman Filter (UKF) odometry package (`go2_ukf`).

**Repository Link:** [https://github.com/sutozan/Prob_Rob_Go2_FinalProject_2025.git](https://github.com/sutozan/Prob_Rob_Go2_FinalProject_2025.git)

## Step 1: System Setup and Build

These instructions assume you are running **Ubuntu 22.04 with ROS 2 Humble** installed.

1.  **Create Workspace and Clone the Repository:**
    *The workspace is named `go2_ws`.*
    ```bash
    mkdir -p ~/go2_ws/src
    cd ~/go2_ws/src
    git clone https://github.com/sutozan/Prob_Rob_Go2_FinalProject_2025.git
    cd ..
    ```

2.  **Install All Required Dependencies:**
    *This single command automatically installs ALL required packages (e.g., robot-localization, gazebo packages, and xacro) specified in the package manifest files.*
    ```bash
    sudo apt update
    sudo apt install ros-humble-rmw-cyclonedds-cpp -y
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the Workspace:**
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

## Step 2: Running the System

### A. Launch the Gazebo Simulator (Terminal 1)

This starts the simulated robot in the Gazebo environment. **To check if everything is working, run this command and verify Gazebo opens and the Go2 robot model loads.**

1.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```
2.  **Run the Launch File (No RVIZ):**
    ```bash
    ros2 launch go2_config gazebo.launch.py
    ```

## Step 3: Running the UKF Odometry.

### A. Launch the Gazebo Simulator (Terminal 1)

Follow the commands above to start the simulator above. Note: If the simulator is not on, overall code will not be producing measurments.

### B. Run Odometry Code (Terminal 2)
This code is to run both the filter and the odometry. Since the filter is defined as a class, running the code below correclty is crucial.

1.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```
2.  **Run the Launch File**
    ```bash
    ros2 run go2_ukf ukf_odometry
    ```

### C. Run Evaluator (Terminal 3)
To view the performance of the filter and all error messages, run the code below.

1.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```
2.  **Run the Launch File:**
    ```bash
    ros2 run go2_ukf evaluator
    ```

### D. Run Teleop (Terminal 4)
To test the filter performance during movement, utilize the teleop feature to move the robot around. We suggets choosing linear motion.

1.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```
2.  **Run the Teleop:**
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```










