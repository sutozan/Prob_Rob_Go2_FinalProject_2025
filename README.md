# Unitree Go2 Simulator with Custom UKF Odometry

This repository contains the full ROS 2 Humble workspace source code for the final project submission, including the Unitree Go2 simulator package and the custom Unscented Kalman Filter (UKF) odometry package (`go2_ukf`).

**Repository Link:** [https://github.com/sutozan/Prob_Rob_Go2_FinalProject_2025.git](https://github.com/sutozan/Prob_Rob_Go2_FinalProject_2025.git)

## 🛠️ Step 1: System Setup and Build

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
    # Install the preferred DDS middleware for Unitree/ROS2 communication
    sudo apt install ros-humble-rmw-cyclonedds-cpp -y
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build the Workspace:**
    ```bash
    colcon build --symlink-install
    . <your_ws>/install/setup.bash
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







