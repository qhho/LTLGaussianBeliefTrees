
# Running the Visualization with RViz and ROS2

Follow these steps to set up and run your visualization using RViz in ROS2.

### **Install The GBT Library**
1. Open a **new terminal window**.
2. Navigate to the `LTLGaussianBeliefTrees/build` directory.
    
    ```bash
    cmake ..
    make
    sudo make install
    ```

### **Running The Visualizer**

1. Open a **new terminal window**.
2. Navigate to the `LTLGaussianBeliefTrees/ros_ws` directory:

    ```bash
    colcon build
    source /opt/ros/foxy/setup.bash
    source install/local_setup.bash
    ros2 launch agent_bringup agent_gbt.launch.py
    ```