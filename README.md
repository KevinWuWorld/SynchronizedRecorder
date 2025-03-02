# SynchronizedRecorder
An ROS node that synchronizes and records images and kinematic data. The synchronized data pairs are saved in organized folders with timestamps as folder names. Each folder contains:
1. A synchronized image (image.png)
2. Corresponding kinematic data in JSON format (kinematics.json)

## File Structure
- Source Code: synchronized_recorder_node.cpp
- Output Directory: recorded_data/ (generated automatically in the working directory).

## Program Parameters
Topics: Update the topic names in the main() function to match the current daVinci setup.
Buffer Size: Adjust MAX_BUFFER_SIZE to control the size of data buffers.
Time Tolerance: Adjust TIME_TOL to specify synchronization accuracy.

## Build & Run Package
1. Clone repository
```
git clone <repository_url>
cd <repository_directory>
```
2. Build package
Ensure your CMakeLists.txt includes the necessary dependencies (ROS, OpenCV, JsonCpp). If something's missing, you'll know from error msgs during build.
```
catkin build --summary
source devel/setup.bash
```

3. Run node
Ensure that you've started all necessary processes (roscore, stereo camera, lighting if necessary).
Here'a full list of all dependencies necessary for this node:
  - Stereo camera. Start this using the following command:
    ```
    roslaunch dvrk_video decklink_stereo_1280x1024.launch stereo_rig_name:=test
    ```
    Important: stereo_rig_name must be set to "test"!!!
    
  - roscore

  - dvrk control console turned on.
    First, navigate to this folder: username@lcsr-dvrk-xx: ~/catkin_ws/src/dvrk/dvrk_config_jhu/jhu-daVinci
    Start this using the following command:
    ```
    rosrun dvrk_robot dvrk_console_json -j console-PSM1.json -p 0.001
    ```
    Important: verify that the dvrk is in the powered on status in the console!
    
  - Verify there's output kinetic data, using this command:
    ```
    rostopic echo /PSM1/measured_js
    ```


## How this works
### Threads
- ROS Spin Thread: Handles ROS callbacks for receiving data.
- Synchronization Thread: Matches image and kinematic data based on timestamps.
- Image Writer Thread: Writes synchronized images to disk.
- Kinematic Writer Thread: Serializes and writes kinematic data in JSON format.
### Workflow
- Callbacks: Subscribe to ROS topics for image and kinematic data.
- Synchronization: Match data with a timestamp tolerance of 1ms.
- Output: Save synchronized data to local storage in timestamped folders.
- Cleanup: Remove incomplete folders to ensure data consistency.

## (Automatic) Double Checking Completeness of Data Pairs
If a folder is incomplete (e.g., missing image.png or kinematics.json), the folder would be removed during cleanup.

