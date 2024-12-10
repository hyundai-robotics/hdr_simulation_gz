HD Hyundai Robotics Robots ROS2 GZ Simulation
==========================================
## Table of Contents
- [Introduction](#introduction)
- [Packages](#packages)
- [Installation](#installation)
- [Usage](#usage)
- [ROS2 Services List](#ros2-services-list)
- [Contact](#contact)

## Introduction
This project provides a ROS2 Gazebo simulation environment for HD Hyundai Robotics robots.

## Packages

| Package Name     | Description |
|------------------|-------------|
| `hdr_description`   | Package containing description files for HD Hyundai Robotics robots, including various configuration files and visual/physical parameter files for the models. |
| `hdr_moveit_config` | Contains MoveIt configuration files, including setup files for integration with MoveIt. |
| `hdr_simulation`   | Main folder of the HDR Simulation package, containing simulation control and related configuration files. |

# Installation

## Setup ROS2 Install

This guide provides instructions for downloading and executing the ros2-humble-desktop-main.sh script to install ROS2 Humble Desktop.

### Installation Steps

1. Download the ros2-humble-desktop-main.sh script from the package repository.
2. Open a terminal and navigate to the directory where you downloaded the script.
3. Make the script executable by running the following command:
    ```
    bash ros2-humble-desktop-main.sh
    ```

    > **NOTE:** A ~/ros2_ws/src folder will be created, and if the user desires, they can create and use a different folder without any issues.
4. Follow any on-screen prompts during the installation process.
5. Once the installation is complete, restart your system or source the ROS2 setup file
    ```
    source /opt/ros/humble/setup.bash
    ```
## Configure and Build Workspace
    
### 1. git clone package

  ```
  cd  ~/ros2_ws/src
  git clone https://github.com/hyundai-robotics/hdr_simulation_gz.git
  ```

### 2. Install Dependency Packages

#### 2.1 Install General Dependencies

Download the required repositories and install package dependencies:

```bash
cd ~/ros2_ws/
rosdep update && rosdep install --ignore-src --from-paths src -y
```

This command updates `rosdep` and automatically installs dependencies for all packages in the `src` folder.

#### 2.2 Install Project-Specific Dependencies

Run the project-specific dependency installation script:

```bash
cd ~/ros2_ws/src/hdr_simulation_gz
bash ros2-install-deps.sh
```

### 3. Configure and Install Workspace

Finally, configure and install the workspace:

```bash
cd ~/ros2_ws/src/hdr_simulation_gz
cp install ~/ros2_ws/
source ~/ros2_ws/install/setup.bash
```

Additionally, to use the simulation, you need to build the workspace using the following command:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

This `colcon build` command builds all packages in the workspace and installs them using symbolic links. This allows changes in source files to take effect immediately without requiring a rebuild.

After the build is complete, open a new terminal or run the following command in your current terminal to apply the updated environment:

```bash
source ~/ros2_ws/install/setup.bash
```

## Running Executable

1. **Source your workspace** 
```
source ~/ros2_ws/install/setup.bash
ros2 launch hdr_simulation hdr_sim_control.launch.py
```

2. **Example using MoveIt with simulated robot** 
```
ros2 launch hdr_simulation hdr_sim_moveit.launch.py
```

3. **Example of connecting to the robot controller** 
```
ros2 launch hdr_ros2_launch ros2_driver.launch.py
```

# Usage

## Supported Robots

The following robots can be simulated using the `hdr_sim_moveit.launch.py` file in the `hdr_simulation` package:

```
"ha006b", "hh7", "hh020", "hs220_02", "hx400"
```

> **NOTE:** You can change the simulated robot by modifying the `hdr_type` parameter in `hdr_sim_moveit_launch.py`. Currently, only `hs220_02` is enabled for simulation.

## Simulation and Control

After launching the HD Hyundai Robotics robot simulation, follow these steps to control the robot:

1. Switch the controller's TP to playback mode.
2. Copy the `.job` code provided in the package to the appropriate location.
3. Load the copied `.job` file on the TP.

Then, execute the following steps:

1. Turn the motor ON:
```
ros2 service call /hdr_ros2_driver/robot/post/motor_control std_srvs/srv/SetBool "{data: true}"
```

2. Start the program:
```
ros2 service call /hdr_ros2_driver/robot/post/robot_control std_srvs/srv/SetBool "{data: true}"
```

3. Set initialization pose:
This service adjusts the Gazebo position based on the actual robot's pose values:
```
ros2 service call /hdr_ros2_driver/inital_pose std_srvs/srv/Trigger
```


# ROS2 Service Lists

This document provides a comprehensive list of ROS2 services available in the HD Hyundai Robotics Robots ROS2 GZ Simulation project. The services are categorized by their functionality and include descriptions and usage examples.

## Table of Contents

- [Version Services](#version-services)
- [Project Services](#project-services)
- [Control Services](#control-services)
- [Robot Services](#robot-services)
- [I/O PLC Services](#io-plc-services)
- [Log Manager Services](#log-manager-services)
- [File Manager Services](#file-manager-services)
- [Task Services](#task-services)
- [Clock Services](#clock-services)
- [Console Services](#console-services)

For examples and detailed information on how to use these services, please refer to the [HI6 Open API documentation](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/README).

## Version Services

| Service Name | Service Type | Description | Example Usage | Documentation Link |
|--------------|--------------|-------------|---------------|--------------------|
| `/hdr_ros2_driver/get/api_ver` | `std_srvs::srv::Trigger` | Get Open API schema version | `ros2 service call /hdr_ros2_driver/get/api_ver std_srvs/srv/Trigger` | [API Schema Version](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/2-version/1-get/1-api_ver) |
| `/hdr_ros2_driver/get/sysver` | `std_srvs::srv::Trigger` | Get robot controller system software version | `ros2 service call /hdr_ros2_driver/get/sysver std_srvs/srv/Trigger` | [System Software Version](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/2-version/1-get/2-sysver) |

## Project Services

| Service Name | Service Type | Description | Example Usage | Documentation Link |
|--------------|--------------|-------------|---------------|--------------------|
| `/hdr_ros2_driver/project/get/rgen` | `std_srvs::srv::Trigger` | Read general information set in the controller | `ros2 service call /hdr_ros2_driver/project/get/rgen std_srvs/srv/Trigger` | [Read General Information](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/1-get/1-rgen) |
| `/hdr_ros2_driver/project/get/jobs_info` | `std_srvs::srv::Trigger` | Get information related to job programs | `ros2 service call /hdr_ros2_driver/project/get/jobs_info std_srvs/srv/Trigger` | [Get Job Information](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/1-get/2-jobs_info) |
| `/hdr_ros2_driver/project/post/reload_updated_jobs` | `std_srvs::srv::Trigger` | Request to update job files | `ros2 service call /hdr_ros2_driver/project/post/reload_updated_jobs std_srvs/srv/Trigger` | [Reload Updated Jobs](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/2-post/1-reload_updated_jobs) |
| `/hdr_ros2_driver/project/post/delete_job` | `hdr_msgs::srv::FilePath` | Request to delete a job file | `ros2 service call /hdr_ros2_driver/project/post/delete_job hdr_msgs/srv/FilePath "{path: '0001.job'}"` | [Delete Job](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/3-project/2-post/2-jobs-delete_job) |

## Control Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/control/get/op_cnd` | `std_srvs::srv::Trigger` | Get condition setting values | `ros2 service call /hdr_ros2_driver/control/get/op_cnd std_srvs/srv/Trigger` | [Get Condition Setting Values](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/1-op_cnd) |
| `/hdr_ros2_driver/control/get/ios_di` | `hdr_msgs::srv::IoRequest` | Get user IO values (Digital Input) | `ros2 service call /hdr_ros2_driver/control/get/ios_di hdr_msgs/srv/IoRequest "{type: 'di', blk_no: 1, sig_no: 1}"` | [Get User IO Values (Digital Input)](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/2-ios-dio) |
| `/hdr_ros2_driver/control/get/ios_do` | `hdr_msgs::srv::IoRequest` | Get user IO values (Digital Output) | `ros2 service call /hdr_ros2_driver/control/get/ios_do hdr_msgs/srv/IoRequest "{type: 'do', blk_no: 1, sig_no: 1}"` | [Get User IO Values (Digital Output)](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/2-ios-dio) |
| `/hdr_ros2_driver/control/get/ios_si` | `hdr_msgs::srv::IoRequest` | Get system IO values (System Input) | `ros2 service call /hdr_ros2_driver/control/get/ios_si hdr_msgs/srv/IoRequest "{type: 'si', blk_no: 1, sig_no: 1}"` | [Get System IO Values (System Input)](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/3-ios-sio) |
| `/hdr_ros2_driver/control/get/ios_so` | `hdr_msgs::srv::IoRequest` | Get system IO values (System Output) | `ros2 service call /hdr_ros2_driver/control/get/ios_so hdr_msgs/srv/IoRequest "{type: 'so', blk_no: 1, sig_no: 1}"` | [Get System IO Values (System Output)](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/3-ios-sio) |
| `/hdr_ros2_driver/control/get/ucs_nos` | `std_srvs::srv::Trigger` | Get list of currently used user coordinate systems | `ros2 service call /hdr_ros2_driver/control/get/ucs_nos std_srvs/srv/Trigger` | [Get List of Currently Used User Coordinate Systems](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/1-get/4-ucss-ucs_nos) |
| `/hdr_ros2_driver/control/post/ios_do` | `hdr_msgs::srv::IoRequest` | Change digital output | `ros2 service call /hdr_ros2_driver/control/post/ios_do hdr_msgs/srv/IoRequest "{type: 'do', blk_no: 1, sig_no: 1, val: 1}"` | [Change Digital Output](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/2-post/1-ios-dio) |
| `/hdr_ros2_driver/control/put/op_cnd` | `hdr_msgs::srv::OpCnd` | Change robot's condition setting values | `ros2 service call /hdr_ros2_driver/control/put/op_cnd hdr_msgs/srv/OpCnd "{playback_mode: 1, step_goback_max_spd: 130, ucrd_num: 2}"` | [Change Robot's Condition Setting Values](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/4-control/3-put/1-op_cnd) |

## Robot Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/robot/get/motor_state` | `std_srvs::srv::Trigger` | Get motor on state | `ros2 service call /hdr_ros2_driver/robot/get/motor_state std_srvs/srv/Trigger` | [Get Motor On State](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/1-motor_on_state) |
| `/hdr_ros2_driver/robot/get/po_cur` | `hdr_msgs::srv::PoseCur` | Get current robot pose | `ros2 service call /hdr_ros2_driver/robot/get/po_cur hdr_msgs/srv/PoseCur "{crd: 0, mechinfo: 1}"` | [Get Current Robot Pose](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/2-po_cur) |
| `/hdr_ros2_driver/robot/get/cur_tool_data` | `std_srvs::srv::Trigger` | Get current tool data | `ros2 service call /hdr_ros2_driver/robot/get/cur_tool_data std_srvs/srv/Trigger` | [Get Current Tool Data](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/3-cur_tool_data) |
| `/hdr_ros2_driver/robot/get/tools` | `std_srvs::srv::Trigger` | Get all tool information | `ros2 service call /hdr_ros2_driver/robot/get/tools std_srvs/srv/Trigger` | [Get All Tool Information](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/4-tools) |
| `/hdr_ros2_driver/robot/get/tools_t` | `hdr_msgs::srv::Number` | Get specific tool setting information | `ros2 service call /hdr_ros2_driver/robot/get/tools_t hdr_msgs/srv/Number "{data: 0}"` | [Get Specific Tool Setting Information](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/1-get/5-tools_t) |
| `/hdr_ros2_driver/robot/post/motor_control` | `std_srvs::srv::SetBool` | Turn robot motor ON/OFF | `ros2 service call /hdr_ros2_driver/robot/post/motor_control std_srvs/srv/SetBool "{data: true}"` | [Turn Robot Motor ON/OFF](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/1-motor-on-off) |
| `/hdr_ros2_driver/robot/post/robot_control` | `std_srvs::srv::SetBool` | Start/Stop robot | `ros2 service call /hdr_ros2_driver/robot/post/robot_control std_srvs/srv/SetBool "{data: true}"` | [Start/Stop Robot](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/2-start-stop) |
| `/hdr_ros2_driver/robot/post/tool_no` | `hdr_msgs::srv::Number` | Set current tool number | `ros2 service call /hdr_ros2_driver/robot/post/tool_no hdr_msgs/srv/Number "{data: 0}"` | [Set Current Tool Number](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/3-tool_no) |
| `/hdr_ros2_driver/robot/post/crd_sys` | `hdr_msgs::srv::Number` | Set current jog coordinate system | `ros2 service call /hdr_ros2_driver/robot/post/crd_sys hdr_msgs/srv/Number "{data: 0}"` | [Set Current Jog Coordinate System](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/5-robot/2-post/4-crd_sys) |
| `/hdr_ros2_driver/robot/post/emergency_stop` | `hdr_msgs::srv::Emergency` | Emergency stop operation | `ros2 service call /hdr_ros2_driver/robot/post/emergency_stop hdr_msgs/srv/Emergency "{step_no: 2, stop_at: 20, stop_at_corner: 0, category: 1}"` | [Emergency stop operation](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/korean/5-robot/2-post/5-emergency_stop) |

## I/O PLC Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/plc/get/relay_value` | `hdr_msgs::srv::IoplcGet` | Get relay values for the entire object type | `ros2 service call /hdr_ros2_driver/plc/get/relay_value hdr_msgs/srv/IoplcGet` | [Get Relay Values](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/1-get/1-relay-value) |
| `/hdr_ros2_driver/plc/post/relay_value` | `hdr_msgs::srv::IoplcPost` | Set relay values | `ros2 service call /hdr_ros2_driver/plc/post/relay_value hdr_msgs/srv/IoplcPost` | [Set Relay Values](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/6-io_plc/2-post/1-set_relay_value) |

## Log Manager Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/log/get/manager` | `hdr_msgs::srv::LogManager` | View event log with specified filter conditions | `ros2 service call /hdr_ros2_driver/log/get/manager hdr_msgs/srv/LogManager` | [View Event Log](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/7-log_manager/1-get/1-search) |

## File Manager Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/file/get/files` | `hdr_msgs::srv::FilePath` | Get file contents from controller | `ros2 service call /hdr_ros2_driver/file/get/files hdr_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` | [Get File Contents](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/1-files) |
| `/hdr_ros2_driver/file/get/file_info` | `hdr_msgs::srv::FilePath` | Get file information based on file path | `ros2 service call /hdr_ros2_driver/file/get/file_info hdr_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` | [Get File Information](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/2-file_info) |
| `/hdr_ros2_driver/file/get/file_list` | `hdr_msgs::srv::FileList` | Get list of files and directories | `ros2 service call /hdr_ros2_driver/file/get/file_list hdr_msgs/srv/FileList "{path: 'project/jobs', incl_file: true, incl_dir: false}"` | [Get List of Files and Directories](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/3-file_list) |
| `/hdr_ros2_driver/file/get/file_exist` | `hdr_msgs::srv::FilePath` | Check if target file exists | `ros2 service call /hdr_ros2_driver/file/get/file_exist hdr_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` | [Check if Target File Exists](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/1-get/4-file_exist) |
| `/hdr_ros2_driver/file/post/rename_file` | `hdr_msgs::srv::FileRename` | Rename target file | `ros2 service call /hdr_ros2_driver/file/post/rename_file hdr_msgs/srv/FileRename "{pathname_from: 'project/jobs/0001.job', pathname_to: 'project/jobs/4321.job'}"` | [Rename Target File](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/1-rename_file) |
| `/hdr_ros2_driver/file/post/mkdir` | `hdr_msgs::srv::FilePath` | Create directory at target path | `ros2 service call /hdr_ros2_driver/file/post/mkdir hdr_msgs/srv/FilePath "{path: 'project/jobs/special'}"` | [Create Directory at Target Path](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/2-mkdir) |
| `/hdr_ros2_driver/file/post/files` | `hdr_msgs::srv::FileSend` | Send file to target path | `ros2 service call /hdr_ros2_driver/file/post/files hdr_msgs::srv::FileSend "{target_file: 'project/jobs/test.job', source_file: '/home/test/test.job'}"` | [Send File to Target Path](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/2-post/3-files) |
| `/hdr_ros2_driver/file/delete/file` | `hdr_msgs::srv::FilePath` | Delete target file or directory | `ros2 service call /hdr_ros2_driver/file/delete/file hdr_msgs/srv/FilePath "{path: 'project/jobs/0001.job'}"` | [Delete Target File or Directory](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/8-file_manager/3-delete/1-files) |

## Task Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/task/post/cur_prog_cnt` | `hdr_msgs::srv::ProgramCnt` | Set current program counter of the task | `ros2 service call /hdr_ros2_driver/task/post/cur_prog_cnt hdr_msgs/srv/ProgramCnt "{pno: -1, sno: -1, fno: -1, ext_sel: 0}"` | [Set Current Program Counter](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/1-cur_prog_cnt) |
| `/hdr_ros2_driver/task/post/reset` | `std_srvs::srv::Trigger` | Reset all tasks | `ros2 service call /hdr_ros2_driver/task/post/reset std_srvs/srv/Trigger` | [Reset All Tasks](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/2-reset) |
| `/hdr_ros2_driver/task/post/reset_t` | `hdr_msgs::srv::Number` | Perform reset on a task | `ros2 service call /hdr_ros2_driver/task/post/reset_t hdr_msgs/srv/Number "{data: 0}"` | [Perform Reset on a Task](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/2-reset) |
| `/hdr_ros2_driver/task/post/assign_var` | `hdr_msgs::srv::ProgramVar` | Reassign variable of current task statement | `ros2 service call /hdr_ros2_driver/task/post/assign_var hdr_msgs/srv/ProgramVar "{name: 'a', scope: 'local', expr: '14 + 2', save: 'true'}"` | [Reassign Variable of Current Task Statement (Expression)](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/3-assign_var_expr), [Reassign Variable of Current Task Statement (JSON)](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/4-assign_var_json) |
| `/hdr_ros2_driver/task/post/release_wait` | `std_srvs::srv::Trigger` | Release statement stop | `ros2 service call /hdr_ros2_driver/task/post/release_wait std_srvs/srv/Trigger` | [Release Statement Stop](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/5-release_wait) |
| `/hdr_ros2_driver/task/post/set_cur_pc_idx` | `hdr_msgs::srv::Number` | Position current cursor at index line | `ros2 service call /hdr_ros2_driver/task/post/set_cur_pc_idx hdr_msgs/srv/Number "{data: 0}"` | [Position Current Cursor at Index Line](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/6-set_cur_pc_idx) |
| `/hdr_ros2_driver/task/post/solve_expr` | `hdr_msgs::srv::ProgramVar` | Solve expression and set result to task's local or global variable | `ros2 service call /hdr_ros2_driver/task/post/solve_expr hdr_msgs/srv/ProgramVar "{name: 'a', scope: 'local'}"` | [Solve Expression and Set Result](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/9-task/2-post/7-solve_expr) |
| `/hdr_ros2_driver/task/post/execute_move` | `hdr_msgs::srv::ExecuteMove` | Moves to a specified pose. | `ros2 service call /hdr_ros2_driver/task/post/execute_move hdr_msgs/srv/ExecuteMove "{task_no: 0, stmt: 'move SP,spd=1sec,accu=0,tool=1 [0, 90, 0, 0, 0, 0]' }"` | [Moves to a specified pose.](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/korean/9-task/2-post/8-execute_move) |

## Clock Services

| Service Name | Service Type | Description | Example Usage | Documentation |
|--------------|--------------|-------------|---------------|----------------|
| `/hdr_ros2_driver/clock/get/date_time` | `std_srvs::srv::Trigger` | Get set system time | `ros2 service call /hdr_ros2_driver/clock/get/date_time std_srvs/srv/Trigger` | [Get System Time](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/11-etc/1-clock/1-get/1-date_time) |
| `/hdr_ros2_driver/clock/put/date_time` | `hdr_msgs::srv::DateTime` | Change system time | `ros2 service call /hdr_ros2_driver/clock/put/date_time hdr_msgs/srv/DateTime "{year: 2024, mon: 7, day: 11, hour: 15, min: 13, sec: 0}"` | [Change System Time](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/english/11-etc/1-clock/2-put/1-date_time) |



# Hi6 Controller Console Command Execution Service

This README provides information about the ROS2 service for executing console commands on the Hi6 controller.

## Service Information

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/hdr_ros2_driver/console/post/execute_cmd` | `hdr_msgs::srv::ExecuteCmd` | Execute console commands on the Hi6 controller. |

## Usage Example

To call the service from the command line, use the following ROS2 command:

```bash
ros2 service call /hdr_ros2_driver/console/post/execute_cmd hdr_msgs/srv/ExecuteCmd "{
cmd_line: [
'rl.stop',
'rl.reinit',
'rl.i move P,spd=100mm/sec,accu=0,tool=0  [10, 90, 20, 0, 0, 0]',
'rl.i move P,spd=200mm/sec,accu=1,tool=0  [-10, 90, -20, 0, 0, 0]',
'rl.i move P,spd=300mm/sec,accu=2,tool=0  [10, 90, 20, 0, 0, 0]',
'rl.i move P,spd=400mm/sec,accu=3,tool=0  [-10, 90, -20, 0, 0, 0]',
'rl.i move P,spd=500mm/sec,accu=4,tool=0  [10, 90, 20, 0, 0, 0]',
'rl.i move P,spd=600mm/sec,accu=7,tool=0  [10, 90, -20, 0, 0, 0]',
'rl.i end',
'rl.start'
]}"
```

This example demonstrates how to:
1. Stop the robot
2. Reinitialize the robot
3. Move the robot to various positions with different speeds and accuracies
4. End the immediate command sequence
5. Start the robot

## Safety Warning

Before executing any commands, ensure that you have taken all necessary safety precautions. Be aware of the robot's current state and its surrounding environment. Sudden movements of the robot can be dangerous if proper safety measures are not in place.

## Additional Documentation

For more detailed information about executing console commands on the Hi6 controller, please refer to the [official documentation](https://hrbook-hrc.web.app/#/view/doc-hi6-open-api/korean/10-console/2-post/1-execute_cmd).

## Contact

For any queries or support, please contact vewry12@hd.com
