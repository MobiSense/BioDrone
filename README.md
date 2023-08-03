✅ **Code and data will be made publicly available before publication.**

# Demo Video of BioDrone

[![Watch the video](https://github.com/Event4Drone/BioDrone/blob/main/IMG/teaser.png)](https://youtu.be/Wt1sbAhmx9g)

---

# Event Camera Preliminary
![](https://github.com/Event4Drone/BioDrone/blob/main/IMG/event-principle.png)
(💡A brief introduction to event camera has already been provided in our submission)

Event camera, a cutting-edge biosensor, operates via a unique mechanism distinct from traditional frame-based cameras. It boasts intelligent pixels that bear resemblance to the photoreceptor cells found in biological retinas, each capable of triggering events independently. 
Unlike conventional cameras that capture images at predetermined intervals, this advanced sensor detects per-pixel luminosity alterations asynchronously, producing a continuous stream of events with a resolution of *microsecond*.

The working principle of an event camera is illustrated in the above figure (left). 
As seen, let $t_{k-1}$ be the last time when an event fired at a pixel location $\boldsymbol{x} = (u, v)$, and $I_{k-1} = I(\boldsymbol{x}, t_{k-1})$ be the intensity level at such pixel at time $t_{k-1}$. 
A new event will be fired at the same pixel location $\boldsymbol{x}$ at time $t_k$ once the difference between the intensity $I_{k-1}$ and $I_k$ is larger than a pre-defined threshold $C > 0$. 
In other words, a new event $e_k = (\boldsymbol{x}, t_k, p_k)$ will be generated if $\Vert I(\boldsymbol{x}, t_{k}) - I(\boldsymbol{x}, t_{k-1}) \Vert \ge C$ (positive event, $p_k=1$) or $\Vert I(\boldsymbol{x}, t_{k}) - I(\boldsymbol{x}, t_{k-1}) \Vert \le -C$ (negative event, $p_k=-1$).

The above figure (right) provides a comparative analysis between an event camera and a frame-based camera. Evidently, the frame-based camera captures frames at a consistent rate, resulting in apparent motion blur. In contrast, the event camera operates continuously, producing a spatiotemporal spiral of polarity changes in brightness.



# Implementation Details
![](https://github.com/Event4Drone/BioDrone/blob/main/IMG/implementation.png)
The above figure presents the implementation of BioDrone on a drone platform for obstacle avoidance.
The arrows indicate data flow.

## Hardware

We deploy BioDrone on an AMOVLAB P450-NX drone testbed.
The drone is equipped with two on-board computational units:

* a [Qualcomm Snapdragon Flight](https://developer.qualcomm.com/hardware/qualcomm-flight-rb5), which is leveraged for monocular visual-inertial odometry (VIO) based pose estimation using the provided Machine Vision SDK and received GPS signals.
* a [Xilinx Zynq-7020](https://www.xilinx.com/products/boards-and-kits/1-571ww1.html) chip or an [Nvidia Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2) (accompanied with an AUVIDEA J90 carrier board) runs our obstacle detection and localization software stack, as well as an avoidance command planning task.

The output of BioDrone is a low-level obstacle orientation and distance information, based on which a [ArduPilot Mega 2.6](https://www.ardupilot.co.uk/) (APM) filght controller will calculate and produce single-rotor commands and feed them to motor controllers.

The drone is equipped with two front-facing [DAVIS-346](https://inivation.com/wp-content/uploads/2019/08/DAVIS346.pdf) event cameras, in a horizontal stereo setup, connected via USB 3.0 micro to the Jetson TX2.
The DAVIS-346 sensor provides both frame and events and has a (346 $\times$ 260 pixels) QVGA resolution.
The horizontal and vertical FoV of DAVIS-346 is around $120^\circ$ and $100^\circ$, respectively.

## Software
We develop the software stack in C++ and leverage Robot Operating System ([ROS Melodic](http://wiki.ros.org/melodic)) for communication among different modules.
We use the open source [event camera driver](https://github.com/uzh-rpg/rpg_dvs_ros) to stream events from camera to Jetson TX2, and the open source [avoidance algorithm](https://github.com/hku-mars/dyn_small_obs_avoidance) to plan avoidance commands based on obstacle's location.
To reduce latency, we implemented the obstacle localization and avoidance algorithms within the same ROS module, so that no message exchange is necessary between the camera drivers and the position controller.
The output of the module is a velocity command, which is then fed to the APM flight controller.
And the APM translates it to low-level motor commands using [PID-like algorithm](https://github.com/prgumd/EVDodgeNet) and finally communicates with the electronic speed controllers to generate the single rotor thrusts. 

---
# 1. Installation
We have tested BioDrone under Ubuntu 18.04 with ROS Melodic, using DAVIS346. We recommend using a powerful computer to ensure the performance.

## 1.1 Installation of Dependencies
- [ROS](http://wiki.ros.org/melodic): operation platform for BioDrone
- [rpg_dvs_ros](https://github.com/gabime/spdlog): camera driver and utilities
- [Eigen3](https://gitlab.com/libeigen/eigen): camera-related transformation and computation
- OpenCV (3.4.16 tested)

## 1.2 Build BioDrone
1. Prepare a catkin workspace
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. Clone the repository to the `src` folder in the workspace
```shell
git clone https://github.com/Event4Drone/BioDrone
```

3. Build the project
```shell
cd ~/catkin_ws
catkin_make
```

## 1.3 Configure BioDrone with your camera
BioDrone needs your own configuration of camera intrinsics and transformation matrices.
1. First locate to the folder of `CameraConfig.h` and open it.
```shell
cd BioDrone/include/BioDrone
vim CameraConfig.h
```
2. Configure your camera. Locate the following constants within the config file, and replace them with your camera setup:
- `const Eigen::Matrix3f Cam::K`: camera intrinsic matrix
- `const Eigen::Matrix3f Cam::T_ci`: transformation matrix from the IMU reference system to the camera reference system.
- `const double Cam::baseline`: the baseline distance for the stereo cameras.

3. Recompile the code to save configuration.
```shell
cd ~/catkin_ws
catkin_make
```

# 2. Dataset Acquisition
BioDrone subscribes to following ROS topics to receive data, with a publish rate of 1,000 Hz:
- `/davis/left/events`: The event stream generated from the left camera.
- `/davis/right/events`: The event stream generated from the right camera.
- `/davis/left/imu`: The IMU data stream generated from the left camera.
- `/davis/left/imu`: The IMU data stream generated from the right camera.

You can use ROS utilities(e.g. [rosbag record](https://wiki.ros.org/rosbag/Commandline#record)) record a dataset that contains above data. 

If the topic names in the recorded dataset don't correspond with above topic names, or the publish rate is not 1,000 Hz, you can manually correct them following [this documentation](https://github.com/HKUST-Aerial-Robotics/ESVO/tree/master/events_repacking_helper).

# 3. Usage
BioDrone currently supports dataset playback under ROS platform.
1. Check and verify dataset information.
```shell
rosbag info your_bag_file.bag
```
Executing above commandline instruction will display the timestamps, topics and number of collected data, so you can verify if they conform to above requirements. Also, note down the beginning timestamp of the bag file, which will be needed as an argument later.

2. Start ROS core and BioDrone.
```shell
roscore
rosrun BioDrone main TIMESTAMP
```
Replace `TIMESTAMP` with the number you've just noted down. You might need to execute above instructions in two separate shells.

3. Dataset playback. In another separate shell, execute:
```
rosbag play your_bag_file.bag
```
This will cause BioDrone to receive data from this bag file and start computation. The localization results will output to shell.
