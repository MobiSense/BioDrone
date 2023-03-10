# Demo Video for BioDrone

[![Watch the video](https://i.imgur.com/TIw235B.jpeg)](https://youtu.be/xxxx)

🚩 Code and data will be made publicly available before publication.

---

# Implementation Details
![](https://github.com/Event4BioDrone/BioDrone/blob/main/IMG/implementation.png)
The above figure presents the implementation of BioDrone on a drone platform for obstacle avoidance.
The arrows indicate data flow.

## Hardware

We deploy BioDrone on an AMOVLAB P450-NX drone testbed.
The drone is equipped with two on-board computational units:

* a **Qualcomm Snapdragon Flight**, which is leveraged for monocular visual-inertial odometry (VIO) based pose estimation using the provided Machine Vision SDK and receiving GPS signals.
* a **Xilinx Zynq-7020** chip or an **Nvidia Jetson TX2** (accompanied with an AUVIDEA J90 carrier board) runs our obstacle detection and localization software stack, as well as an avoidance command planning task.

The output of BioDrone is a low-level obstacle orientation and distance information, based on which a **ArduPilot Mega (APM)** filght controller will calculate and produce single-rotor commands and feed them to motor controllers.

The drone is equipped with two front-facing DAVIS-346 event cameras, in a horizontal stereo setup, connected via USB 3.0 micro to the Jetson TX2.
The DAVIS-346 sensor provides both frame and events and has a (346 $\times$ 260 pixels) QVGA resolution.
The horizontal and vertical FoV of DAVIS-346 is around 120$\circ$ and 100$\circ$, respectively.

## Software
