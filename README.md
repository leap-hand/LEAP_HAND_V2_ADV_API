# LEAP Hand v2 Advanced API

Welcome to the API for **LEAP Hand v2 Advanced** – a state-of-the-art 17-DOF hybrid rigid-soft robotic hand, engineered for dexterity, adaptability, and ease of use. This advanced hand features a combination of 3D-printed soft and rigid structures, powered articulations in the palm, and a human-like MCP kinematic design.

Learn more about the hand, its assembly, and usage at our website:
👉 [https://v2-adv.leaphand.com/](https://v2-adv.leaphand.com/)

> ⚠️ Note: This is **not** the same as the standard LEAP Hand v2. That model is a lightweight, 8-DOF version designed for simplicity and lower cost (\~\$200).

---

## Getting Started

### Prerequisites

* **Operating System**: Ubuntu 22.04 (recommended)
* **ROS 2**: Humble Hawksbill (verified)
* **Python**: API is written entirely in Python

---

## Installation

1. Set up your ROS 2 workspace.
2. Copy the `leapv2` folder into your ROS 2 workspace (`src` directory).
3. Build the workspace:

```bash
colcon build --symlink-install
```

---

## Initial Setup: Motor Alignment

Before first use (or after replacing a tendon), run the motor alignment script:

```bash
python3 realign.py
```

This process:

* Uses current control mode to find fully open and closed positions.
* Saves calibration data to a CSV file:

  * First value: fully open position
  * Second value: fully closed position
* The order is in motor order.

> 💡 Tip: Re-run this occasionally if you notice inaccuracies or tendon slippage.

---

## ROS 2 Node Overview

This ROS 2 node manages LEAP Hand v2 Advanced via **position control** and accepts a 20-DOF pose array (e.g., from a glove or motion capture system).

### Pose Array Format

* **Finger Order**:
  `Index[0:4], Middle[4:8], Ring[8:12], Pinky[12:16], Thumb[16:20]`

* **Joint Order**:
    Joint order: 
    * MCP side[0,4,8,12,16]
    * MCP forward[1,5,9,13,(17 actually palm thumb forward)]
    * PIP[2,6,10,14,(18 actually MCP thumb forward)]
    * DIP[3,7,11,15,(19 actually thumb curl)]

### Joint Command Ranges

* **MCP Side**: `-1.57 to 1.57 rad` (0 is neutral)
* **MCP Forward**: `0 (open) to ~3 rad (closed)`
* **PIP/DIP**: `0 (uncurled) to 1.57 rad (fully curled)`

Note that you do not have independent control of PIP and DIP as they are coupled by a tendon.

The two articulated palm motor joints are controlled:
>
> * `palm_4_fingers` \~ max 1 rad and calculated from the 4 MCP forward joints of the Index-Pinky Finger
> * `palm_thumb` derived from MCP forward of thumb

---

## Direct Motor Commands

Instead of the scaled values, you can directly send a **17D vector** to control the motors:

### Motor Command Order

| Group          | MCP Side | MCP Forward | Curl |
| -------------- | -------- | ----------- | ---- |
| Index          | 0        | 1           | 2    |
| Middle         | 3        | 4           | 5    |
| Ring           | 6        | 7           | 8    |
| Pinky          | 9        | 10          | 11   |
| Thumb          | 12       | 13          | 14   |
| Palm (Thumb)   | —        | —           | 15   |
| Palm (Fingers) | —        | —           | 16   |

## Reading Joint Positions

You can access joint positions, velocities, and efforts using ROS 2 services. These services follow the same joint ordering used in motor commands.

Using services instead of topics reduces communication overhead and is preferred for efficiency, especially when frequent polling is not required.

---

## Launching the Hand

Use the provided launch script:

```bash
ros2 launch launch_leap.py
```

---
Although you can use this API without ROS 2 since it is all Python, using it within a ROS environment greatly simplifies, Teleoperation, Retargeting, Integration with learning-based control systems.

## Telekinesis Node (Fingertip IK)

To control the hand by specifying fingertip targets (e.g., using inverse kinematics), you can use the **Telekinesis Node**. This module optimizes grasping postures using a method similar to those described in the [Robotic Telekinesis](https://robotic-telekinesis.github.io/) and [Bimanual Dexterity](https://bidex-teleop.github.io/) papers. It is fully compatible with the [Bidex Glove API](https://github.com/leap-hand/Bidex_Manus_Teleop). In essence, the system loads the URDF into PyBullet, receives joint angles, applies SDLS inverse kinematics, and outputs the desired joint angles to guide the hand back to the target configuration. I recommend you check the [Bidex Glove API](https://github.com/leap-hand/Bidex_Manus_Teleop) for more details on how to use this and set it up with human videos such as from Vision Pro/MANO-like format or Manus Gloves.

## Resources

* **Website**: [https://v2-adv.leaphand.com/](https://v2-adv.leaphand.com/)
* **Documentation**: See comments in the code for detailed parameter info and customization guidance.

---

Feel free to reach out via [https://v2-adv.leaphand.com/](https://v2-adv.leaphand.com/) for more support or collaboration opportunities!

---
