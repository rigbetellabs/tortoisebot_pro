

# Tortoisebot-Pro ROS2 Humble Release

# ![TortoiseBot Banner](https://github.com/rigbetellabs/tortoisebot_docs/raw/master/imgs/packaging/pack_front.png)

![stars](https://img.shields.io/github/stars/rigbetellabs/tortoisebot?style=for-the-badge)
![forks](https://img.shields.io/github/forks/rigbetellabs/tortoisebot?style=for-the-badge)
![watchers](https://img.shields.io/github/watchers/rigbetellabs/tortoisebot?style=for-the-badge)
![repo-size](https://img.shields.io/github/repo-size/rigbetellabs/tortoisebot?style=for-the-badge)
![contributors](https://img.shields.io/github/contributors/rigbetellabs/tortoisebot?style=for-the-badge)

---
<p align="center"><a href="#connect-with-us-">Connect with Us</a> • <a href="#1-installation">Installation</a> 

<h1 align="center"> TortoiseBot-Pro </h1>

# Connect with us ![some-changes](https://img.shields.io/badge/some_changes-yellow)

<a href="https://rigbetellabs.com/">![Website](https://img.shields.io/website?down_color=lightgrey&down_message=offline&label=Rigbetellabs%20Website&style=for-the-badge&up_color=green&up_message=online&url=https%3A%2F%2Frigbetellabs.com%2F)</a>
<a href="https://rigbetellabs.com/discord">![Discord Channel](https://img.shields.io/discord/890669104330063903?logo=Discord&style=for-the-badge)</a>
<a href="https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA">![Youtube Subscribers](https://img.shields.io/youtube/channel/subscribers/UCfIX89y8OvDIbEFZAAciHEA?label=YT%20Subscribers&style=for-the-badge)</a>
<a href="https://www.instagram.com/rigbetellabs/">![Instagram](https://img.shields.io/badge/Follow_on-Instagram-pink?style=for-the-badge&logo=appveyor?label=Instagram)</a>

<h1 align="center"> TortoiseBot-Pro </h1>

<details open="open">
  <summary>Table of Contents</summary>
<ol>
    <li><a href="#1-installation">Installation</a>
    </li>
    <li><a href="#2-connection">Connection</a>
    </li>
    <li><a href="#3-package-description">Package Description</a>
        <ol>
            <li><a href="#31-tortoisebotpro_bringup">tortoisebotpro_bringup</a>
            </li>
            <li><a href="#32-tortoisebotpro_description">tortoisebotpro_description</a>
            </li>
            <li><a href="#33-tortoisebotpro_firmware">tortoisebotpro_firmware</a>
            </li>
            <li><a href="#34-tortoisebotpro_gazebo">tortoisebotpro_gazebo</a>
            </li>
            <li><a href="#35-tortoisebotpro_navigation">tortoisebotpro_navigation</a>
            <li><a href="#37-tortoisebotpro_slam">tortoisebotpro_slam</a>
            </li>
            <li><a href="#38-installsh">install.sh</a></li>
        </ol>
    </li>
    <li><a href="#4-launch-sequence">Launch Sequence</a>
    <ol>
        <li><a href="#41-gazebo-simulation">Gazebo Simulation</a></li>
            <ol>
                <li><a href="#411-map-generation">Map Generation</a></li>
                <li><a href="#412-autonomous-navigation-in-the-saved-map">Autonomous Navigation in the saved map</a></li>
                <li><a href="#413-slam">SLAM</a></li>
            </ol>
        <li><a href="#42-actual-robot">Actual Robot</a></li>
            <ol>
                <li><a href="#421-map-generation">Map Generation</a></li>
                <li><a href="#422-autonomous-navigation-in-the-saved-map">Autonomous Navigation in the saved map</a></li>
                <li><a href="#423-slam">SLAM</a></li>
            </ol>
     </ol>
    </li>
        <li><a href="#5-general-robot-information">General Robot Information</a>
        <ol>
            <li><a href="#51-topic-description">Topic Description</a></li>
            <li><a href="#52-battery">Battery</a></li>
            <li><a href="#53-robot-velocities">Robot Velocities</a></li>
            <li><a href="#54-usb-ports">USB Ports</a></li>
        </ol>
    </li>
        </li>
        <li><a href="#⚠️-6-usb-port-configuration-for-esp-and-lidar">USB Port Configuration for ESP and LiDAR</a></li>
    </li>
</ol>
</details>


## 1. Installation

Clone the repository into your workspace,

```py
cd ~/ros2_ws/src # Assuming ros2_ws is the name of the workspace
git clone https://github.com/rigbetellabs/tortoisebot_pro.git
```

Build the workspace,
```py
cd ~/ros2_ws/
colcon build
```

Installation of dependent packages,
```py
cd ~/ros2_ws/src/
cat requirements.txt | xargs sudo apt-get install -y 
# This installs all the packages mentioned in the requirements.txt
```

> [!NOTE]
> Check if you already have the lidar packages installed; if not, get the packages from repos below.

```py
cd ~/ros2_ws/src/
git clone https://github.com/rigbetellabs/ydlidar_ros.git
```

## 2. Connection

Wifi Setup?

To start any operation within the robot we need to SSH into and then perform operations. 

> [!NOTE]
> After switching the robot ON the Computing device takes a minute or so to boot up, wait for a while and then SSH into the robot using the below credentials.

```py
ssh "your-robot-name"@"your-robot-ip"
```

> [!NOTE]
> Please refer the robot for `robot_name` and the login password.
> Verify the IP that gets assigned to the robot via your network manager.

If you do not want to recheck if robot is connected to the network now or then you can utilize the `connect_tortoisebotpro.sh` script.

```py
./connect_tortoisebotpro.sh "username" "robot-ip"
```

The scripts scan the local network you are connected to and initiates a SSH connection if the robot is connected, the process continues until the robot is connected.

Successful execution looks something like this.

<p align="center">
	<img src="images/connect.png" width="900"/>
</p>


## 3. Package Description

### 3.1 [tortoisebotpro\_bringup](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_bringup/)

Provides a unified launch file to bring up the entire TortoiseBot Pro system including simulation, URDF, sensors, and navigation stack.

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Nodes Launched</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_bringup/launch/autobringup.launch.py">autobringup.launch.py</a></td>
            <td>
                Launches the complete robot system including state publisher, Gazebo (for simulation), camera node, micro-ROS agent, Cartographer or Nav2 (based on argument), depending on mode selected.<br><br>
                <b>Arguments:</b><br>
                <code>use_sim_time</code>: Set to <code>True</code> for simulation, <code>False</code> for real robot.<br>
                <code>exploration</code>: Set to <code>True</code> to run SLAM (Cartographer), <code>False</code> to use pre-saved map with Nav2.
            </td>
            <td>
                <code>/robot_state_publisher</code>,<br>
                <code>/joint_state_publisher</code>,<br>
                <code>gazebo</code>,<br>
                <code>camera_controller</code>,<br>
                <code>micro_ros_agent</code>,<br>
                <code>nav2_bringup</code> or <code>cartographer</code>
            </td>
        </tr>
    </tbody>
</table>

**Example Commands:**

* **Simulation + SLAM (exploration):**

  ```bash
  ros2 launch tortoisebotpro_bringup autobringup.launch.py use_sim_time:=True exploration:=True
  ```

* **Simulation + Navigation (with saved map):**

  ```bash
  ros2 launch tortoisebotpro_bringup autobringup.launch.py use_sim_time:=True exploration:=False
  ```

* **Real Robot + SLAM (exploration):**

  ```bash
  ros2 launch tortoisebotpro_bringup autobringup.launch.py use_sim_time:=False exploration:=True
  ```

* **Real Robot + Navigation (with saved map):**

  ```bash
  ros2 launch tortoisebotpro_bringup autobringup.launch.py use_sim_time:=False exploration:=False
  ```

**To save a map after SLAM:**

```bash
ros2 run nav2_map_server map_saver_cli -f /home/<your-nuc-user>/ros2_ws/src/tortoisebot_pro-ros2/tortoisebotpro_navigation/maps/test1
```

> Replace `<your-nuc-user>` with your actual robot's NUC username in the path.


### 3.2 [tortoisebotpro_description](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_description/)

Holds the robot description including `urdf`, `stl`

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Nodes Launched</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_description/launch/state_publisher.launch.py">state_publisher.launch.py</a></td>
            <td>Starts the publishign of the robot urdf on the topic /robot_description.</td>
            <td>
                    <code>/robot_state_publisher</code>,
                    <code>/joint_state_publisher</code>
                    </td>
        </tr>
    </tbody>
</table>


### 3.3 [tortoisebotpro_firmware](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_firmware/)

As the name suggest get all the sensor and actuation topics available to you
<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Nodes Launched</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_firmware/launch/micro_ros.launch.py">micro_ros.launch.py</a></td>
            <td>Launches Robot state publishers, serial node for communication with ESP32.</td>
            <td>
                <code>/cmd_vel</code>, <code>/wheels_ticks</code>, <code>/imu_data</code>
            </td>
    </tbody>
</table>

### 3.4 [tortoisebotpro_gazebo](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_gazebo/)

Simulation environment for tortoisebotpro in Gazebo

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Nodes Launched</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_gazebo/launch/gazebo.launch.py">gazebo.launch.py</a></td>
            <td>Launches gazebo basic world.</td>
            <td><code>/spawn_urdf</code>, <code>/gazebo</code></td>
        </tr>
    </tbody>
</table>

### 3.5 [tortoisebotpro_navigation](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_navigation/)

Autonomous navigation of robot using `move_base` in a know as well as unknown environment

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Node Launched</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_navigation/launch/navigation.launch.py">navigation.launch.py</a></td>
            <td>Launches the nav2 stack to navigate the robot, and based on exploration parameter it launches with saved map and without saved map  .</td>
            <td>
                <code>/nav2</code>
            </td>
        </tr>
    </tbody>
</table>


<!-- ### 3.6 [tortoisebotpro_odometry](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_odometry/)

How will the robot know where it is in the environment? Well it generates its own odometry for the purpose.

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Nodes Launched</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_odometry/launch/tortoisebotpro_icp_odom.launch">tortoisebotpro_icp_odom.launch</a></td>
            <td>Produces odometry data for the robot using Lidar and IMU.</td>
            <td>
                        <code>/icp_odometry</code>
                        <code>/ekf_localization_node</code>,
                        <code>/alpha_beta_filter</code>
            </td>
        </tr>
            <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_odometry/launch/tortoisebotpro_carto_odom.launch">tortoisebotpro_carto_odom.launch</a></td>
            <td>Produces odometry data for the robot using cartographer.</td>
            <td>
                        <code>/cartographer_node</code>
            </td>
        </tr>
        <tr>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_odometry/scripts/alpha_beta_filter.py">alpha_beta_filter.py</a></td>
            <td>Alpha beta filter to smoothen out the translation in x and y. Odometry generated is purely based on lidar and IMU. This is how we do it.</td>
            <td>
               <code>/alpha_beta_filter</code>
            </td>
        </tr>
    </tbody>
</table>

<p align="center">
	<img src="images/odom.png" width="900"/>
</p>

TF of odom is broadcasted by `alpha_beta_filter` for mapping agents. -->

### 3.6 [tortoisebotpro_slam](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_slam/)

SLAM!

<table>
    <thead>
        <tr>
            <th>File</th>
            <th>Description</th>
            <th>Node Launched</th>
        </tr>
    </thead>
    <tbody>
            <td><a href="https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_slam/launch/cartographer.launch">cartographer.launch</a></td>
            <td>To generate the map of the environment using Cartographer.</td>
            <td>
                <code>/cartographer_node</code>
            </td>
        </tr>
    </tbody>
</table>


**We have installed everything for you no need to worry about!**

## 4. Launch Sequence

### 4.1 Gazebo Simulation

#### 4.1.1 Map Generation

```py
ros2 launch tortoisebotpro_bringup autobringup.launch use_sim_time:=True exploration:=True # To launch The robot in sim without a saved map
```

<p align="center">
	<img src="images/sim_gz.png" width="700"/>
</p>


<p align="center">
	<img src="images/sim_gmap.png" width="700"/>
</p>

```py
ros2 run teleop_twist_keyboard teleop_twist_keyboard # To control the robot using keyboard
```


#### 4.1.2 Autonomous Navigation in the saved map

```py
ros2 launch tortoisebotpro_bringup autobringup.launch use_sim_time:=True exploration:=False # To launch robot in sim with a saved map
```


<p align="center">
	<img src="images/sim_mapnav.png" width="700"/>
</p>

### 4.2 Actual Robot

> [!NOTE]
> For every command to be executed within the robot a new SSH connection needs to be established.

#### 4.2.1 Map Generation

```py
ros2 launch tortoisebotpro_bringup autobringup.launch use_sim_time:=False exploration:=True # To launch real robot in without a saved map
```

```py
ros2 run teleop_twist_keyboard teleop_twist_keyboard # If using computer keyboard to control the robot
```



<p align="center">
	<img src="images/robo_map.jpg" width="700"/>
</p>

#### 4.2.2 Autonomous Navigation in the saved map

```py
ros2 launch tortoisebotpro_bringup autobringup.launch use_sim_time:=False exploration:=False # To launch real robot in with a saved map
```


<p align="center">
	<img src="images/robo_nav.jpg" width="700"/>
</p>



## 5. General Robot Information


<table>
    <tr>
        <th>Parameter</th>
        <th>Value</th>
    </tr>
    <tr>
        <td>Wheel Separation Length</td>
        <td>0.195m</td>
    </tr>
    <tr>
        <td>Motor Type</td>
        <td>Planetary DC Geared Motor</td>
    </tr>
    <tr>
        <td>RPM</td>
        <td>110</td>
    </tr>
    <tr>
        <td>Encoder Type</td>
        <td>Magnetic Encoder</td>
    </tr>
    <tr>
        <td>PPR (Pulses Per Revolution)</td>
        <td>420</td>
    </tr>
    <tr>
        <td>Microcontroller</td>
        <td>DOIT-ESP32 Devkit V1</td>
    </tr>
    <tr>
        <td>PC Used</td>
        <td>Intel NUC i3 10th Gen</td>
    </tr>
    <tr>
        <td>Robot Payload Capacity</td>
        <td>15 kgs</td>
    </tr>
    <tr>
        <td>Battery Life</td>
        <td>About 1.5 hours</td>
    </tr>
    <tr>
        <td>Battery Type</td>
        <td>Lithium-ion 6-cell, 22.2V</td>
    </tr>
</table>

### 5.1 Topic Description

<table>
    <thead>
        <tr>
            <th>Topic</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td><code>/bat_per</code></td>
            <td>Battery percentage remaining until complete discharge</td>
        </tr>
        <tr>
            <td><code>/bat_voltage</code></td>
            <td>Battery voltage</td>
        </tr>
        <tr>
            <td><code>/cmd_vel</code></td>
            <td>Command velocity for the robot</td>
        </tr>
        <tr>
            <td><code>/diagnostics</code></td>
            <td>Diagnostics messages</td>
        </tr>
        <tr>
            <td><code>/heading</code></td>
            <td>Robot heading based on magnetometer</td>
        </tr>
        <tr>
            <td><code>/imu/data</code></td>
            <td>IMU data including orientation, rotational velocities and linear acceleration</td>
        </tr>
        <tr>
            <td><code>/wheel/ticks</code></td>
            <td>Encoder reading of wheels in an array of the format of [left, right]</td>
        </tr>
        <tr>
            <td><code>/wheel/vel</code></td>
            <td>Wheel velocities in an array of the format of [left, right]</td>
        </tr>
        <tr>
            <td><code>/odom</code></td>
            <td>Odometry generated from wheel encoders</td>
        </tr>
        <tr>
            <td><code>/pid/constants</code></td>
            <td>Set PID constants</td>
        </tr>
        <tr>
            <td><code>/pid/control</code></td>
            <td>Should PID be used or not</td>
        </tr>
        <tr>
            <td><code>/scan</code></td>
            <td>Lidar measurements</td>
        </tr>
        <tr>
            <td><code>/usb_cam</code></td>
            <td>Cascaded topics providing complete information about the camera</td>
        </tr>
        <tr>
            <td><code>/diagnostics/test</code></td>
            <td>Run diagnostics on the robot</td>
        </tr>
    </tbody>
</table>

### 5.2 Battery

Within the robot a buzzer beeps to indicate the status of battery.

<table>
    <thead>
    <tr>
        <th>Battery Level</th>
        <th>Beeps Status</th>
    </tr>
    </thead>
    <tbody>
    <tr>
        <td>100 % to 20 %</td>
        <td>No beeps</td>
    </tr>
    <tr>
        <td>20 % to 15 %</td>
        <td>Beeps after every 2 mins</td>
    </tr>
    <tr>
        <td>15 % to 10 %</td>
        <td>Beeps after every 1 min</td>
    </tr>
    <tr>
        <td>10 % to 0 %</td>
        <td>Continuous Beeps</td>
    </tr>
</table>


> [!CAUTION]
> Do not drain the battery below `10 %`, doing so will damage the battery permanently.
> Maximum battery voltage is 25.2V and minimum usable battery voltage is 19.8V

A battery is made available on the robot which indicate the status of the battery so that you don't have to `echo` on topics. Every bar on the indicator indicates 25% battery health. So,

<table>
    <thead>
    <tr>
        <th>Bar Level</th>
        <th>Battery Level</th>
    </tr>
    </thead>
    <tbody>
    <tr>
        <td>1</td>
        <td>0 % to 25 %</td>
    </tr>
    <tr>
        <td>2</td>
        <td>25 % to 50 %</td>
    </tr>
    <tr>
        <td>3</td>
        <td>50 % to 75 %</td>
    </tr>
    <tr>
        <td>4</td>
        <td>75 % to 100 %</td>
    </tr>
</table>

<p align="center">
	<img src="images/bat.jpg" width="700"/>
</p>


### 5.3 Robot Velocities

Maximum Linear Velocity  - <code>0.37 m s<sup>-1<sup></code>
<br>
Maximum Angular Velocity  - <code>3.836 rad s<sup>-1<sup></code>

### 5.4 USB Ports

A strict rule needs to be followed while connecting Lidar, ESP32 and USB camera. These ports are hardcoded and devices needs to be connected as depicted below.

<p align="center">
	<img src="images/back.png" width="700"/>
</p>

<p align="center">
	<img src="images/front.png" width="700"/>
</p>

## ⚠️ 6. USB Port Configuration for ESP and LiDAR

To ensure consistent port names for the ESP and LiDAR devices across reboots and plug-in orders, we assign **static USB names** using **udev rules**. Follow the steps below carefully and **only on the robot's NUC via SSH**.

> \[!IMPORTANT]
> This setup ensures your ESP and LiDAR always map to `/dev/esp` and `/dev/lidar`, respectively. This is necessary because the launch files in the repository are already configured to use these port names.
> If not set correctly, the robot will **fail to communicate with the microcontroller or the LiDAR**.

---

### Step-by-step Instructions

#### ✅ Step 1: Connect the ESP to the NUC

Plug the ESP device into the NUC using the USB port.

Check the device path:

```bash
ls /dev/ttyUSB*
```

It should show something like:

```
/dev/ttyUSB0
```

Now identify the USB ID:

```bash
udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep KERNELS
```

You will see an output like this:

```
ATTRS{...}
KERNELS=="1-1"
ATTRS{...}
```

> \[!NOTE]
> **Note down the line with `KERNELS=="..."`**. This identifies the USB path for the ESP.
> In this example, it is:
> `KERNELS=="1-1"`

---

#### ✅ Step 2: Connect the LiDAR to the NUC

Now plug the **LiDAR** (e.g., YD LiDAR) into another USB port.

Again check the device path:

```bash
ls /dev/ttyUSB*
```

Now you'll see:

```
/dev/ttyUSB0  /dev/ttyUSB1
```

(Assuming the ESP is still connected as `/dev/ttyUSB0`, the new one `/dev/ttyUSB1` is the LiDAR.)

Now run:

```bash
udevadm info --name=/dev/ttyUSB1 --attribute-walk | grep KERNELS
```

You'll get an output like:

```
ATTRS{...}
KERNELS=="1-2"
ATTRS{...}
```

> \[!NOTE]
> Again, **note down** the line with `KERNELS=="..."`.
> In this example: `KERNELS=="1-2"`

---

#### ✅ Step 3: Edit the `install.sh` File

Now, open the file:

```bash
cd ~/ros2_ws/src/tortoisebot_pro-ros2-humble
```

Edit the `install.sh` script. Replace the values in the following lines with what you got in Steps 1 and 2:

```bash
SUBSYSTEM=="tty", KERNELS=="1-2", SYMLINK+="esp"
SUBSYSTEM=="tty", KERNELS=="1-1", SYMLINK+="lidar"
```

For example, if:

* ESP → `KERNELS=="1-1"`
* LiDAR → `KERNELS=="1-2"`

Then your lines should be:

```bash
SUBSYSTEM=="tty", KERNELS=="1-1", SYMLINK+="esp"
SUBSYSTEM=="tty", KERNELS=="1-2", SYMLINK+="lidar"
```

Save the file.

Now make it executable and run it:

```bash
chmod +x install.sh
./install.sh
```

You should see:

```
USB Ports configured!..
```

This sets up the udev rules and reloads them.

---

### ⚠️ Final Steps: Update Launch and Config Files

Now that the ports are set, make sure the port names in your launch/config files are correct.

Check and ensure:

#### 1. For ESP (micro-ROS firmware):

```python
tortoisebotpro_firmware/launch/micro_ros.launch.py
```

Set the serial device to:

```python
/dev/esp
```

#### 2. For LiDAR:

```yaml
ydlidar_ros2_driver/params/ydlidar.params
```

Set the port to:

```yaml
/dev/lidar
```

---

### ⚠️ Disclaimer

> \[!WARNING]
> This step involves modifying system-level USB rules. Please do it **carefully and only on the NUC's SSH terminal**.
> Misconfiguration can prevent your robot from detecting the ESP or LiDAR correctly.
> Make sure the values for `KERNELS==...` are correct and mapped to the correct devices.

---

Let us know if you face any issues with this step!
