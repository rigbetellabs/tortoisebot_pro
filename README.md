
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
            <li><a href="#31-tortoisebotpro_control">tortoisebotpro_control</a>
            </li>
            <li><a href="#32-tortoisebotpro_description">tortoisebotpro_description</a>
            </li>
            <li><a href="#33-tortoisebotpro_firmware">tortoisebotpro_firmware</a>
            </li>
            <li><a href="#34-tortoisebotpro_gazebo">tortoisebotpro_gazebo</a>
            </li>
            <li><a href="#35-tortoisebotpro_navigation">tortoisebotpro_navigation</a>
            </li>
            <li><a href="#36-tortoisebotpro_odometry">tortoisebotpro_odometry</a>
            </li>
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
</ol>
</details>


## 1. Installation

Clone the repository into your workspace,

```py
cd ~/catkin_ws/src # Assuming catkin_ws is the name of the workspace
git clone https://github.com/rigbetellabs/tortoisebot_pro.git
```

Build the workspace,
```py
cd ~/catkin_ws/
catkin_make
```

Installation of dependent packages,
```py
cd ~/catkin_ws/src/
cat requirements.txt | xargs sudo apt-get install -y 
# This installs all the packages mentioned in the requirements.txt
```

> [!NOTE]
> Check if you already have the lidar packages installed; if not, get the packages from repos below.

```py
cd ~/catkin_ws/src/
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

> [!IMPORTANT]
> We have tested mapping and navigation using Gmapping and Cartographer, Cartographer has some inherent flaws hence we prefer to use Gmapping over Cartographer. Launch files for cartographer are provided for you to experiment. Description for these launch files are provided but the launch sequence for these lauch file has not been added.


### 3.1 [tortoisebotpro_description](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_description/)

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

### 3.7 [tortoisebotpro_slam](https://github.com/rigbetellabs/tortoisebot_pro/blob/master/tortoisebotpro_slam/)

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

### 3.8 install.sh

Performs,
<br>
- Installation of udev rules to hardcode the physical USB ports

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