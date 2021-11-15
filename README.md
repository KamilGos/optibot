
<div align="center" id="top"> 
  <img src=images/robot1.jpg width="250" />
  <img src=images/robot2.jpg width="238" />
  <img src=images/robot3.jpg width="247" />
  &#xa0;
</div>

<h1 align="center"> OptiBot </h1>
<h2 align="center"> Mobile platform for analysis of localization methods using the Intel RealSense T265 sensor </h2>


<p align="center">
  <img alt="Status" src="https://img.shields.io/badge/Status-done-green?style=for-the-badge&logo=appveyor">
    <img alt="Repository size" src="https://img.shields.io/github/languages/code-size/KamilGos/OptiBot?style=for-the-badge">
</p>


<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0;
  <a href="#package-content">Content</a> &#xa0; | &#xa0;
  <a href="#eyes-implementation">Implementation</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="#technologist-author">Author</a> &#xa0; | &#xa0;
</p>

<br>

## :dart: About ##

This robot was created for my Master Thesis. The main scope of this thesis was to analyse the localisation methods for mobile robots that operate in indoor environments. The method of interest acknowledged in this thesis is Visual-Inertial Odometry using RealSense T265 by Intel 

During this work, the evaluation environment was created to compare the data from the T265 sensor with different localization methods, including wheel odometry, lidar odometry, and motion capture system, which was used as the reference one. The main goal of this thesis was to confirm whether the out-of-a-box VIO method implemented in the Intel RealSense T265 sensor is suitable for the localisation of mobile robots that operates in indoor spaces. 

This repository presents the mechanical, electronic and software aspects of the test environment that was created to evaluate selected localization methods - the **OptiBot** mobile platform. 

## :package: Content

1. **Inventor** - CAD files for 3d model of created robot. Project was created using Autodesk Inventor.
2. **Teendy** - code for microcontroler - ros_serial, PID and odometry impelmentation
3. **ROS** - ROS package for mobile platform
4. **Results** - pdf file with tests resutlts

## :eyes: Implementation ##

<h2 align="left">1. Mechanics </h2>
<div align="center" id="inventor"> 
  <img src=images/model_1.png width="230" />
  <img src=images/model_2.png width="236" />
  <img src=images/model_3.png width="228" />
  &#xa0;
</div>

Figures shows the model of designed mobile platform. The 3D model was prepared using the Autodesk Inventor software. The mechanics of the robot have been designed to meet the assumptions of the differential drive model. 

The robot has two of the same wheels mounted on a common axis at the centre of the robot. This allows the platform to rotate in place. Each wheel is independent. It can spin at different speeds and in different directions (forward and backwards). To stabilise the robot, additional two swivel wheels were bolted to the chassis. The swivel wheels are free to turn and not controlled. The body of the robot is a traditional cuboid, which makes the construction rigid and tough. The robot frame was built with aluminium construction profiles. They combine low weight, high strength and easy assembly. The robot's chassis and elevating elements were made of profiles with a diameter of 20x20mm. Additionally, the chassis has been reinforced around the engines using the additional 20x60 profile, which strengthens the structure around the wheels. The elements constituting the top of the robot are made of 20x40 profiles. The use of extended profiles makes it possible to easily install additional elements on the roof. With evenly distributed weight, the robot can carry up to 40 kilograms. The plate constituting the chassis of the robot is made of 4mm plexiglass. Mounting holes have been laser cut, thus obtaining very high accuracy. Plexiglass is a sturdy and durable material, however easy to drill, so the element can be easily modified. The chassis plate is attached to the profiles with bolts. This element is also the basis for the assembly of all electronics. The roof of the robot is also made of plexiglass (3mm, black). It is attached to the robot with hinges and neodymium magnets. This element has an additional cut-out hole for the fan, thanks to which the exchange of air between the robot and the environment is easier. The rear part of the robot is a 3D printed panel with displays, pushbuttons, switches and connectors. The robot is driven by two BLDC wheels. The tyre is made of rubber, which significantly increases the traction of the robot to the ground. This type of wheels is characterized by a fixed shaft. This shaft is attached to the robot's construction with a holder and massive metal screws, which makes them very stable. Additionally, 3D printed gears are mounted to the wheels, which enables the transmission of torque to rotary encoders. The second gear is mounted on the encoder shaft. To connect the shafts the timing belt is used. All elements are GT2 type. The use of such elements made it possible to obtain the gear ratio 142:50, which gives 2.84 revolutions of the encoder shaft per one revolution of the wheel.

<h2 align="left">2. Electronics </h1>
<div align="center" id="electronics"> 
  <img src=images/electronics.png width="500" />
  &#xa0;
</div>
Figure shows the general idea of robot electronic system architecture. It presents the most important elements of which the robot is composed. The whole system is divided into two main parts labelled **ROS** and **Microcontroller**. Elements in the ROS area are controlled directly by the Robot Operating System (ROS) while the elements in the Microcontroller area are controlled by the algorithms implemented on a used microcontroller. The microcontroller is also a bridge between the ROS and motion control algorithms.


<h2 align="left">3. ROS </h1>
<div align="center" id="ros"> 
  <img src=images/ros_structure.png width="500" />
  &#xa0;
</div>
 
 * **tf** is a package that allows keeping track of multiple coordinate frames over time. It handles all the coordinate systems relations and lets the user transform points between them. There are 5 main frames in the robot: map frame (global coordinate system), base frame (mobile platform local coordinate system), lidar frame (lidar local coordinate system) and cameras frames (T256 local coordinate systems). 
  * **transfer functions** is the node that broadcast the transform between frames.
  * **teleop_joy** is a node that maintains the connection with the joystick. It is used to allow wireless control of the robot using a classical gamepad with two potentiometers. It reads the signal from a joystick, processes it and publishes the control values using the cmd_vel topic. cmd_vel is a topic that sends the information about angular and linear velocity using standard geometry_msgs/Twist message.
  * **serial_node** is a node that handles the serial communication with the microcontroller. It subscribes to the cmd_vel topic which allows to control the robot directly from the gamepad or using commands sending from the workstation. It sends the velocity to Teensy, which process the data and produce the control signal. On the other hand, serial_node topic is responsible for publishing the odometry information obtained from a microcontroller. Those information are sending using standard nav_msgs/Odometry message. nav_msgs/Odometry is the estimate, that contains the pose and twist information.
  * **lidar** node is a programme that maintains the connection with lidar. It is supplied by Slamtec together with the lidar, so it is optimised for the given lidar model. It publish the scan topic which is a standard sensor_msgs/LaserScan message that contains information about distances to the obstacle for each measurement. 
  * **rf2o** is a node that execute the rf2o algorithm. It subscribes to the scan topic and processes it to obtain the odometry information. 
  * **camera_1** and **camera_2** are the same (but independent) topics that handle connection with RealSense T265 camera. This node is provided by Intel company, so as well as for lidar, it is fully optimised for this camera. Both of those topics publish several different topics but only the odometry topic is used. 

During the tests of the localization algorithms, the **rosbag** package was also used, which enables the record of the messages that appear in the system. This package makes it possible to recreate the entire course of the experiment at any time without losing any information. The data collected in this way (called rosbag) is also easy to analyse because all the data can be exported to a standard CSV file or directly imported into a Python script. Also, it is important to note, that all the nodes responsible for odometry calculations run in parallel and independent of each other and exposed to the same environment at the same time, which ensure that the results are fair. 


<h2 align="left"> Environment, Grand truth, Evaluation, Data Analysis and Results</h1>
This is very widly decribed part of this projekt, so it was moved to /results/results.pdf file. 

## :memo: License ##

This project is under license from MIT. For more details, see the [LICENSE](LICENSE.md) file.

## :technologist: Author ##

Made with :heart: by <a href="https://github.com/KamilGos" target="_blank">Kamil Goś</a>

&#xa0;

<a href="#top">Back to top</a>






