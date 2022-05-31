Assignment - Research Track 2
================================

Code Documentation
------------
The project documentation, generated with Doxygen can be found in the following link:

[__marcomacchia99.github.io/RT2_Assignment/index.html__](https://marcomacchia99.github.io/RT2_Assignment/index.html)

Data Analysis
------------
The results of the data analysis, computed with Matlab, can be found in the [__PDF Report__](https://github.com/marcomacchia99/RT2_Assignment/blob/master/RT2_Data_Analysis.pdf)

-----------------------

Introduction
------------

The final assignment deals with a robot moving into an initlially __unknown__ enviroment. Depending on the mode selected, the robot can __drive autonomously, reaching a given goal__, __being driven by the user__ and __being driven by the user, assisting them to avoid collisions__.

Installing and running
----------------------

The simulator requires [__ROS__](http://wiki.ros.org) (__Robot-Operating-Systems__) to be installed on the machine. In particular, the [Noetic Release of ROS](http://wiki.ros.org/noetic/Installation) was used.

For this particular simulation there are some required elements:
* [Slam Gmappic package](https://github.com/CarmineD8/slam_gmapping)
* ros navigation stack
* xterm

If you don't have any of these elements you can run the following instructions:

```console
$ git clone https://github.com/CarmineD8/slam_gmapping.git
```
```console
$ sudo apt-get install ros-<your_ros_distro>-navigation
```
```console
$ sudo apt install xterm
```
After doing that, you are ready to __launch the simulation!__
A launch file, called `launcher.launch`, is provided to run all the required nodes.

this is its structure:

```xml
<launch>
    <include file="$(find RT2_Assignment)/launch/simulation_gmapping.launch"/>
    <include file="$(find RT2_Assignment)/launch/move_base.launch"/>
    <node pkg="RT2_Assignment" type="mainController" name="mainController" output="screen" required="true" launch-prefix="xterm -fa 'Monospace' -fs 11 -e"/>
</launch>
```

Notice that `launch-prefix` of mainController node contains some rules regarding the font family and the font size, you are completely free to change it!

Simulation environment
---------

After launching the simulation using the provided commands two programs will open, [__Gazebo__](http://gazebosim.org/) and [__Rviz__](http://wiki.ros.org/rviz).

Gazebo is an open-source 3D robot simulator. Here's the simulation view from Gazebo:

![alt text](https://github.com/marcomacchia99/RT2_Assignment/blob/master/assets/gazebo1.jpg)

ROS generate the environment based on the file `house.world`, stored into the __world__ folder.

Initially the robot knows only what he can see, here's the image showing his initial known map.

![alt text](https://github.com/marcomacchia99/RT2_Assignment/blob/master/assets/rviz1.png)

After some time the robot has explored and mapped all the surrounding walls using his laser scan.

We can now see the full map into rviz, as shown below:

![alt text](https://github.com/marcomacchia99/RT2_Assignment/blob/master/assets/rviz2.png)

MainController node
--------------

The mainController node is the first node, spawned with the `launcher.launch` file. This node simply prompts some instruction in the xterm console, then it detects and interprets the user inputs.

The user can:
* __1__ - Reach autonomousely a given position
* __2__ - Drive the robot with the keyboard
* __3__ - Drive the robot with the keyboard with automatic collision avoidance
* __4__ - Reset simulation
* __0__ - Exit from the program

Generally speaking, this simulation includes a non-blocking getchar function, ideal to speed up the program execution and to improve the user execution.

I found this function in [teleop_twist_keyboard_cpp repository](https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp), and it temporarily edits the system settings in order to catch immediately what the user writes.

Remember that the `termios.h` library is required, so don't remove it!

Finally, based on the input received, the mainController node runs the selected node, using `system()` function.

For example, if the number 1 is pressed, this command is executed:

```c
system("rosrun RT2_Assignment reachPoint");
```

ReachPoint node
--------------
The reachPoint node implements the first required feature. In fact it set a new goal for the robot according to what the user wants.

At his initial state, the node request the x and y coordinates of the goal, then it generates a new message of type `move_base_msgs/MoveBaseActionGoal`.
The message is then published into the `/move_base/goal` topic. 

When the message is published, the robot starts looking for a valid path which can lead to the goal, and he followes it.

During the navigation, the user can at any time:
* stop the navigation by pressing the __q__ key, or
* exit the node by pressing __CTRL-C__ key

If one of this keys is pressed, a message of type `actionlib_msgs/GoalID` is generated and then published into the `/move_base/cancel` topic.
In particular, every goal is tracked by the node with its __id__, randomly generated by the node itself using `rand()` function, so sending the goal cancel message is quite easy.

In order to know if the robot has reached the goal or if the robot can't reach it a `/move_base/status` message handler is implemented. It continousely checks the messages published into that topic, in particular it looks for the __status__ code.

Initially the status code is __1__, meaning that the robot is following his path. When the robot stop there are two possibilities: if the code is equal __3 (succeded)__ then it means that the goal has been successfully reached, otherwise if the robot can't reach the goal the status code will be set to __4 (aborted)__.

Based on the code the node displays the result in the console, then it asks if the user wants to select a new goal or exit from this node.


DriveWithKeyboard node
--------------
The driveWithKeyboard node let the user drive the robot using the keyboard. 

Here, I decided to __edit the teleop_twist_keyboard__ node, starting from the `.cpp` version which I found in the [teleop_twist_keyboard_cpp repository](https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp).

In particular I clean the user interface, and I added the possibility to reset the linear and the angular speed. Also there's a new possibility to safely quit from the execution using CTRL-C combination.

The node simply checks the user imputs according to the instructions prompted in the console, and it publish the new speed to the `/cmd_vel` topic.

The speed is computed as the relative speed multiplied with the selected direction, which is an integer defined between -1 and 1. 1 means that the robot must go forward or turn left, -1 means backward (or turn right), 0 means stop.

Here's the computations in terms of code:

```c
//define variables for vel direction
int lin=0; //linear direction
int ang =0; //angular direction


vel.angular.z = turn_speed * ang;
vel.linear.x = speed * lin;
```
The user can use a 3x3 input keys as they are a "joystick". Here's the keys:
<center>

|| Turn left | Don't turn | Turn right|
|:--------:|:--------:|:----------:|:----------:|
|__Go forward__|u|i|o
|__Dont' go__|j|k|l
|__Go backward__|m|,|.
    
</center>

Also, the user can set the robot linear and angular joystick, using this set of commands:

<center>
    
|| Change linear and angular | Change linear only | Change angular only|
|:--------:|:--------:|:----------:|:----------:|
|__Increase__|q|w|e
|__Reset__|a|s|d
|__Decrease__|z|x|c

</center>


DriveWithKeyboardAssisted node
--------------

The driveWithKeyboardAssisted node, similar to the node above, let the user drive the robot using the keyboard, __assisting him during the navigation__.

In particular, the node reads the same identical user inputs as the driveWithKeyboard node, but it also checks what the robot's laser scanner sees.
To do so, the node subscribes to the `/scan` topic, and it uses the message received to detect walls too close to the robot. This topic is composed by 720 _ranges_, in which there are all the detected distances. the sensor can see from -90 to 90 degrees, so each sensor has 1/4 of degree of view.

After a message from `/scan` is recieved, the node enters inside the `checkWalls` function, that filters all the ranges taking only the one from:
* -90° to -55° referred to the walls on the right, 
* -17.5° to 17.5° referred to the walls in front of the robot,
* 55° to 90° referred to the walls on the left.

The function then checks the minimum distance inside this ranges, and if a wall is closer than `wall_th = 1 (meter)` it prevents the robot from getting too close to it. In particulat, if the front wall is too close the robot can't advance, while if one of the walls on the left or on the right is too close the robot can't turn in that direction.

To actuate this security feature the functions simply edits the linear and angualar direction according to the rules above, setting them to __0__ when required.

Finally, a __red danger warning string__ is prompted to the user.


Flowchart
--------

<image src="https://github.com/marcomacchia99/RT2_Assignment/blob/master/assets/diagram.png" width="600px">

    
Project graph
--------
 
Here's the project graph which explains the relationship within the nodes.
The graph can be generated using this command:
 
```console
$ rqt_graph
``` 

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/assets/graph.png)

 
Conclusion and future improvements
-------------------
    
By now the robot can autonomousely drive inside the [Autodromo Nazionale di Monza](https://www.monzanet.it/), but all the movement aren't smooth at all.
A next update could introduce better movements inside the turn, expecially inside the _Prima variante_, and a bettere _user experience_.

It could also be possibile to dynamically change the robot speed, as the real cars actually do: The robot can drive at a speed that is inversely proportional to the amount of remaining straight. In this case however, the user input will became useless.
