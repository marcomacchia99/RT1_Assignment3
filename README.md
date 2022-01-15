Assignment 3 (final assignment) - Research Track 1 
================================

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
    <include file="$(find RT1_Assignment3)/launch/simulation_gmapping.launch"/>
    <include file="$(find RT1_Assignment3)/launch/move_base.launch"/>
    <node pkg="RT1_Assignment3" type="mainController" name="mainController" output="screen" required="true" launch-prefix="xterm -fa 'Monospace' -fs 11 -e"/>
</launch>
```

Notice that `launch-prefix` of mainController node contains some rules regarding the font family and the font size, you are completely free to change it!

Simulation environment
---------

After launching the simulation using the provided commands two programs will open, [__Gazebo__](http://gazebosim.org/) and [__Rviz__](http://wiki.ros.org/rviz).

Gazebo is an open-source 3D robot simulator. Here's the simulation view from Gazebo:

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/world/tracciato.png)

ROS generate the environment based on the file `house.world`, stored into the __world__ folder.

Initially the robot knows only what he can see, here's the image showing his initial known map.

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/world/tracciato.png)

After some time the robot has explored and mapped all the surrounding walls using his laser scan.

We can now see the full map into rviz.

Controller node
--------------

The controller node is capable of driving potentially indefinitely all around the track, automatically detecting straights and turns. When the robot is approaching a turn, the node automatically tells him to slow down, so that he can make the right controls.

The controller uses all the sensors data received by the `/base_scan` publisher after he subscribes to it. this topic is composed by 720 _ranges_, in which there are all the detected distances. the sensor can see from -90 to 90 degrees, so each sensor has 1/4 of degree of view.

After a message from `/base_scan` is recieved, the controller node enters inside the `checkTrackLimits` function, that filters all the ranges taking only the one from:
* -90° to -70°, 
* -10° to 10°,
* 70° to 90°. 

After that the function checks for the minimum value inside each of the three sets, and choose what action has to be done:

* if the front wall is nearer then `f_th = 2`meters, he checks the lateral distances:
  * if the left distance is more then the right distance he slightly turns to the right
  * otherwise he slightly turns to the left
* if the front wall is furthest then the treshold, then the robot goes straight.


When the robot goes straight, he uses the `/Speed_val` value as speed value. `/Speed_val` is managed by the UI node, according to what he receives from the `/accelerator` service. 

After doing that, the controller node publish the data to the `/cmd_vel` topic, used to control the robot movement.

### Flowchart ###

<image src="https://github.com/marcomacchia99/RT1_Assignment2/blob/main/assets/diagram.png" width="600px">


### Accelerator Service ###

The accelerator service,  runned by the controller node,  manages the robot's speed, and he works in close contact with the UI node, used to interact with the final user. It simply checks the character received by the UI node and increments or decrements the speed by 0,5. Also, if the button R is pressed, the service automatically resets the robot position to its initial position and velocity, using the `/reset_positions` service.

UI node
------

The UI node is responsible of receving the user input from the keyboard. Firstly it reads the input command, then it checks if it is correct, acoording to this table:

<center>

| Input | Action |
|:--------:|:----------:|
|__[a], [A]__|__Accelerate__|
|__[d], [D]__|__Decelerate__|
|__[r], [R]__|__Reset position__|

</center>

If the command is correct, it simply sends it to the controller node (via acceleration service), and prints to the user the new velocity.
This is the code used to "talk" with acceleration service.

```cpp
service.request.input_char=inputChar;
        

        service_client.waitForExistence();
        service_client.call(service);

        RT1_Assignment2::Speed_val speed;
        speed.speed = service.response.value;
        pub.publish(speed);
        system("clear");
        std::cout << "New speed: "<< service.response.value<<"\n\n";
```

Project graph
-------------------
 
Here's the project graph which explains the relationship within the nodes.
The graph can be generated using this command:
 
```console
$ rqt_graph
``` 

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/assets/graph.png)

Video demonstration
-------------------
 
This is the outcome of the project. The robot is able to stay inside the track limits until he reaches a velocity of 26,0. His optimal speed, however, is from 8,0 to 10,0.

![alt text](https://github.com/marcomacchia99/RT1_Assignment2/blob/main/assets/video.gif)
 
Conclusion and future improvements
-------------------
By now the robot can autonomousely drive inside the [Autodromo Nazionale di Monza](https://www.monzanet.it/), but all the movement aren't smooth at all.
A next update could introduce better movements inside the turn, expecially inside the _Prima variante_, and a bettere _user experience_.

It could also be possibile to dynamically change the robot speed, as the real cars actually do: The robot can drive at a speed that is inversely proportional to the amount of remaining straight. In this case however, the user input will became useless.
