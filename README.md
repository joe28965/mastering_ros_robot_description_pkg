# **mastering\_ros\_demo\_pkg**

Source code for _Chapter 2_ of the book [Master ROS for Robotic Programming, Second edition](https://www.packtpub.com/hardware-and-creative/mastering-ros-robotics-programming-second-edition) source code main repository.

### Main repository
[mastering\_ros\_2nd\_ed](https://github.com/jocacace/mastering_ros_2nd_ed)

![book_cover](http://wpage.unina.it/jonathan.cacace/Media/book_cover.png "mastering_ros_for_robotics_programming")

## **Author**
[Jonathan Cacace](http://wpage.unina.it/jonathan.cacace), PhD.


## **Description**

**Chapter 2**: Working with 3D Robot Modeling in ROS

Working with ROS robot modeling using URDF.

_Original source code_ from [Josep Lentin](https://www.linkedin.com/in/lentinjoseph/): [Mastering Ros for Robotics Programming]( https://github.com/qboticslabs/mastering_ros)

### **Installation** 
Download this package in the _src_ folder of your ROS workspace

```git clone -b foxy https://github.com/jocacace/mastering_ros_robot_description_pkg```

### **Get this book** (in pre-order)
- [Packt Publishing](https://www.packtpub.com/hardware-and-creative/mastering-ros-robotics-programming-second-edition) 
- [Amazon](https://www.amazon.com/Mastering-ROS-Robotics-Programming-Second/dp/1788478959)

## Added for Foxy Branch
Currently only the pan_tilt urdf works.

Launch the following file to start gazebo and rviz with the pan_tilt robot.

```ros2 launch mastering_ros_robot_description_pkg view_pan_tilt_urdf.launch.py```

Launch ```rqt``` in a different terminal.
Go to Plugins->Topics->Message Publisher.
Add the topic ```pan_topic``` and ```tilt_topic``` to your list of topics.

Now you can set values to those topics to publish (do not forget to tick the checkbox to start publishing).

The range for the pan joint is ```lower="-3.14" upper="3.14"```

The range for the tilt joint is ```lower="-4.64" upper="-1.5"```

