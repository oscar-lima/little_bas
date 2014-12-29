Hello world ros arduino
=======================

This tutorial will help you to make your first program to convert your arduino into a ros node

documentation can be found on:

		http://wiki.ros.org/rosserial_arduino

1. Install dependencies

		sudo apt-get install ros-hydro-rosserial-arduino
		sudo apt-get install ros-hydro-rosserial
		sudo apt-get install ros-hydro-rosserial-leonardo-cmake

2. go to your sketchbook libraries arduino directory

		cd ~/sketchbook
		mkdir libraries
		cd libraries

3. generate the needed libraries

		roscore
		rosrun rosserial_arduino make_libraries.py .

4. load hello_world tutorial

		open your arduino IDE
		select file->examples->ros_lib->HelloWorld
		compile and upload the program to your arduino board

5. run the nodes and test

		roscore
		rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
		rostopic echo chatter

6. Done! now you should be able to see this message:

---
data: hello world!
---
data: hello world!
---
data: hello world!

which means your arduino is publishing on ros network