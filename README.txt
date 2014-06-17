Project: Pick ‘n’ Place Robot

Students:
Name	        Roll No.	Email
Bharat Jain	09305029	bharatj@cse.iitb.ac.in
Dinesh Rajpoot	09305043	dineshr@cse.iitb.ac.in

Project Objective
The aim of the project is to implement a robot which identifies objects and places them on a given place.  There is a 
field of an uniform background colour on which the objects are placed. The images are taken from an overhead camera, 
which is interfaced with MATLAB7.0. The positions of the robot and objects are obtained using image processing and the 
nearest object is given highest priority.

Hardware Platform
1.	Firebird V ATMEGA 2560
2.	Xbee modules for communication
3.	Robotic  Arm Assembly
4.	External Power Supply (Battery provided with robot is not sufficient to drive robotic arm assembly.)
5.	Logitech 2 mega pixel camera. (Any camera with high resolution will work)

Software
1.	AVR Studio 4
2.	MATLAB R2008b
3.	Logitech camera device driver.
 
Code Description
Code Files.
Filename	      Purpose	                                        Executes on
main.c	              Main Program	                                   Robot
firbird.h             Contains the abstractions of major operations.	   Robot
Imageprocessing.m     Perform Image processing and sends control           PC 
                      commands to robot.	



Execution Instructions
Before starting the execution user should ensure that he has all the files mentioned in code files section
such as "main.c", "firebird.h" and "imageprocessing.m".
Follow the instruction below:
1.	Open AVR Studio and make new project.
2.	Add "main.c"  in source file and "firebird.h" in header  file section.
3.	Ensure to check box related to hex file creation in configuration setting.
4.	Compile “main.c” and burn the hex file in micro-controller.
5.	Attach overhead camera at some height and plug-in into PC. 
6.	Open MATLAB and set the colour intensity in “imageprocessing.m” as per condition of lighting in the room.
7.	Once colour setting is done, run  imageprocessing.m in  MATLAB. 
