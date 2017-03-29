# Autonomous Driving

This software was developed as part of college project at Hochschule Darmstadt in semester 2016 / 2017.

You can watch a demo of this project by clicking at the image below.
[![DEMO](http://img.youtube.com/vi/hJZtA5Ydt8g/0.jpg)](http://www.youtube.com/watch?v=hJZtA5Ydt8g)


# Installation

## Preparations

First of all you need a ROS installation with the Indigo distribution.
A detailed installation guide for ROS can be found at http://wiki.ros.org

There are some dependencies requiered to build the project.
You can either install the manually or by using the [rosdep](http://wiki.ros.org/rosdep)-command from the ros environment. 

## Build project

Next you have to create a workspace at any place of your file system. 

```shell
mkdir -p <workspace_folder>/src
cd <workspace_folder>/src

catkin_init_workspace
```

With the following statement you add important workspace informations to your environment variables. You always have to do this before you can use this package. 

```shell
cd ../

catkin_make

source devel/setup.bash
```

After that you go to the src-folder, clone the project into it and build the ros package
```shell
cd src
git clone https://github.com/StatueFungus/autonomous_driving.git autonomous_driving

cd ../
catkin_make
```
