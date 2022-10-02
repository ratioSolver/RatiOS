# RatiOS

The Robot Operating System ([ROS](https://www.ros.org/)) is a set of software libraries and tools that help you build robot applications. ROS processes are represented as nodes in a graph structure, connected by edges called _topics_. Such nodes can pass _messages_ to one another through _topics_, make _service calls_ to other nodes, provide a _service_ for other nodes, or set or retrieve shared data from a communal database called the parameter server. 

This folder contains a ROS package, called _RatiOS_, which, by implementing the [AerialS](https://github.com/ratioSolver/AerialS) API, allows interacting with the [PlExA](https://github.com/ratioSolver/PlExA) Plan Executor and Adaptor.