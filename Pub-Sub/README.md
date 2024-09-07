# Custom ROS 2 Messages - Pub-Sub Systems

This README.md file contains three beginner-level ROS 2 systems, all using the **publish-subscribe (pub-sub)** model. 
Each system utilizes custom ROS 2 messages for communication between nodes. No sensor subscriptions are involved; the focus is on simplicity and mastering the basics.

## Problem 1: Decentralized Robotic Fleet Telemetry System with Dynamic Task Distribution
In a distributed warehouse environment, a fleet of autonomous robots must asynchronously transmit operational telemetry data (battery levels, spatial coordinates, and 
task engagement) to a central coordinator. The coordinator must ingest, filter, and evaluate each robot's status to dynamically allocate tasks based on real-time robot 
availability and operational health using a custom message protocol.

### Objective: 
Build a ROS 2 system where robots publish their internal state to the fleet manager. The fleet manager, in turn, continuously evaluates these states and processes 
the data to assign tasks.

### Challenges:
- Robots must publish a consistent data stream in real-time without collision or data loss.
- The central node should evaluate the status, calculate optimal task assignments, and simulate task distribution.

---------------------------------------------------------------

## Problem 2: Decentralized Robotic Fleet Telemetry System with Dynamic Task Distribution
An autonomous aerial vehicle must propagate continuous kinematic and power data to a monitoring station via a custom inter-node communication protocol. 
The vehicle must send complex, multi-dimensional information (including velocity vectors, power levels, and altitude) at varying frequencies. 
The monitoring station must efficiently parse and visualize the incoming data for real-time flight diagnostics.

### Objective:
 Implement a ROS 2 system where the drone node generates simulated sensor data (without directly subscribing to sensor nodes),
 publishing its status via a custom message to the ground control station.
 
 ### Challenges:
 - Efficiently simulate complex kinematic data transmission without sensor integration.
 - Ensure that the ground station accurately reconstructs the data in real-time and handles high-frequency updates.

----------------------------------------------------------------

## Problem 3: Advanced Object Recognition and Grasping Framework with Industrial Robotics Interface
A vision-processing node continuously analyzes incoming visual data, identifying objects' spatial orientation and type in a dynamic industrial environment. 
The node must asynchronously transmit this information to a robotic manipulator for potential grasping. The robotic arm must evaluate the viability of grasping based on
object-specific parameters, including geometry, type, and accessibility, relying solely on the vision node's data stream.

### Objective:
Create a ROS 2 system where the vision node simulates object detection, publishing detailed
metadata for each object. The robotic arm node subscribes to this stream and determines grasp 
feasibility without executing physical motions.

### Challenges:
- Simulate the vision-processing pipeline for detecting objects and calculating their graspable status.
- Build a complex message structure that provides detailed object information for robotic arm control logic.

-----------------------------------------------------
## NOTE: All three problem statements and solutions are beginner-friendly and adhere to the pub-sub model without involving any sensor data subscriptions!
