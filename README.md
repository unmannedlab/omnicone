# Omnicone Package
A package containing the code for the automatic omnicone placement robots. 


## Dependencies 
All dependencies are internal to this pacakge except
[ubxTranslator](https://github.com/unmannedlab/ubxtranslator.git).

## Quick Start 

1. Clone the repository into a catkin workspace
2. Run `catkin_make`
3. Copy omnicone/udev/99-serial.rules into /etc/udev/rules.d/

## Physical Setup 

Setup the RTK base station tripod and, once stationary, plug in the RTK base station board. Corrections should be availalble in ~10 minutes or less. 

Ensure the network 

SSH into each cone in separate terminals (Note: these IP addresses only work when using the configured Ubiquiti access point and Edge router). Once connected to the cones run the ROS launch file for each cone.

```bash
ssh pi@192.168.1.11
roslaunch omnicone demo1.launch
```
```bash
ssh pi@192.168.1.12
roslaunch omnicone demo2.launch
```

In a third terminal, launch the cone commander gui
```bash
roslaunch omnicone Heartbeat.launch
```