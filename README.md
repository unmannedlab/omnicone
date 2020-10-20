# Omnicone Package
A package containing the code for the automatic omnicone placement robots. 

## Quick Start 

1. Clone the repository into a catkin workspace
2. Run `catkin_make`
3. Copy omnicone/udev/99-serial.rules into /etc/udev/rules.d/

## Physical Setup 

Setup the RTK base station tripod and, once stationary, plug in the RTK base station board. Corrections should be availalble in ~10 minutes or less. 

Ensure the 

SSH into each cone in separate terminals using:

```bash
ssh pi@192.168.1.11
```
```bash
ssh pi@192.168.1.12
```

Once connected to the cones run the ROS launch file for each cone

```bash
roslaunch omnicone demo1.launch
```

```bash
roslaunch omnicone demo2.launch
```

In a third terminal, launch the cone commander gui
```bash
roslaunch omnicone Heartbeat.launch
```