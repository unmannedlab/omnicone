This ROS package is a driver node for controlling the roboteq SDCx motor
controllers.

### Devices that have been used successfully
- SDC2130

### Setting up udev

We plug these in via a mini-usb cable; they show up as /dev/ttyACMX.

```bash
$ lsusb
Bus 003 Device 002: ID 8087:8001 Intel Corp.
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 005: ID 8087:0a2a Intel Corp.
Bus 001 Device 004: ID 20d2:5740
Bus 001 Device 003: ID 20d2:5740
Bus 001 Device 002: ID 046d:c534 Logitech, Inc.
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

The 20d2:5740 entries are two roboteq controllers. We can check on the serial
number for one of them

```bash
$ udevadm info -a -n /dev/ttyACM0 | grep '{serial}' | head -n1
    ATTRS{serial}=="SDC2XXX"
```

Unfortunately, the devices do not seem to have unique serial numbers, so we
cannot use that to distinguish between the two roboteq controllers on our
robot.

To get around this, we use the `set_roboteq_id` script to save a unique id in
the EEPROM of each controller, and tell udev to use the `get_roboteq_id` script
to include that unique id in a symlink for that device.

```bash
$ rosrun roboteq_python set_roboteq_id /dev/ttyACM0 42
Setting idnum for roboteq attached to /dev/ttyACM0 to 42
Saving configuration to EEPROM
ID number set successfully
```

Now if we read the values back we see the same id numbers.

```bash
$ rosrun roboteq_python get_roboteq_id /dev/ttyACM0
42
```

Next we add a udev script in `/etc/udev/rules.d/` telling the kernel to run the
`get_roboteq_id` script and use the ID as part of the device name.  See the
included file `udev/99-roboteq.rules` for an example. That file assumes that
you have copied the `get_roboteq_id` to `/usr/local/bin/` so that udev knows
where to find it.
