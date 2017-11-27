# refills_first_review
Configuration and launch-files for the use case scenarios of the first REFILLS review.

# general notes

- have a roscore running on homer
- use it in homer and amy: export ROS_MASTER_URI=http://homer.ai.loc:11311
- keep the emergency stop close to you at all times
- if there are obstacles close to the robot that appear on the laser scanners, the base won't move (safety feature of nav_pcontroller).
- check the voltage of the batteries after some hours disconnected, if the voltage is below 46V, connect the plug of the charger on a socket.
- charge the PS3 controller after use by connecting it to the mini-USB cable.
- if the images captured by the camera are too dark, check that the ring-light is getting power. Power is activated on the I/O tab of the UR5 controller, make sure it is set to 24VDC.
- If the EtherCAT does not work after a reboot, run: sudo /etc/init.d/ethercat restart
- After launching the bringup.launch script, remember to press "play" on the PS3 controller to release the soft-run-stop. Without it the base won't move.
- When using rviz, try to minimize data flow leaving donbot, as the wireless link has limited bandwidth. If the signal strength LEDs on the mikrotik WLAN device blink together, it is complaining about too much data for the wireless link. For example, always use compressed topics, or throttled topics, to get data out.

# how to run demo
bringup donbot on 1. terminal on homer:
``` 
source /home/stelter/homer_ws/devel/setup.bash
roslaunch iai_donbot_bringup bringup.launch
```

start barcode detector in an amy terminal:
``` 
source /home/stelter/refills_ws/devel/setup.bash
roslaunch barcode_detector refills_find_barcode.launch 
```

barcodes in camera image on local terminal:
``` 
rosrun image_view image_view image:=/refills_wrist_camera/barcodes_detected _image_transport:=compressed
```

Start demo in 2. terminal on homer:
``` 
source /home/stelter/homer_ws/devel/setup.bash
rosrun refills_first_review donbot_brain.py
```


