# refills_first_review
Configuration and launch-files for the use case scenarios of the first REFILLS review.

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


