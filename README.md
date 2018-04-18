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
- unplug donbot
- turn ur5 on and set payload to 3 kg
- on homer: roscore
- on homer: roslaunch refills_first_review bringup_everything.launch
- on amy: set homer to master and roslaunch refills_first_review amy.launch
- on your laptop: rviz
- on your laptop: rosrun refills_mock_gui mock_gui.py
- on homer: rosrun refills_first_review donbot_brain.py 
the brain requires some inputs. everything other than a 'y' counts as a no
```
refills@homer:~/workspace/rs_ws/src/refills_first_review (master)$ rosrun refills_first_review donbot_brain.py 
[WARN] [1524051428.975111]: robosherlock not fully integrated
[INFO] [1524051428.994867]: connecting to nav_pcontroller/move_base ...
[INFO] [1524051430.502880]: connected to nav_pcontroller/move_base
[INFO] [1524051431.013499]: connecting to /qp_controller/command
[INFO] [1524051431.296206]: connected to /qp_controller/command
start demo? [y]y
[INFO] [1524051434.464302]: starting REFILLS scenario 1 demo
[INFO] [1524051434.464631]: shelf baseboard detection requires manuel mode
[INFO] [1524051434.464873]: move to free space plx
done? [y]y
add new shelf system? [y]y
before lock
sending belief_new_object(dmshop:'DMShelfSystem', R), rdf_assert(R, knowrob:describedInMap, iaishop:'IAIShop_0', belief_state)
solutions [{u'R': u"'http://knowrob.org/kb/dm-market.owl#DMShelfSystem_DLPXCBMI'"}]
----------------------
[INFO] [1524051440.467542]: added shelf system http://knowrob.org/kb/dm-market.owl#DMShelfSystem_DLPXCBMI
[INFO] [1524051440.467924]: moving arm to baseboard scanning pose
[INFO] [1524051456.623886]: scan all shelf baseboard plx
finished scanning shelf system? [y]
```
at this point you have two options:
- take the controller and scan the qr codes, the terminal will tell you if a shelf was successfully added. y+enter to continue
- write 1337+enter to skip the qr code scanning and add shelfs at the right positions
- you could also scan the qr codes and type 1337. this will only add the perfect shelf positions.

```
finished scanning shelf system? [y]1337
[WARN] [1524051628.017757]: skipping baseboard detection
[INFO] [1524051628.018337]: added shelf system 0
[INFO] [1524051628.018541]: shelf system 0 complete
[INFO] [1524051628.018726]: added shelf system 1
[INFO] [1524051628.018918]: shelf system 1 complete
[INFO] [1524051628.019098]: added shelf system 2
[INFO] [1524051628.019276]: shelf system 2 complete
[INFO] [1524051628.019448]: added shelf system 3
[INFO] [1524051628.019628]: shelf system 3 complete
before lock
sending belief_new_object(dmshop:'DMShelfFrameFrontStore', ID), rdf_assert('http://knowrob.org/kb/dm-market.owl#DMShelfSystem_DLPXCBMI', knowrob:properPhysicalParts, ID, belief_state),object_affordance_static_transform(ID, A, [_,_,T,R]),rdfs_individual_of(A, dmshop:'DMShelfPerceptionAffordance')
solutions [{u'A': u"'http://knowrob.org/kb/dm-market.owl#DMShelfPerceptionAffordanceFrameFront_JCXYGROK'", u'R': [0, 0, 0, 1], u'T': [-0.5, -0.313126, -0.8705], u'ID': u"'http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_ZDOBXQRG'"}]
----------------------
before lock
sending belief_at_update('http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_ZDOBXQRG', ['map', _, [1.1675,-0.276874,0.8705], [0.0,0.0,-0.0,1.0]])
solutions [True]
----------------------
before lock
sending belief_new_object(dmshop:'DMShelfFrameFrontStore', ID), rdf_assert('http://knowrob.org/kb/dm-market.owl#DMShelfSystem_DLPXCBMI', knowrob:properPhysicalParts, ID, belief_state),object_affordance_static_transform(ID, A, [_,_,T,R]),rdfs_individual_of(A, dmshop:'DMShelfPerceptionAffordance')
solutions [{u'A': u"'http://knowrob.org/kb/dm-market.owl#DMShelfPerceptionAffordanceFrameFront_NHOFPRJV'", u'R': [0, 0, 0, 1], u'T': [-0.5, -0.313126, -0.8705], u'ID': u"'http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_JWSERDIC'"}]
----------------------
before lock
sending belief_at_update('http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_JWSERDIC', ['map', _, [2.165,-0.276874,0.8705], [0.0,0.0,-0.0,1.0]])
solutions [True]
----------------------
before lock
sending belief_new_object(dmshop:'DMShelfFrameFrontStore', ID), rdf_assert('http://knowrob.org/kb/dm-market.owl#DMShelfSystem_DLPXCBMI', knowrob:properPhysicalParts, ID, belief_state),object_affordance_static_transform(ID, A, [_,_,T,R]),rdfs_individual_of(A, dmshop:'DMShelfPerceptionAffordance')
solutions [{u'A': u"'http://knowrob.org/kb/dm-market.owl#DMShelfPerceptionAffordanceFrameFront_MESTDCGB'", u'R': [0, 0, 0, 1], u'T': [-0.5, -0.313126, -0.8705], u'ID': u"'http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_XREDIQGU'"}]
----------------------
before lock
sending belief_at_update('http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_XREDIQGU', ['map', _, [3.165,-0.276874,0.8705], [0.0,0.0,-0.0,1.0]])
solutions [True]
----------------------
before lock
sending belief_new_object(dmshop:'DMShelfFrameFrontStore', ID), rdf_assert('http://knowrob.org/kb/dm-market.owl#DMShelfSystem_DLPXCBMI', knowrob:properPhysicalParts, ID, belief_state),object_affordance_static_transform(ID, A, [_,_,T,R]),rdfs_individual_of(A, dmshop:'DMShelfPerceptionAffordance')
solutions [{u'A': u"'http://knowrob.org/kb/dm-market.owl#DMShelfPerceptionAffordanceFrameFront_RBOWPNDU'", u'R': [0, 0, 0, 1], u'T': [-0.5, -0.313126, -0.8705], u'ID': u"'http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_PMSBLUFH'"}]
----------------------
before lock
sending belief_at_update('http://knowrob.org/kb/dm-market.owl#DMShelfFrameFrontStore_PMSBLUFH', ['map', _, [4.165,-0.276874,0.8705], [0.0,0.0,-0.0,1.0]])
solutions [True]
----------------------
add new shelf system? [y]
[INFO] [1524051632.014475]: MAKE SURE NOTHING IS CLOSE!!!!11elf
rdy? [y]y
before lock
sending cram_start_situation('http://knowrob.org/kb/knowrob.owl#RobotExperiment', '1524051644.06', R), rdf_assert(R,knowrob:performedBy,donbot:iai_donbot_robot1, 'LoggingGraph'), rdf_assert(R,knowrob:performedInMap,iaishop:'IAIShop_0', 'LoggingGraph')
solutions [{u'R': u"'http://knowrob.org/kb/knowrob.owl#RobotExperiment_HYGBWEMJ'"}]
----------------------
[INFO] [1524051644.078035]: waiting for scanning action goal
```

now use the gui to scan the shit out of the shelves.

start barcode detector in an amy terminal:
``` 
source /home/stelter/refills_ws/devel/setup.bash
roslaunch barcode_detector refills_find_barcode.launch 
```

# known problems
- sometimes the ring light does not turn on/off. It has to be turned on for the qr/barcode scanning. You can manuelly turning it on/off by calling the /ring_light_switch/setbool service.
- in rare cases the script does not continue for no apperent reason. Just wait it usually recovers after a few minutes.
- if you scan the same shelf twice shit will break, even if the first try failed for some reason. Go for a different one or restart the bringup launch file.
- restarting the bringup launch file will clear knowrob.
- in rare cases, especially if the shit is running for a long time, the barcode detector might go into a weired state where he does not detect barcodes anymore. just restart the shit, even if the demo is running, it should crash anything.
- the error "[WARN] [1524052005.585685]: failed to export logs, IO error" can be ignored .
