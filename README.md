# Projektseminar Autonomes Fahren - tokyodrift
Dieses Repository enthält die Software für ein Modellauto, das sich autonom auf einem vorher unbekannten Rundkurs orientieren und Hindernisse kollisionsfrei umfahren kann.

## Abhängigkeiten
- ROS2 Humble
- OpenCV
- CV Bridge
- Eigen3
- PCL
- Colcon

## Bauen und Ausführen
- Zuerst müssen die initialen Projekt relevanten Packages gebaut werden:
```
colcon build --packages-select psaf_ucbridge psaf_ucbridge_msgs psaf_launch psaf_configuration
```
- Um das Projekt komplett zu bauen, kann anschließend folgender Befehl genutzt werden: 
```
colcon build --packages-select button_launch utility timer helpers lane_detection start_box sensor_filter depth_obstacle_detection lane_transform bezier_curve simulated_control uc_com
```
- Nach dem Bauen kann folgender Befehl genutzt werden, um das Projekt auszuführen:
```
source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=bezier use_foxglove:=false start_box:=false obstacle_detection_type:=none driving_speed:=fast driving_lane:=outer
```
- Außerdem können die Packages auch getestet werden:
```
colcon test --packages-select button_launch utility timer helpers lane_detection sensor_filter depth_obstacle_detection lane_transform bezier_curve simulated_control uc_com
```

## Auto
Josua Lindemann
