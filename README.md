# Projektseminar Autonomes Fahren WiSe 2024-25 - tokyodrift
Dieses Repository enthält die Software für ein Modellauto, das sich autonom auf einem vorher unbekannten Rundkurs orientieren und Hindernisse kollisionsfrei umfahren kann.
Das Projekt nutzt dabei OpenCV für die Bildverarbeitung und ROS2 als Betriebssystem für das Auto. Des Weiteren nutzt dieses Projekt einen ORB3-SLAM für die Hindernisumfahrung.

## Abhängigkeiten
- ROS2 Humble
- OpenCV
- CV Bridge
- ORB3 SLAM
- Eigen3
- PCL
- Colcon

## Struktur
Das Projekt ist unterteilt in vier Haupt-Teile:
- Common: Enthält überwiegend Helper-Methoden und -Klassen, die von allen anderen Teilen des Codes verwendet werden. Außerdem sind hier die für dieses Projekt individuellen ROS2-Messages definiert.
- Sense: Enthält alle Klassen, die mit den Sensoren des Modellautos interagieren - darunter Lane Detection, Start-Box Detection, Sensorfilter, Depth Obstacle Detection und Obstacle Detection.
- Plan: Enthält alle Klassen, die für die Planung der Trajektorie des Autos zuständig sind, d.h. die Pure-Pursuit-Steuerung (aufgeteilt in Lateral und Longitudinal Control), die Implementierung des ORB3-SLAMs und die Bezier-Curve-Berechnung.
- Act/UC-com: Enthält die Teile des Codes, die die Kommunikation zwischen Plan und der tatsächlichen Hardware des Autos regeln. Hier wird bspw. die von der Pure-Pursuit-Steuerung berechnete Geschwindigkeit in einen Zahlenwert zwischen 0 und 1000 umgerechnet und dann an das UC-Board mitgeteilt.

Die Kommunikation zwischen den einzelnen sog. Nodes findet mittels ROS2-Messages statt.

## Bauen und Ausführen
- Zuerst müssen die initialen Projekt relevanten Packages gebaut werden:
```
colcon build --packages-select psaf_ucbridge psaf_ucbridge_msgs psaf_launch psaf_configuration
```
- Um das Projekt komplett zu bauen, kann anschließend folgender Befehl genutzt werden: 
```
colcon build --packages-select button_launch utility timer helpers lane_detection start_box sensor_filter depth_obstacle_detection obstacle_detection lane_transform purepursuite bezier_curve simulated_control uc_com global_obstacle_avoidance
```
- Nach dem Bauen kann folgender Befehl genutzt werden, um das Projekt auszuführen:
```
source install/setup.bash && ros2 launch utility tokyodrift.launch.py controller_type:=purepursuit use_foxglove:=false start_box:=false obstacle_detection_type:=none driving_speed:=slow driving_lane:=outer
```
- Außerdem können die Packages auch getestet werden:
```
colcon test --packages-select button_launch utility timer helpers lane_detection start_box sensor_filter depth_obstacle_detection obstacle_detection lane_transform purepursuite bezier_curve simulated_control uc_com global_obstacle_avoidance
```

## Funktionalität
Im Rahmen eines Tests wurde die Strecke, die im Video zu sehen ist, in einer Zeit von 9,8s durchfahren. Des Weiteren konnte das Auto auch mit einigen Hindernissen kollisionsfrei mehrere Runden lang die Strecke absolvieren.

![Gif not available](parcours-optimized.gif)




## Mögliche Erweiterungen
Die Lane Detection könnte so erweitert werden, dass sie auch in schlechten Lichtverhältnissen weiterhin robust funktioniert. Außerdem könnte die Hindernisumfahrung insofern erweitert werden,
dass das Auto auch dynamische Objekte umfahren kann. Zu guter Letzt könnte der aufgrund fehlender Zeit nicht vollständig umgesetzte Ansatz für einen MPC (Model Predictive Controller) fertig implementiert werden.

## Autoren
Josua Lindemann, Cedric Strobel, Raphael Capezzuto, Asaria Worku, Aaron Wickert

## Status des Projekts
Da dieses Repository im Rahmen eines Projektseminars erstellt und dieses Projektseminar erfolgreich beendet wurde, wird an diesem Projekt nicht mehr aktiv gearbeitet. 
