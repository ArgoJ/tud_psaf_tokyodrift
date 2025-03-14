# Helpers
Dieses Package ist ein Helper Package für andere Packages welches nur header beinhaltet.

### filter:
Implementiert eine Klasse eines Lowpassfilters.

### integrator:
Implementiert eine Integrator Klasse, welche das Bicycle Model eine Runge Kutta 4. Ordnung Integration nutzt. 

### kalman_estimator:
Implementiert einen Kalman Estimator mit voreingestellten Werten für die Standardabweichung der drei Sensoren (Hall, Rotation Acceleration Z, Linear Acceleration X)

### marker_publisher:
Stellt Funktionen bereit um Marker kompakt über verschiedene Frames zu publishen. 

### point_msg_helper:
Fasst einige Funktionen von Euklidischer Distanz sowie Purepursuit relevante functionen für Vektoren von Points. Außerdem auch Trajektorien Smoothing und Verschiebung.