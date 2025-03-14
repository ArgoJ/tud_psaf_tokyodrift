# Button Launch
This button node listens to the buttons and starts a specific launch based on the config strings.
If pressed once, the desired launch starts and if pressed again it stops. If another Button is pressed once, the first launch stops and the new launch starts.
If pressed to fast after each other, this procedure leads to issues. Here it does not correctly stops the launch and leaves nodes running. Which means some nodes can be running multiple times.


## Node Parameters
- *first* - The options for the button_tokyodrift.launch.py as a string for button A.
- *second* - The options for the button_tokyodrift.launch.py as a string for button B.
- *third* - The options for the button_tokyodrift.launch.py as a string for button C.
