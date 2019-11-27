# Lidar to Gridmap

Simple simulation to simulate 2d mapping and flight rerout to the shortest path using posisition and lidar data.

This program is developed using python3 on Debian Buster. You can install python3 using:
```sh
sudo apt update
sudo apt install python3.7
```
Install the dependencies using pip.
```sh
sudo apt update
sudo apt install python3-pip python3-tk
sudo pip3 install numpy matplotlib
```

There are 2 files as an input LIDARPoints.csv and FlightPath.csv
file format: 
 - LIDARPoints.csv: The first line has the scan ID and number
of data lines (number of recorded points for that sweep). Each following line has the
angle of the data point (in degrees) and the distance (in millimeters) until the next
scan ID header line.
 - The first line has the scan ID and number of data line (always 1). The next
line is the X,Y location of the drone in meters

make sure that the 2 files always exist same path as the script

Assumed that the robot is non-holonomic robot that always head paralel to X axis

to run the simulation use
```sh
python3 lidar_to_grid_map.py
```
