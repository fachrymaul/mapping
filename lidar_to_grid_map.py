"""

LIDAR to 2D grid map example

author: Erno Horvath, Csaba Hajdu based on Atsushi Sakai's scripts (@Atsushi_twi)

"""

# import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import a_star

EXTEND_AREA = 1.0


def lidar_file_read(lidar):
    """
    Reading LIDAR laser beams (angles and corresponding distance data)
    input: 
        lidar: path to lidar file
    output:
        angles: lidar angle data
        distance: lidar distance data
    """
    measures = [line.split(",") for line in open(lidar)]
    scanId = 0
    dataLen = 0
    angles = []
    distances = []
    counter = 0
    state = 0
    for measure in measures:
        if state == 0:
            scanId = int(measure[0])
            dataLen = int(measure[1])
            angles.append([0] * dataLen)
            distances.append([0] * dataLen)
            state = 1
            counter = 0
        elif state == 1:
            if counter < dataLen - 1:
                angles[scanId][counter] = float(measure[0])
                distances[scanId][counter] = float(measure[1]) / 1000
                counter += 1
            else:
                state = 0
    return angles, distances

def flight_file_read(flight):
    """
    Reading flight path file
    input:
        flight: path to flight pathfile
    output:
        x: drone x position
        y: drone y position
    """
    points = [line.split(",") for line in open(flight)]
    scanId = 0
    dataLen = 0
    x = []
    y = []
    counter = 0
    state = 0
    for point in points:
        if state == 0:
            state = 1
        elif state == 1:
            x.append(float(point[0]))
            y.append(float(point[1]))
            state = 0
    x = np.array(x)
    y = np.array(y)
    return x, y

def write_new_flight_path(rx, ry):
    """
    write new flight path to newFlightPath.csv
    input:
        rx: drone new x position
        ry: drone new y position
    output:
        newFlightPath.csv: drone new flight path file
    """
    f= open("newFlightPath.csv", "w+")
    i = 0
    for (x,y) in zip(np.flipud(rx),np.flipud(ry)):
        scanid = "%d,1\n\r" % i
        data = "%5.5f,%5.5f\n\r" % (float(x), float(y))
        f.write(scanid)
        f.write(data)
        i += 1
    f.close()
    print("newFlightPath.csv is saved")

def mapping(ang, dist, dronex, droney):
    """
    collecting lidar projection data and plot
    input:
        ang: lidar angle data(deg)
        dist; lidar distance data per angle (mm)
        dronex: drone x position
        droney: drone y position
    output:
        tx: lidar x projection data
        ty: lidar y projection data
    """
    tx = []
    ty = []
    counter = 0
    for (cx, cy) in zip(dronex, droney):
        angle = np.array(ang[counter])
        distance = np.array(dist[counter])
        dotx = []
        doty = []
        for (d,a) in zip(distance, angle):
            if d != 0:
                dotx = np.append(dotx, cx + (math.cos(a * math.pi / 180.) * d))
                doty = np.append(doty, cy - (math.sin(a * math.pi / 180.) * d))
        tx = np.append(tx, dotx)
        ty = np.append(ty, doty)
        counter += 1
        plt.figure(1, figsize=(8,8))
        plt.cla()
        plt.plot(tx, ty, "ro")
        plt.axis("auto")
        plt.plot(cx,cy, "ob")
        plt.gca().set_aspect("equal", "box")
        plt.gca().invert_yaxis()
        bottom, top = plt.ylim()  # return the current ylim
        plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
        plt.grid(True)
        plt.pause(0.01)
    return tx, ty

def path_finding(tx, ty, dronex, droney):
    """
    A star path finding and plot new route
    input;
        tx: lidar projection x coordinat [m]
        ty: lidar projection y coordinat [m]
        dronex: drone position x coordinate [m]
        droney: drone position y coordinate [m]
    output:
        rx: new x position path
        rx: new y position path
    """
    sx = dronex[0]
    sy = droney[0]
    gx = dronex[-1]
    gy = droney[-1]
    plt.figure(1, figsize=(8,8))
    plt.cla()
    plt.plot(tx, ty, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    astar = a_star.AStarPlanner(tx, ty, 0.3, 0.3)
    rx, ry = astar.planning(sx,sy,gx,gy)
    plt.plot(rx, ry, "-r")

    return rx, ry


def main():
    print(__file__, "start")
    
    ang, dist = lidar_file_read("LIDARPoints.csv")
    dronex, droney = flight_file_read("FlightPath.csv")

    tx, ty = mapping(ang, dist, dronex, droney)
    rx, ry = path_finding(tx, ty, dronex, droney)
    write_new_flight_path(rx, ry)

    plt.show()

if __name__ == '__main__':
    main()
