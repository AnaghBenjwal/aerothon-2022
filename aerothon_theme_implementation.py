#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# 192.168.141.18:14551
"""

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time
import math
import cv2


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Trigger just below target alt.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def navigate_to_relative_waypoint(dnorth, deast, dalt):
    """
    Navigates vehicle from ccurrent location to a given waypoint
    location of waypont is given as metres of offset from present vehicle location
    """
    relative_waypoint = get_location_metres(original_location, dnorth, deast)
    relative_waypoint.alt = dalt
    print('next waypoint: ', relative_waypoint)
     # relative_waypoint = LocationGlobalRelative(relative_waypoint.lat, relative_waypoint.lon, dalt)

    vehicle.simple_goto(relative_waypoint, groundspeed=5)

    # distance_from_waypoint = get_distance_metres(
    #        relative_waypoint, vehicle.location.global_frame)
    # while (distance_from_waypoint >= 0.9):
    #     distance_from_waypoint = get_distance_metres(
    #         relative_waypoint, vehicle.location.global_frame)
    #     vehicle.simple_goto(relative_waypoint, groundspeed=5)
    #     time.sleep(0.5)
    #     print('distance from waypoint: ', distance_from_waypoint)
    


def target_detection(frame) :
    """
    detects target area and returns center coords X and Y of detected target.
    """
    imgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 185, 255, cv2.THRESH_BINARY_INV)

    # edges = cv2.Canny(imgray, 127, 255)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    print('len(contours): ', len(contours))
    print('len(contuors[0]: ', len(contours[0]))

    centers = []
    for contour in contours:
        # compute the center of the contour
        M = cv2.moments(contour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centers.append([cx, cy])
        cv2.circle(frame, (cx, cy), 3, (0, 0, 0), -1)
        # approx = cv2.approxPolyDP(contour, epsilon, True)
        # print(approx)

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    # print('len(centers): ', len(centers))
    # print('centers: ', centers)
    # for center in centers:
    #     print('center: ', center)

    centerX = 0
    centerY = 0
    for center in centers:
        centerX += int((center[0]/len(centers)))
        centerY += int((center[1]/len(centers)))


    # cv2.imshow('img', img)
    hastargetbeendetected = True
    return centerX, centerY

    
        
def alignment(frame, centerX, centerY, cameraX, cameraY) :
    """
    aligns drone with center of detected target
    M1 - give drone velocities to bring center of target to center of camera frame
    M2 - recursively give drone waypoints corresponsing to target center location
    """

    ####  M1  ####
    # while (((centerX - camera_resolution/2)**2 >= 1) == False) or (((centerY - camera_resolution/2)**2) >= 1 == False) :
    #give drone necessary velocity for each of 4 conditions

    if hastargetbeendetected == True :
        if (centerX > cameraX/2) and (centerY > cameraY/2) :
            print('velocity - west south')
            
        elif (centerX > cameraX) and (centerY < cameraY) :
            print('velocty - west north')

        elif (centerX < cameraX) and (centerY > cameraY) :
            print('velocity - east south')

        elif (centerX < cameraX) and (centerY < cameraY) :
            print('velocity - east north')
        
        #servo_drop()
        return

    else : 
        return

    ####  M2  ####
    # centercoordinates = target_detection(frame)
    # get_distance_metres(centercoordinates, vehicle.location.global_frame)
    # vehicle.simple_goto()





global camereX
global cameraY
global hastargetbeendetected
global listofwaypoints

listofwaypoints = []
cameraX = 1920
cameraY = 1080
hastargetbeendetected = False

vehicle = connect(ip='127.0.0.1:14550', wait_ready=True, baud=921600)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
cap = cv2.VideoCapture(0)

original_location = vehicle.location.global_frame
print('original location initialised to: ', original_location)

arm_and_takeoff(10)

while vehicle.armed :
    ret, frame = cap.read()
    for relative_waypoint in listofwaypoints :
        navigate_to_relative_waypoint(relative_waypoint)
        distance_from_waypoint = get_distance_metres(relative_waypoint, vehicle.location.glboal_frame)
        while (distance_from_waypoint >= 0.95) :
            centerX, centerY = target_detection(frame)
            alignment(frame, centerX, centerY)
            distance_from_waypoint = get_distance_metres(relative_waypoint, vehicle.location.glboal_frame)
            navigate_to_relative_waypoint(relative_waypoint)
        
        print('reached waypoint')
        continue
    
        # current_alt = vehicle.location.global_frame.alt
        # distance_from_waypoint = get_distance_metres(relative_waypoint, vehicle.location.glboal_frame)
        # while (distance_from_waypoint >= 0.9):
        # distance_from_waypoint = get_distance_metres(relative_waypoint, vehicle.location.global_frame)
        # vehicle.simple_goto(relative_waypoint, groundspeed=5)
        # time.sleep(0.5)
        # print('distance from waypoint: ', distance_from_waypoint)
        # navigate_to_relative_waypoint(15, 0, current_alt)
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    print('VEHICLE MODE: ', vehicle.mode) 

    while vehicle.armed :
        vehicle.mode = VehicleMode("RTL")

print('VEHICLE DISARMED')

#waypoint_location = LocationGlobalRelative(-35.361354, 149.165218, 10)
#vehicle.simple_goto(waypoint_location, groundspeed = 3)
# time.sleep(50)


