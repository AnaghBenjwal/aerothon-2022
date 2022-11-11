#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# 192.168.141.18:14551
"""

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time
import math

vehicle = connect(ip='127.0.0.1:14550', wait_ready=True, baud=921600)


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

    vehicle.simple_goto(relative_waypoint, groundspeed=2)

    distance_from_waypoint = get_distance_metres(
           relative_waypoint, vehicle.location.global_frame)
    while (distance_from_waypoint >= 1):
        distance_from_waypoint = get_distance_metres(
            relative_waypoint, vehicle.location.global_frame)
        vehicle.simple_goto(relative_waypoint, groundspeed=2)
        time.sleep(5)
        print('distance from waypoint: ', distance_from_waypoint)
    print('reached waypoint')


cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

original_location = vehicle.location.global_frame
print('original location initialised to: ', original_location)

arm_and_takeoff(10)
current_alt = vehicle.location.global_frame.alt
navigate_to_relative_waypoint(10, 0, current_alt)


#waypoint_location = LocationGlobalRelative(-35.361354, 149.165218, 10)
#vehicle.simple_goto(waypoint_location, groundspeed = 3)
# time.sleep(50)

print("Returning to Launch")
while True:
    vehicle.mode = VehicleMode("RTL")
