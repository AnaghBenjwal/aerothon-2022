#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# 192.168.141.18:14551
"""

######################################################################
#   IMPORT LIBRARIES
######################################################################

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time
import math
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
# import rospy

######################################################################
#   GLOBAL VAIRABLES
######################################################################

global camereX
global cameraY
global centerX
global centerY
global hastargetbeendetected
global distance_from_waypoint
global listofwaypoints

listofwaypoints = ([13.344069, 74.793536], [13.343796, 74.793661], [13.343928, 74.793960], [13.344228, 74.793785])
cameraX = 640/2
cameraY = 480/2
centerX = None
centerY = None
hastargetbeendetected = False
distance_from_waypoint = 1000

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("PRINTMSG: Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("PRINTMSG: Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" PRINTMSG: Waiting for arming...")
        time.sleep(1)

    print("PRINTMSG: Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" PRINTMSG: Altitude: ", vehicle.location.global_relative_frame.alt)
        # Trigger just below target alt.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("PRINTMSG: Reached target altitude")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)


def navigate_to_waypoint(waypointcoord):
    """
    Navigates vehicle from current location to given waypoint while maintaining altitude
    location of waypont is given as metres of offset from present vehicle location
    """
    global distance_from_waypoint

    dlat = waypointcoord[0]
    dlon = waypointcoord[1]
    dalt = vehicle.location.global_frame.alt

    # relative_waypoint = get_location_metres(original_location, dnorth, deast)
    # relative_waypoint.alt = dalt
    # print('next waypoint: ', relative_waypoint)

    waypoint = LocationGlobal(dlat, dlon, dalt)
    distance_from_waypoint = get_distance_metres(waypoint, vehicle.location.global_frame)
    vehicle.simple_goto(waypoint, groundspeed=2)
    return


def target_detection(frame) :

    """
    Continuously run inference on images acquired from the camera.

    Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
    """

    global centerX
    global centerY
    global hastargetbeendetected

    if (frame is not None) :
        image = cv2.flip(frame, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = detector.detect(input_tensor)

        if len(detection_result.detections) > 0 :
            hastargetbeendetected == True
            centerX = detection_result.detections[0].bounding_box.origin_x
            centerY = detection_result.detections[0].bounding_box.origin_y
        
        else :
            hastargetbeendetected == False
            centerX == None
            centerY == None

    else : return 


def align_and_drop() :
    """
    aligns drone with center of detected target
    """

    global centerX
    global centerY
    global cameraX
    global cameraY

    print('PRINTMSG: beginning alignment')
    offsetX = (centerX - cameraX)**2
    offsetY = (centerY - cameraY)**2

    if (offsetY >= 100) or (offsetX >= 100) :
        print('PRINTMSG: target offset: ', (offsetX, offsetY))
        while (offsetX >= 100) or (offsetY >= 100) :
            ret, frame = cap.read()
            target_detection(frame)
            lenX = (centerX - cameraX)
            lenY = (centerY - cameraY)
            yaw_deg = math.atan2(lenY, lenX) * (180 / math.pi)
            if (yaw_deg < 0) : yaw_deg += 360

            if yaw_deg > 10 :
                print('PRINTMSG: yaw degree: ',yaw_deg)
                condition_yaw(yaw_deg, relative=True)
                time.sleep(3)
            
            send_ned_velocity(1,0,0)

    print('PRINTMSG: dropping servo')
    
    while vehicle.armed :
        vehicle.mode = VehicleMode("RTL")
   



vehicle = connect(ip='127.0.0.1:14550', wait_ready=True, baud=921600)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
cap = cv2.VideoCapture(0)

# Initialize the object detection model
base_options = core.BaseOptions(file_name='aerothon.tflite', use_coral=False, num_threads=4)
detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)

original_location = vehicle.location.global_frame
print('PRINTMSG: original location initialised: ', original_location)

arm_and_takeoff(10)

count = 0
for waypointcoord in listofwaypoints :
    print('PRINTMSG: next waypoint: ', waypointcoord)
    navigate_to_waypoint(waypointcoord)
    count += 1
    while (distance_from_waypoint >= 0.95) :
        ret, frame = cap.read()
        target_detection(frame)
        navigate_to_waypoint(waypointcoord)

        if hastargetbeendetected == True : 
            vehicle.mode = VehicleMode("BRAKE")
            print('PRINTMSG: VEHICLE MODE after detection: ', vehicle.mode)
            time.sleep(5)
            print('PRINTMSG: target has been detected at: ', (centerX, centerY))
            align_and_drop(frame)
            break
        if count%20 == 0 : print('PRITNMSG: distance from waypoint: ', distance_from_waypoint)
    
    print('PRITNMSG: reached waypoint')
    continue
    
print("PRINTMSG: Returning to Launch")
vehicle.mode = VehicleMode("RTL")
print('PRINTMSG: VEHICLE MODE: ', vehicle.mode) 

while vehicle.armed :
    vehicle.mode = VehicleMode("RTL")

print('PRINTMSG: VEHICLE DISARMED')