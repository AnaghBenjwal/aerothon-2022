from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time

vehicle = connect(ip = '127.0.0.1:14550', wait_ready=True, baud = 921600)

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
    print('VEHICLE MODE: ', vehicle.mode) 

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)
        	


arm_and_takeoff(10)
vehicle.mode = VehicleMode("LAND")
print('VEHICLE MODE: ', vehicle.mode) 

print("Returning to Launch")
while vehicle.armed :
	vehicle.mode = VehicleMode("LAND")
print('VEHICLE DISARMED') 
