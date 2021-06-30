#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Disable "Bare exception" warning
# pylint: disable=W0702

from __future__ import print_function
import time
import math
import sys
# Import mavutil
from pymavlink import mavutil
from pymavlink import mavwp

class location:
    '''represent a GPS coordinate'''
    def __init__(self, lat, lng, alt=0, heading=0):
        self.lat = lat
        self.lng = lng
        self.alt = alt
        self.heading = heading

    def __str__(self):
        return "lat=%.6f,lon=%.6f,alt=%.1f" % (self.lat, self.lng, self.alt)


# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
connection_string = '127.0.0.1:14551' #'tcp:127.0.0.1:14550' #args.connect

myDrone = mavutil.mavlink_connection(connection_string)
wpWizard = mavwp.MAVWPLoader()
# Get some information !
# while True:
#     try:
#         print(myDrone.recv_match().to_dict())
#     except:
#         pass
#     time.sleep(0.1)


### Function Dictionary ###
def arm_sub(myDrone_connection):
    # Wait a heartbeat before sending commands
    myDrone_connection.wait_heartbeat()

    # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

    # Arm
    # master.arducopter_arm() or:
    myDrone_connection.mav.command_long_send(
        myDrone_connection.target_system,
        myDrone_connection
        .target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
def disarm_sub(myDrone_connection):
    # Wait a heartbeat before sending commands
    myDrone_connection.wait_heartbeat()

    # https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
def change_mode(myDrone_connection, mode):
    # Choose a mode
    # mode = 'STABILIZE'

    # Check if mode is available
    if mode not in myDrone_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(myDrone_connection.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = myDrone_connection.mode_mapping()[mode]
    # Set new mode
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # master.set_mode(mode_id) or:
    myDrone_connection.mav.set_mode_send(
        myDrone_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    while True:
        # Wait for ACK command
        ack_msg = myDrone_connection.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break
def get_myLocation(myDrone_connection):
    '''return current location'''
    relative_alt=False
    myDrone_connection.wait_gps_fix()
    # wait for another VFR_HUD, to ensure we have correct altitude
    myDrone_connection.recv_match(type='VFR_HUD', blocking=True)
    myDrone_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if relative_alt:
        alt = myDrone_connection.messages['GLOBAL_POSITION_INT'].relative_alt*0.001
    else:
        alt = myDrone_connection.messages['VFR_HUD'].alt
    return location(myDrone_connection.messages['GPS_RAW_INT'].lat*1.0e-7,
                    myDrone_connection.messages['GPS_RAW_INT'].lon*1.0e-7,
                    alt,
                    myDrone_connection.messages['VFR_HUD'].heading)
def get_location_metres(myDrone_connection, original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = get_myLocation(myDrone_connection).lat + (dLat * 180/math.pi)
    newlon = get_myLocation(myDrone_connection).lng + (dLon * 180/math.pi)
    return location(newlat,
                    newlon,
                    alt,
                    myDrone_connection.messages['VFR_HUD'].heading)
def logData(myDrone_connection, file, start_time):
    # Open function to open the file "MyFile1.txt"
    # (same directory) in append mode and
    run_time = time.time() - start_time
    lat = get_myLocation(myDrone_connection).lat;
    lon = get_myLocation(myDrone_connection).lng;
    alt = get_myLocation(myDrone_connection).alt;
    L =str(run_time) + ", " + str(lat) + ", " + str(lon) + ", " + str(alt)+ "\n"
    file.writelines(L)

# def distance_to_current_waypoint():
#     """
#     Gets distance in metres to the current waypoint.
#     It returns None for the first waypoint (Home location).
#     """
#     nextwaypoint = vehicle.commands.next
#     if nextwaypoint==0:
#         return None
#     missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
#     lat = missionitem.x
#     lon = missionitem.y
#     alt = missionitem.z
#     targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
#     distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
#     return distancetopoint
#
#
# def get_distance_metres(aLocation1, aLocation2):
#     """
#     Returns the ground distance in metres between two LocationGlobal objects.
#
#     This method is an approximation, and will not be accurate over large distances and close to the
#     earth's poles. It comes from the ArduPilot test code:
#     https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
#     """
#     dlat = aLocation2.lat - aLocation1.lat
#     dlong = aLocation2.lon - aLocation1.lon
#     return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
# def distance_to_current_waypoint():
#     """
#     Gets distance in metres to the current waypoint.
#     It returns None for the first waypoint (Home location).
#     """
#     nextwaypoint = vehicle.commands.next
#     if nextwaypoint==0:
#         return None
#     missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
#     lat = missionitem.x
#     lon = missionitem.y
#     alt = missionitem.z
#     targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
#     distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
#     return distancetopoint
# def travel_updates():
#     threshold = 5;
#     travel_time = 0;
#     dt = 5;
#     d = distance_to_current_waypoint()
#     if d != None:
#         if d > threshold:
#             print("Distance to WP: ", distance_to_current_waypoint(), " || Depth: ", vehicle.location.global_relative_frame.alt)
#             d = distance_to_current_waypoint()
#         else:
#             print("Within", threshold, "m of wp. Travel time: ", travel_time)
#     else:
#         print("Aquiring waypoint...")
# def adds_square_mission(aLocation, aSize, depth):
#     """
#     Adds a takeoff command and four waypoint commands to the current mission.
#     The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
#
#     The function assumes vehicle.commands matches the vehicle mission state
#     (you must have called download at least once in the session and after clearing the mission)
#     """
#
#     cmds = vehicle.commands
#
#     print(" Clear any existing commands")
#     cmds.clear()
#
#     print(" Define/add new commands.")
#     # Add new commands. The meaning/order of the parameters is documented in the Command class.
#
#     #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0,depth))
#
#     #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
#     point1 = get_location_metres(aLocation, aSize, -aSize)
#     point2 = get_location_metres(aLocation, aSize, aSize)
#     point3 = get_location_metres(aLocation, -aSize, aSize)
#     point4 = get_location_metres(aLocation, -aSize, -aSize)
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, depth))
#     #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, depth))
#
#     print(" Upload new commands to vehicle")
#     cmds.upload()
# def adds_grid_mission(aLocation, aSize, depth):
#     """
#     Adds a takeoff command and four waypoint commands to the current mission.
#     The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
#
#     The function assumes vehicle.commands matches the vehicle mission state
#     (you must have called download at least once in the session and after clearing the mission)
#     """
#
#     cmds = vehicle.commands
#
#     print(" Clear any existing commands")
#     cmds.clear()
#
#     print(" Define/add new commands.")
#     # Add new commands. The meaning/order of the parameters is documented in the Command class.
#
#     #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
#
#     #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
#     point0 = get_location_metres(aLocation, 0, 0)
#     point1 = get_location_metres(aLocation, aSize, 0)
#     point2 = get_location_metres(aLocation, aSize, aSize)
#     point3 = get_location_metres(aLocation, 2*aSize, aSize)
#     point4 = get_location_metres(aLocation, 2*aSize, 0)
#     point5 = get_location_metres(aLocation, 3*aSize, 0)
#     point6 = get_location_metres(aLocation, 3*aSize, aSize)
#     point7 = get_location_metres(aLocation, 4*aSize, aSize)
#     point8 = get_location_metres(aLocation, 4*aSize, 0)
#     point9 = get_location_metres(aLocation, 5*aSize, 0)
#
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point0.lat, point1.lon, 0))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point5.lat, point5.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point6.lat, point6.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point7.lat, point7.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point8.lat, point8.lon, depth))
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point9.lat, point9.lon, depth))
#
#     #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
#     cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point9.lat, point9.lon, depth))
#
#     print(" Upload new commands to vehicle")
#     cmds.upload()
# def timed_logData(file, duration):
#     count = 0
#     dt = 0.01
#     while count < duration:
#         logData(file,mission_a_go,time_start)
#         time.sleep(dt)
#         count += dt


# Wait a heartbeat before sending commands
myDrone.wait_heartbeat()

### Point Home ###
home_lat = get_myLocation(myDrone).lat;
home_long = get_myLocation(myDrone).lng;
home_alt = get_myLocation(myDrone).alt;

### Show home location ###
print(" Altitude: ", home_alt)
print(" Home: ", home_lat , ", ", home_long)
# Break and return from function just below target altitude.
time.sleep(1)

arm_sub(myDrone)
print("Sub ready!")
change_mode(myDrone, 'STABILIZE')
# print("Set default/target speed...")
# vehicle.groundspeed = 10
#
print("Creating mission (for current location)... # of WP:")
wpWizard.clear()
wpWizard.load('../mission_grid1.txt')
numWP = wpWizard.count()
print(wpWizard.count())
#
#
# print("Starting mission")
# mission_a_go = True
time_start = time.time()
# # Reset mission set to first (0) waypoint
# vehicle.commands.next=0
#
# # Set mode to AUTO to start mission
change_mode(myDrone, 'AUTO')
file = open("log.txt","a")
file.truncate(0)
#
while True:
    # travel_updates()
    logData(myDrone,file,time_start)
    nextwaypoint=vehicle.commands.next

    if nextwaypoint==10: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print("Final waypoint reached!")
        break;

# vehicle.groundspeed = 15
# vehicle.mode = VehicleMode("CIRCLE")
# timed_logData(file, 180)
#
# print("Surfacing...")
# vehicle.mode = 100
# while vehicle.location.global_relative_frame.alt < -1:
#     # print("Surfacing...", vehicle.location.global_relative_frame.alt)
#     logData(file,mission_a_go,time_start)
#     # time.sleep(5)
#
# print("Surfaced and Maintaining Position")
# # Close vehicle object before exiting script
# mission_a_go = False
# file.close()
# print("Close vehicle object")
# vehicle.close()
#
# # Shut down simulator if it was started.
# if sitl:
#     sitl.stop()
