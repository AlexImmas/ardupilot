#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Disable "Bare exception" warning
# pylint: disable=W0702

from __future__ import print_function
import time
import math
import sys
import matplotlib.pyplot as plt
# Import mavutil
from pymavlink import mavutil
from pymavlink import mavwp

class location(object):
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

connection_string = '127.0.0.1:14551'#'tcp:127.0.0.1:5760' #args.connect

myDrone = mavutil.mavlink_connection(connection_string)
print("Connected to drone!")
wpWiz = mavwp.MAVWPLoader()

latDat = [];
lonDat = [];
####################### Function Dictionary ######################
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
    myDrone_connection.mav.command_long_send(
        myDrone_connection.target_system,
        myDrone_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
def change_mode(mode, myDrone_connection):
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
def myLocation(myDrone_connection):
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
def delta_WPgen(myDrone_connection, d, brng, depth):
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
    # earth_radius=637816000 #cm or 6378137.0 m = Radius of "spherical" earth
    # #Coordinate offsets in radians
    # dLat = dNorth/earth_radius
    # dLon = dEast/(earth_radius*math.cos(math.pi*myLocation(myDrone_connection).lat/180))
    #
    # #New position in decimal degrees
    # newlat = myLocation(myDrone_connection).lat + (dLat * 180/math.pi)
    # newlon = myLocation(myDrone_connection).lng + (dLon * 180/math.pi)
    newlat = delta_WPloc(myDrone_connection, d, brng)[0]
    newlon = delta_WPloc(myDrone_connection, d, brng)[1]
    return add_latlonalt(newlat, newlon, depth, terrain_alt=False)
def delta_WPloc(myDrone_connection, d, brng):
    R = 6378160.00 #Radius of the Earth in m
    brng = math.radians(brng) #Bearing is 90 degrees converted to radians.

    #lat2  52.20444 - the lat result I'm hoping for
    #lon2  0.36056 - the long result I'm hoping for.

    lat1 = math.radians(myLocation(myDrone_connection).lat) #Current lat point converted to radians
    lon1 = math.radians(myLocation(myDrone_connection).lng) #Current long point converted to radians

    lat2 = math.asin( math.sin(lat1)*math.cos(d/R) +
         math.cos(lat1)*math.sin(d/R)*math.cos(brng))

    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat1),
                 math.cos(d/R)-math.sin(lat1)*math.sin(lat2))

    newlat = math.degrees(lat2)
    newlon = math.degrees(lon2)

    # earth_radius=637816000 #cm or 6378137.0 m = Radius of "spherical" earth
    # #Coordinate offsets in radians
    # dLat = dNorth/earth_radius
    # dLon = dEast/(earth_radius*math.cos(math.pi*myLocation(myDrone_connection).lat/180))
    #
    # #New position in decimal degrees
    # newlat = myLocation(myDrone_connection).lat + (dLat * 180/math.pi)
    # newlon = myLocation(myDrone_connection).lng + (dLon * 180/math.pi)
    return [newlat, newlon]
def logData(myDrone_connection, file, start_time):
    # Open function to open the file "MyFile1.txt"
    # (same directory) in append mode and
    run_time = time.time() - start_time
    lat = myLocation(myDrone_connection).lat;
    lon = myLocation(myDrone_connection).lng;
    alt = myLocation(myDrone_connection).alt;
    L =str(run_time) + ", " + str(lat) + ", " + str(lon) + ", " + str(alt)+ "\n"
    file.writelines(L)
def cmd_set_home(home_location, altitude):
    print('--- ', myDrone.target_system, ',', myDrone.target_component)
    myDrone.mav.command_long_send(
        myDrone.target_system, myDrone.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude)
def uploadmission(aFileName):
    home_location = None
    home_altitude = None

    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                ln_z=float(linearray[10])
                ln_autocontinue = int(float(linearray[11].strip()))
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(myDrone.target_system, myDrone.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wpWiz.add(p)

    print("Waypoints added to py queue: ", wpWiz.count())

    cmd_set_home(home_location,home_altitude)
    msg = myDrone.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)

    #send waypoint to airframe
    myDrone.waypoint_clear_all_send()
    myDrone.waypoint_count_send(wpWiz.count())

    for i in range(wpWiz.count()):
        msg = myDrone.recv_match(type=['MISSION_REQUEST'],blocking=True)
        wp_idx = msg.seq
        wp_cur = wpWiz.wp(wp_idx)
        myDrone.mav.send(wp_cur)
        print('Sending waypoint {0}'.format(msg.seq))
def timed_logData(file, duration):
    count = 0
    dt = 0.01
    while count < duration:
        logData(file,mission_a_go,time_start)
        time.sleep(dt)
        count += dt
def add_latlonalt(lat, lon, altitude, terrain_alt=False):
    '''add a point via latitude/longitude/altitude'''
    if terrain_alt:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
    else:
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    p = mavutil.mavlink.MAVLink_mission_item_message(myDrone.target_system,
                                                     myDrone.target_component,
                                                     0,
                                                     frame,
                                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                     0, 0, 0, 0, 0, 0,
                                                     lat, lon, altitude)
    return p
def send_myWP(wp):
    wpWiz.add(wp)
    myDrone.mav.send(wp)

####################### Priming Vehicle Protocol ######################


# Wait a heartbeat before sending commands
myDrone.wait_heartbeat()
print("Heartbeat acquired!")
### Point Home ###
home_lat = myLocation(myDrone).lat;
home_long = myLocation(myDrone).lng;
home_alt = myLocation(myDrone).alt;

### Show home location ###
print(" Altitude: ", home_alt)
print(" Home: ", home_lat , ", ", home_long)
# Break and return from function just below target altitude.
time.sleep(1)

arm_sub(myDrone)
print("Sub ready!")
change_mode("STABILIZE", myDrone)

####################### Mission Protocol ######################

print("Creating mission (for current location)...")
wpWiz.clear()

delta = 1;
bearing = 90;
dz = -10

#Queue up waypoints using linear distance delta method
wpWiz.add(delta_WPgen(myDrone, 0, 0, dz))
# latDat.append(delta_WPgen(myDrone, 0, 0, dz).lat), latDat.append(delta_WPgen(myDrone, 0, 0, dz).lon)
wpWiz.add(delta_WPgen(myDrone, delta, 0, dz))
# latDat.append(delta_WPgen(myDrone, dx, 0, dz).lat), latDat.append(delta_WPgen(myDrone, dx, 0, dz).lon)
wpWiz.add(delta_WPgen(myDrone, delta, 90, dz))
# latDat.append(delta_WPgen(myDrone, dx, dy, dz).lat), latDat.append(delta_WPgen(myDrone, dx, dy, dz).lon)
wpWiz.add(delta_WPgen(myDrone, delta, 180, dz))
# latDat.append(delta_WPgen(myDrone, 0, dy, dz).lat), latDat.append(delta_WPgen(myDrone, 0, dy, dz).lon)
wpWiz.add(delta_WPgen(myDrone, delta, 270, dz))
wpWiz.add(delta_WPgen(myDrone, delta, 270, dz))

# latDat.append(delta_WPgen(myDrone, 0, 2*dy, dz).lat), latDat.append(delta_WPgen(myDrone, 0, 2*dy, dz).lon)

##### UNCOMMENT BELOW SECTION TO CHECK PATH IN PYPLOT ########
# latDat.append(delta_WPloc(myDrone, 0, 0)[0])
# lonDat.append(delta_WPloc(myDrone, 0, 0)[1])
#
# latDat.append(delta_WPloc(myDrone, delta, 0)[0])
# lonDat.append(delta_WPloc(myDrone, delta, 0)[1])
#
# latDat.append(delta_WPloc(myDrone, delta, 90)[0])
# lonDat.append(delta_WPloc(myDrone, delta, 90)[1])
#
# latDat.append(delta_WPloc(myDrone, delta, 180)[0])
# lonDat.append(delta_WPloc(myDrone, delta, 180)[1])
#
# latDat.append(delta_WPloc(myDrone, delta, 270)[0])
# lonDat.append(delta_WPloc(myDrone, delta, 270)[1])


# plt.plot(latDat, lonDat, '-*')
# plt.show()
##### UNCOMMENT ABOVE SECTION TO CHECK PATH IN PYPLOT ########

print("Delta Waypoints Queued! Sending...")

myDrone.waypoint_clear_all_send()
myDrone.waypoint_count_send(wpWiz.count())

for i in range(wpWiz.count()):
    msg = myDrone.recv_match(type=['MISSION_REQUEST'],blocking=True)
    wp_idx = msg.seq
    wp_cur = wpWiz.wp(wp_idx)
    myDrone.mav.send(wp_cur)
    print('Sending waypoint {0}'.format(msg.seq))


print("Starting mission")
time_start = time.time()

# Set mode to AUTO to start mission
change_mode("NONLIN", myDrone)
# file = open("log.txt","a")
# file.truncate(0)

while True:
    # travel_updates()
    # logData(myDrone, file, time_start)
    currentWaypoint = myDrone.waypoint_current()
    time.sleep(1)
    if currentWaypoint == wpWiz.count()-1: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print("Final waypoint reached!")
        break;

# change_mode("STABILIZE", myDrone)
# timed_logData(file, 180)

####################### Closing Mission Protocol ######################

print("Surfacing...")
change_mode("SURFACE", myDrone)

time.sleep(10)

print("Surfaced and Maintaining Position")
# Close vehicle object before exiting script
print("Close vehicle object")
# change_mode("STABILIZE", myDrone)
disarm_sub(myDrone)
