#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import argparse
import subprocess
import numpy as np
import librosa
import scipy.signal
import pyaudio
import wave
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


#################### Function Definitions ####################
# Define the function for condition_yaw
def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # delay to wait until yaw of copter is at desired yaw angle
    time.sleep(3)
    
# Define the function for current location
def get_current_location_rad():
    earth_radius = 6378137.0  # Radius of "spherical" earth
    ref_lat = 35.727279       # AERPAW initial latitude
    ref_lon = -78.69611       # AERPAW initial longitude
    current_location = vehicle.location.global_relative_frame
    current_lat = current_location.lat
    current_lon = current_location.lon

    # Convert latitude and longitude to radians
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)
    lat_rad = math.radians(current_lat)
    lon_rad = math.radians(current_lon)

    x = (lon_rad - ref_lon_rad) * math.cos((lat_rad + ref_lat_rad) / 2) * earth_radius
    y = (lat_rad - ref_lat_rad) * earth_radius
    print("\n Funciton[get_curr_location]: Get_current_location_rad: x = %f, y = %f" % (x, y))

    return x, y

# Define the function for indentifying the microphone
def list_devices():
    pa = pyaudio.PyAudio()
    device_count = pa.get_device_count()

    print("Audio devices:")
    for i in range(device_count):
        device_info = pa.get_device_info_by_index(i)
        print(f"{i}: {device_info['name']}")

    pa.terminate()

# Define the function for recording
def record_audio(output_filename, device_index, duration, sample_rate=44100, channels=1, chunk_size=1024):
    '''
    audio_format = pyaudio.paInt16
    pa = pyaudio.PyAudio()

    stream = pa.open(rate=sample_rate, channels=channels, format=audio_format, input=True, input_device_index=device_index, frames_per_buffer=chunk_size)

    print(f"Recording with microphone {device_index}...")

    frames = []
    num_chunks = int(sample_rate / chunk_size * duration)

    for _ in range(num_chunks):
        data = stream.read(chunk_size)
        frames.append(data)

    print(f"Recording finished with microphone {device_index}.")

    stream.stop_stream()
    stream.close()
    pa.terminate()

    wf = wave.open(output_filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(pa.get_sample_size(audio_format))
    wf.setframerate(sample_rate)
    wf.writeframes(b''.join(frames))
    wf.close()

    print(f"Saved recording from microphone {device_index} to {output_filename}")
    '''
    pass

# Define the function for multiple audio recording
def record_multiple_microphones(device_indices, duration):
    '''
    threads = []
    print("debugg point #1")
    for device_index in device_indices:
        output_filename = f"recording_mic{device_index}.wav"
        t = threading.Thread(target=record_audio, args=(output_filename, device_index, duration))
        threads.append(t)
    print("debugg point #2")
    for t in threads:
        t.start()

    for t in threads:
        t.join()
    '''
    pass
        
# Define the function for TDoA
def calculate_tdoa(file1, file2, sample_rate):
    audio1, _ = librosa.load(file1, sr=sample_rate)
    audio2, _ = librosa.load(file2, sr=sample_rate)

    correlation = scipy.signal.correlate(audio1, audio2, mode='full')
    lag = np.argmax(correlation) - len(audio1) + 1
    time_difference = lag / sample_rate
    print("\n Funciton[Calcuate_toda]")
    return time_difference

# Define the function for localization
def localization():
    # 1. Calculate the time difference of arrival (TDoA) between the 4 microphones
    mic1_file = "recording_mic1.wav"
    mic2_file = "recording_mic2.wav"
    mic3_file = "recording_mic3.wav"
    mic4_file = "recording_mic4.wav"

    sample_rate = 44100

    tdoa12 = calculate_tdoa(mic1_file, mic2_file, sample_rate)
    tdoa13 = calculate_tdoa(mic1_file, mic3_file, sample_rate)
    tdoa14 = calculate_tdoa(mic1_file, mic4_file, sample_rate)
    tdoa23 = calculate_tdoa(mic2_file, mic3_file, sample_rate)
    tdoa24 = calculate_tdoa(mic2_file, mic4_file, sample_rate)
    tdoa34 = calculate_tdoa(mic3_file, mic4_file, sample_rate)

    # 2. Calculate the distance between the microphones
    d12 = speed_of_sound * tdoa12
    d13 = speed_of_sound * tdoa13
    d14 = speed_of_sound * tdoa14
    d23 = speed_of_sound * tdoa23
    d24 = speed_of_sound * tdoa24
    d34 = speed_of_sound * tdoa34

    # 3. Calculate the position of the target
    x = (d12**2 - d23**2 + 1**2) / (2 * 1)
    y = (d12**2 - d24**2 + 1**2 + 1**2) / (2 * 1) - (x * 1) / 1

    print("\n Funciton[Localization]: \n The target is at: ", x, y)
    return x, y

# Define the function for changing 2D coordinates to GPS coordinates
def change_coordinates(x, y):
    earth_radius = 6378137.0  # Radius of "spherical" earth
    ref_lat = 35.727279       # AERPAW initial latitude
    ref_lon = -78.69611       # AERPAW initial longitude

    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    lat = math.degrees(math.asin(math.sin(ref_lat_rad) + y / earth_radius))
    lon = math.degrees(math.asin(math.sin(ref_lon_rad) + x / earth_radius / math.cos(math.radians(lat))))
    print("\n Funciton[change_coordinates]")
    return lat, lon

# Define the function for get estimation error
def get_error(current_location, target_location):
    dlat = target_location[0] - current_location.lat
    dlong = target_location[1] - current_location.lon
    print("\n Funciton[get_error]: ", math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5)
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

# Define the function for sending the target position to the vehicle     
    
#################### Main Script ####################

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

#connection_string = '127.0.0.1:14551'  # args.connect
connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('\n Connect #1. Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print('\n Connect #2. Successfully connected to vehicle')

# Constants
speed_of_sound = 340.29  # Speed of sound in m/s
vehicle.airspeed = 3     # Set default/target airspeed to 3 m/s
loop_count = 0           # Loop counter
REAL_TX_LAT = 35.727279 
REAL_TX_LON = -78.69611
TARGET_ALTITUDE = 10    # Target altitude in meters
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached"
rcin_4_center = False

# Arm and take off to altitude of 30 meters
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding
print('\n Connect #3. Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('\n Connect #4. Armed...')
vehicle.mode = VehicleMode("GUIDED")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)

    # Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)

    while True:
        # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)
    # yaw north
    condition_yaw(0)

#################### Start of the Loop (at every location) ####################
while loop_count < 10:
    # Get the current location in radians
    print("\n Main #1[Get the current location]")
    lat_rad, lon_rad = get_current_location_rad()

    # Define the coordinates of the 4 microphones
    mic_arr = [
        [lat_rad - 0.5, lon_rad],
        [lat_rad, lon_rad - 0.5],
        [lat_rad + 0.5, lon_rad],
        [lat_rad, lon_rad + 0.5]] 

    # Record audio from all 4 microphones
    print("\n Main #3[Recording]")
    device_indices = [0, 1, 2, 3]
    duration = 10  # Duration in seconds
    record_multiple_microphones(device_indices, duration)
    print("\n [Hovering for 30 seconds: recording]")
    time.sleep(30)   # Should be 30 seconds
    
    # Execute the localization (Get the estimated target location)
    print("\n Main #4[Localization]")
    est_x, est_y = localization()
    x_axis, y_axis = change_coordinates(est_x, est_y)
    print("\n x_axis is: ", x_axis, "y_axis is: ", y_axis)
    print("\n [Hovering for 5 seconds: localization]")
    time.sleep(5)   # Should be 5 seconds

    # Go to the estimated location of the target
    print("\n Main #5[Going to the estimated target]")
    estLocation = LocationGlobalRelative(x_axis, y_axis, TARGET_ALTITUDE)
    vehicle.simple_goto(estLocation)

    # sleep so we can see the change in map
    time.sleep(10)   # Should be 30 seconds

    # Check if the vehicle is within 5 meters of the target location
    print("\n Main #6[Error check: within 5 meters]")
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = [REAL_TX_LAT, REAL_TX_LON]
    distance = get_error(currentLocation, targetLocation)
    print("\n Main 6.1 Error Distance (metres): ", distance)
    if distance <= 5:
        print("\n Main 6.2[Localizing successfully]")
        break
    else:
        print("\n Main 6.2[Localizing again]]")
        loop_count += 1
#################### End of the Loop ####################

# 8. Return to launch
print("\n Main #7[Returning to Launch]")
vehicle.mode = VehicleMode("RTL")
time.sleep(30)

# 9. Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
