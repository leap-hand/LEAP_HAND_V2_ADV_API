#!/usr/bin/env python3
import sys
import time
import numpy as np

import sys
sys.path.insert(0, "../leap_v2/leap_v2/leap_v2_utils")
from leap_v2_utils.dynamixel_client import *
'''
#######################################################
Aligns the Hand.  

The XM430s do have current mode so we can use this for calibration and then set current limits at runtime.

Note: Reset the motors first by unplugging and replugging them and the U2D2.

Slow does each motor individually
Fast does joints in parallel on each finger to save time

Saves values in an alignment file that can be read by the running script.
#######################################################
'''
def main(fn, isLeftcmd):
    
    motors_side =    [0,3,6,9 ,12]
    motors_forward = [1,4,7,10,13]
    motors_curl =   [2,5,8,11,14]
    motors_palm =    [15,16]
    all_motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
    # for left hand, set this to -1, for right hand set it to 1
    if isLeftcmd:
        isLeft = -1
        fp = "alignments/" + str(fn) + "_L.csv"
        port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISVLF-if00-port0"
    else:
        isLeft = 1
        fp = "alignments/" + str(fn) + "_R.csv"
        port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9HDDWR-if00-port0"
    dxl_client = DynamixelClient(all_motors, port, 4000000)
    dxl_client.connect()

    dxl_client.sync_write(motors_curl, np.ones(len(motors_curl)) * 0, 11, 1)  #set current control mode
    dxl_client.sync_write(motors_curl, np.ones(len(motors_curl)) * 0, 102, 2)  #set goal current to 0
    dxl_client.set_torque_enabled(motors_curl, True)
    output = np.zeros((17,2)) 

    ma_high = signed_to_unsigned(isLeft * (-800/2.69), 2)  #ma * 2.69
    ma_high_neg = signed_to_unsigned(isLeft * (800/2.69),  2)  #ma * 2.69

    #Forward motors: we run pwm mode to the lower position.  We know where the upper position is relative to the lower one.
    dxl_client.sync_write(motors_forward, np.ones(len(motors_forward))*0, 11, 1)  #set current control mode
    dxl_client.set_torque_enabled(motors_forward, True)
    dxl_client.sync_write(motors_forward, [ma_high, ma_high, ma_high_neg, ma_high_neg, ma_high] , 102, 2)
    time.sleep(1)
    dxl_client.sync_write(motors_forward, [0,], 100, 2)
    time.sleep(0.5)
    for i in range(0,5):
        pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
        time.sleep(0.1)
    output[motors_forward,0] = pos_now[motors_forward]  
    output[motors_forward,1] = output[motors_forward,0] + np.array([isLeft*1.57, isLeft*1.5, isLeft*-1.5, isLeft*-1.57, isLeft*1.5])

    #Curl motors: first we read the outer position, then we curl them and read the closed position
    #For the curl motors, we have the user uncurl the fingers.  We record that uncurled position and then curl it and record the curled position
    for i in range(0,5):
        pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
        time.sleep(0.1)
    output[motors_curl,0] = pos_now[motors_curl]
    dxl_client.sync_write(motors_curl, [ma_high_neg, ma_high, ma_high_neg, ma_high, ma_high], 102, 2)
    time.sleep(5)
    for i in range(0,5):
        pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
        time.sleep(0.1)
    output[motors_curl,1] = np.array(pos_now[motors_curl])   

    #Set then back to the uncurled position using current position control
    dxl_client.set_torque_enabled(motors_curl, False)
    time.sleep(0.0001)
    dxl_client.sync_write(motors_curl, np.ones(len(motors_curl))*0, 11, 1)
    dxl_client.sync_write(motors_curl, np.ones(12)*[0,], 102, 2) 
    dxl_client.set_torque_enabled(motors_curl, True)
    for i in range(0,5):
        pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
        time.sleep(0.1)
    output[motors_curl,0] = pos_now[motors_curl]

    #For side to side motors, we do nothing since they are already mounted correctly.

    output[motors_side,0] = 3.14 + isLeft * -1.57
    output[motors_side,1] = 3.14 + isLeft * 1.57
    
    ##For palm motors, we run them down to the bottom positions and then we know the top position relative to the bottom
    dxl_client.sync_write(motors_palm, np.ones(len(motors_palm))*0, 11, 1)  #set current control mode
    dxl_client.sync_write(motors_palm, np.ones(len(motors_palm)) * 0, 102, 2)  #set goal current to 0
    dxl_client.set_torque_enabled(motors_palm, True)
    dxl_client.sync_write(motors_palm, [ma_high, ma_high_neg], 102, 2)
    time.sleep(2)
    for i in range(0,5):
        pos_now, vel_now, cur_now = dxl_client.read_pos_vel_cur()
        time.sleep(0.1)
    output[motors_palm,0] = pos_now[motors_palm]
    output[motors_palm,1] = pos_now[motors_palm] + np.deg2rad([isLeft*70,isLeft*-70])  #this is thumb then MCP for 4 fingers
    
    # all_in_range = False
    # while not all_in_range:
    #     all_in_range = True
    #     for i in range(0,17):
    #         if output[i,0] < 0:
    #             output[i,:] = output[i,:] + (np.pi * 2)
    #             all_in_range = False
    #         if output[i,1] > (np.pi * 2):
    #             output[i,:] = output[i,:] - (np.pi * 2)
    #             all_in_range = False
                        
    output = np.degrees(output)
    print(output)
    np.savetxt(fp, output, delimiter=",", fmt = '%10.5f')
    dxl_client.disconnect()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-f',
        '--file',
        default='test',
        help='The file to save')
    parser.add_argument(
        '-l',
        '--isleft',
        action="store_true",
        help='If True then its left.  Otherwise its right')
    parsed_args = parser.parse_args()
    main(parsed_args.file, parsed_args.isleft)