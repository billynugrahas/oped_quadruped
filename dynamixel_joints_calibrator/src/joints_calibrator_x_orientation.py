#!/usr/bin/env python
#credits to: Ryu Woon Jung (Leon) and ROBOTIS CO., LTD.
#modified by Billy Nugraha Sutandi

from __future__ import print_function

import os
import sys, select, termios, tty
import numpy as np
import time

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_AX_GOAL_POSITION       = 4
LEN_AX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Dynamixel Address
LFH_SERVO_ID                = 3
LFU_SERVO_ID                = 4
LFL_SERVO_ID                = 5

RFH_SERVO_ID                = 0
RFU_SERVO_ID                = 1
RFL_SERVO_ID                = 2

LHH_SERVO_ID                = 9
LHU_SERVO_ID                = 10
LHL_SERVO_ID                = 11

RHH_SERVO_ID                = 6
RHU_SERVO_ID                = 7
RHL_SERVO_ID                = 8


dxl_quadruped_servo_address = [[LFH_SERVO_ID, LFU_SERVO_ID, LFL_SERVO_ID],
                               [RFH_SERVO_ID, RFU_SERVO_ID, RFL_SERVO_ID],
                               [LHH_SERVO_ID, LHU_SERVO_ID, LHL_SERVO_ID],
                               [RHH_SERVO_ID, RHU_SERVO_ID, RHL_SERVO_ID]]

#TO DO: Changes these values as your first offset joint
# leg_joint_offsett           = [[0, 79, 237],             #leg_joint_offsett
#                                [-5, 240, -260],
#                                [5, 82, 263],
#                                [5, 234, -267]]

# # X ORIENTATION TERBARU
leg_joint_offsett           = [[9, 52, 227],             #leg_joint_offsett
                               [-11, 259, -204],
                               [-11, 259, -186],
                               [9, 46, 173]]

# # FIX TERBARU
# leg_joint_offsett           = [[9, 46, 251],             #leg_joint_offsett
#                                [-11, 259, -258],
#                                [10, 71, 247],
#                                [-10, 201, -238]]

# # # OK
# leg_joint_offsett           = [[15, 79, 182],             #leg_joint_offsett
#                                [-20, 220, -165],
#                                [10, 92, 178],
#                                [-10, 204, -172]]

# # # X ORIENTATION
# leg_joint_offsett           = [[15, 79, 182],             #leg_joint_offsett
#                                [-20, 220, -165],
#                                [10, -105, 149],
#                                [-10, 402, -161]]

# leg_joint_offsett           = [[15, 59, 182],             #leg_joint_offsett
#                                [-20, 240, -165],
#                                [10, 92, 178],
#                                [-10, 204, -172]]
                               
leg_joint_const             = [[512, 512, 512],           #leg_joint_const as constant value for add to your leg_joint_offsett
                               [512, 512, 512],
                               [512, 512, 512],
                               [512, 512, 512]]

leg_joint_current_position  = [[leg_joint_offsett[0][0] + leg_joint_const[0][0], leg_joint_offsett[0][1] + leg_joint_const[0][1], leg_joint_offsett[0][2] + leg_joint_const[0][2]],
                               [leg_joint_offsett[1][0] + leg_joint_const[1][0], leg_joint_offsett[1][1] + leg_joint_const[1][1], leg_joint_offsett[1][2] + leg_joint_const[1][2]],
                               [leg_joint_offsett[2][0] + leg_joint_const[2][0], leg_joint_offsett[2][1] + leg_joint_const[2][1], leg_joint_offsett[2][2] + leg_joint_const[2][2]],
                               [leg_joint_offsett[3][0] + leg_joint_const[3][0], leg_joint_offsett[3][1] + leg_joint_const[3][1], leg_joint_offsett[3][2] + leg_joint_const[3][2]]]

param_goal_position         = [512, 512, 512, 512]

ADD_CONST                   = 3
temp_joint_offsett          = [[0, 0, 0],           #leg_joint_const as constant value for add to your leg_joint_offsett
                               [0, 0, 0],
                               [0, 0, 0],
                               [0, 0, 0]]

BAUDRATE                    = 1000000           # Dynamixel default baudrate : 1000000
DEVICENAME                  = '/dev/U2D2'       # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

DXL_MINIMUM_POSITION_VALUE  = 100               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

dxl_goal_position = 512                         # Goal position

torque_enabled_flag_        = False

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)


class Calibrator:
    def __init__(self):
        self.move_const = 1
        self.main_msg = """
=========== QUADRUPED JOINTS CALIBRATOR ===========

       TOP VIEW         ||     SIDE VIEW
                        ||
    LEFT      RIGHT     ||
1   \__[  ^  ]__/   2   ||   ____________ ->
       |     |          ||   |/-- __ /--|    Hip
     __|     |__        ||   /      /        Upper Leg
3   /   -----   \   4   ||   \      \        Lower Leg

====================================================

SETTING LEFT - RIGHT LEG
-------------------------
   >>   0           FRONT
        1           HIND

Press <enter> to keep the current choice[>>], or type selection number
"""

        self.msg = """
q/a : increase/decrease left hip angle
w/s : increase/decrease left upper leg angle
e/d : increase/decrease left lower leg angle

i/j : increase/decrease right hip angle
o/k : increase/decrease right upper leg angle
p/l : increase/decrease right lower leg angle

Enter : Save your changes
ESC   : return to main menu
CTRL-C to quit
        """
        self.dynamixelSetup()
        self.mainMenu()

    def mainMenu(self):
        global torque_enabled_flag_
        self.settings = termios.tcgetattr(sys.stdin)
        try:
            print(self.main_msg)
            
            while 1:
                key = self.getKey()

                if key == '0':
                    print (">> FRONT LEGS")
                    self.poll_keys(0)
                elif key == '1':
                    print (">> HIND LEGS")
                    self.poll_keys(2)
                elif key == chr(0x0d): #carriage return
                    print (">> FRONT LEGS")
                    self.poll_keys(0)
                elif key == chr(0x1b): #esc
                    if torque_enabled_flag_:
                        self.disableTorque()
                    portHandler.closePort()
                    quit()
                else:
                    if (key == '\x03'):
                        if torque_enabled_flag_:
                            self.disableTorque()
                        portHandler.closePort()
                        quit()
        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        

    def poll_keys(self, orientation):
        global torque_enabled_flag_
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            print(self.msg)
            
            while 1:
                key = self.getKey()
                if key == 'q':
                    temp_joint_offsett[orientation][0] += ADD_CONST
                    leg_joint_current_position[orientation][0] += ADD_CONST
                    print("[LH] - increased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation][0], leg_joint_current_position[orientation][0]))
                    self.syncWriteJoints()
                elif key == 'a':
                    temp_joint_offsett[orientation][0] -= ADD_CONST
                    leg_joint_current_position[orientation][0] -= ADD_CONST
                    print("[LH] - decreased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation][0], leg_joint_current_position[orientation][0]))
                    self.syncWriteJoints()

                elif key == 'w':
                    temp_joint_offsett[orientation][1] += ADD_CONST
                    leg_joint_current_position[orientation][1] += ADD_CONST
                    print("[LU] - increased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation][1], leg_joint_current_position[orientation][1]))
                    self.syncWriteJoints()
                elif key == 's':
                    temp_joint_offsett[orientation][1] -= ADD_CONST
                    leg_joint_current_position[orientation][1] -= ADD_CONST
                    print("[LU] - decreased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation][1], leg_joint_current_position[orientation][1]))
                    self.syncWriteJoints()

                elif key == 'e':
                    temp_joint_offsett[orientation][2] += ADD_CONST
                    leg_joint_current_position[orientation][2] += ADD_CONST
                    print("[LL] - increased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation][2], leg_joint_current_position[orientation][2]))
                    self.syncWriteJoints()
                elif key == 'd':
                    temp_joint_offsett[orientation][2] -= ADD_CONST
                    leg_joint_current_position[orientation][2] -= ADD_CONST
                    print("[LL] - decreased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation][2], leg_joint_current_position[orientation][2]))
                    self.syncWriteJoints()

                if key == 'i':
                    temp_joint_offsett[orientation+1][0] += ADD_CONST
                    leg_joint_current_position[orientation+1][0] += ADD_CONST
                    print("[RH] - increased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation+1][0], leg_joint_current_position[orientation+1][0]))
                    self.syncWriteJoints()
                elif key == 'j':
                    temp_joint_offsett[orientation+1][0] -= ADD_CONST
                    leg_joint_current_position[orientation+1][0] -= ADD_CONST
                    print("[RH] - decreased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation+1][0], leg_joint_current_position[orientation+1][0]))
                    self.syncWriteJoints()

                elif key == 'o':
                    temp_joint_offsett[orientation+1][1] += ADD_CONST
                    leg_joint_current_position[orientation+1][1] += ADD_CONST
                    print("[RU] - increased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation+1][1], leg_joint_current_position[orientation+1][1]))
                    self.syncWriteJoints()
                elif key == 'k':
                    temp_joint_offsett[orientation+1][1] -= ADD_CONST
                    leg_joint_current_position[orientation+1][1] -= ADD_CONST
                    print("[RU] - decreased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation+1][1], leg_joint_current_position[orientation+1][1]))
                    self.syncWriteJoints()

                elif key == 'p':
                    temp_joint_offsett[orientation+1][2] += ADD_CONST
                    leg_joint_current_position[orientation+1][2] += ADD_CONST
                    print("[RL] - increased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation+1][2], leg_joint_current_position[orientation+1][2]))
                    self.syncWriteJoints()
                elif key == 'l':
                    temp_joint_offsett[orientation+1][2] -= ADD_CONST
                    leg_joint_current_position[orientation+1][2] -= ADD_CONST
                    print("[RL] - decreased by %d [new joint offsett = %d] goal: %d" %(ADD_CONST, temp_joint_offsett[orientation+1][2], leg_joint_current_position[orientation+1][2]))
                    self.syncWriteJoints()

                elif key == chr(0x0d): #carriage return
                    self.showNewJointsOffsett(orientation)

                elif key == chr(0x1b): # esc button
                    print (self.main_msg)
                    break

                else:
                    if (key == '\x03'):
                        if torque_enabled_flag_:
                            self.disableTorque()
                        portHandler.closePort()
                        quit()
                
        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            quit()
        
    def showNewJointsOffsett(self, orientation):
        global torque_enabled_flag_
        self.settings = termios.tcgetattr(sys.stdin)

        self.final_msg = "\nSAVED!\n"
        print (self.final_msg)
        if orientation == 0:
            print ("""++ Your new Front joints offsett: ++""")
        elif orientation == 1:
            print ("""++ Your new Hind joints offsett: ++""")
        print (temp_joint_offsett)

        print ("Current leg joint offsett: (After added leg_joint_offsett)")
        print ("Leg 1: %d %d %d" % (temp_joint_offsett[0][0]+leg_joint_offsett[0][0], temp_joint_offsett[0][1]+leg_joint_offsett[0][1], temp_joint_offsett[0][2]+leg_joint_offsett[0][2]))
        print ("Leg 2: %d %d %d" % (temp_joint_offsett[1][0]+leg_joint_offsett[1][0], temp_joint_offsett[1][1]+leg_joint_offsett[1][1], temp_joint_offsett[1][2]+leg_joint_offsett[1][2]))
        print ("Leg 3: %d %d %d" % (temp_joint_offsett[2][0]+leg_joint_offsett[2][0], temp_joint_offsett[2][1]+leg_joint_offsett[2][1], temp_joint_offsett[2][2]+leg_joint_offsett[2][2]))
        print ("Leg 4: %d %d %d" % (temp_joint_offsett[3][0]+leg_joint_offsett[3][0], temp_joint_offsett[3][1]+leg_joint_offsett[3][1], temp_joint_offsett[3][2]+leg_joint_offsett[3][2]))
        # # WIP - print temp_joint_offsett + leg_joint_offsett
        # leg_joint_offsett           = [[0, 79, 237],             #leg_joint_offsett
        #                        [-5, 240, -260],
        #                        [5, 82, 263],
        #                        [5, 234, -267]]

        print ("===========================")
        print ("Enter : return to main menu")
        print ("ESC   : quit")
        print ("CTRL-C to quit")
        print ("===========================")

        try:
            while 1:
                key = self.getKey()

                if key == chr(0x0d): #carriage return
                    self.mainMenu()

                elif key == chr(0x1b): # esc button
                    if torque_enabled_flag_:
                        self.disableTorque()
                    portHandler.closePort()
                    quit()

                else:
                    if (key == '\x03'):
                        if torque_enabled_flag_:
                            self.disableTorque()
                        portHandler.closePort()
                        quit()

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        
    def dynamixelSetup(self):
        self.settings = termios.tcgetattr(sys.stdin)
        # portHandler = PortHandler(DEVICENAME)
        # packetHandler = PacketHandler(PROTOCOL_VERSION)

        # groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION)

        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        
        try:
            print("\n\nAre you sure you have finished set your initial joints position?")
            print("<enter>  : Yes")
            print("ESC      : No / exit")
            while 1:
                key = self.getKey()

                if key == chr(0x0d): #carriage return
                    self.enableTorque()
                    print ("OK")
                    break
                elif key == chr(0x1b): #esc
                    print ("ESC")
                    portHandler.closePort()
                    quit()
                else:
                    if (key == '\x03'):
                        print ("CTRL-C")
                        portHandler.closePort()
                        quit()
                
        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


    def enableTorque(self):
        global torque_enabled_flag_
        for leg_number in range(4):
            for joint_number in range(3):
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_quadruped_servo_address[leg_number][joint_number], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("leg = %d joint = %d ID:%d has been successfully connected" % (leg_number, leg_number, dxl_quadruped_servo_address[leg_number][joint_number]))
        
        torque_enabled_flag_ = True
        print ("Torque enabled!")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def disableTorque(self):
        global torque_enabled_flag_
        for leg_number in range(4):
            for joint_number in range(3):
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_quadruped_servo_address[leg_number][joint_number], ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                else:
                    print("leg = %d joint = %d ID:%d has been successfully disconnected" % (leg_number, leg_number, dxl_quadruped_servo_address[leg_number][joint_number]))
        
        torque_enabled_flag_ = False
        print ("Torque disabled!")

    def syncWriteJoints(self):
        for leg_number in range(4):
            for joint_number in range(3):
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(leg_joint_current_position[leg_number][joint_number])), DXL_HIBYTE(DXL_LOWORD(leg_joint_current_position[leg_number][joint_number])), DXL_LOBYTE(DXL_HIWORD(leg_joint_current_position[leg_number][joint_number])), DXL_HIBYTE(DXL_HIWORD(leg_joint_current_position[leg_number][joint_number]))]
                dxl_addparam_result = groupSyncWrite.addParam(dxl_quadruped_servo_address[leg_number][joint_number], param_goal_position)
                if dxl_addparam_result != True:
                    print("leg = %d joint = %d [ID:%02d] groupSyncWrite addparam failed" % (leg_number, joint_number, dxl_quadruped_servo_address[leg_number][joint_number]))
                    # quit()
        dxl_comm_result = groupSyncWrite.txPacket()
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("Failed: %s" % packetHandler.getTxRxResult(dxl_comm_result)) 
        # else:
        #     print("SyncWrite Succeeded")
        print("OK")
        groupSyncWrite.clearParam()
        time.sleep(0.2)
        print("SyncWrite Succeeded")
                


if __name__ == "__main__":
    calibrator = Calibrator()
       
# while 1:
#     print("Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break

#     # Allocate goal position value into byte array
#     param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]


#     for leg_number in range(4):
#         for joint_number in range(3):
#             dxl_addparam_result = groupSyncWrite.addParam(dxl_quadruped_servo_address[leg_number][joint_number], param_goal_position)
#             if dxl_addparam_result != True:
#                 print("leg = %d joint = %d [ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
#                 quit()

#     # Syncwrite goal position
#     dxl_comm_result = groupSyncWrite.txPacket()
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

#     # Clear syncwrite parameter storage
#     groupSyncWrite.clearParam()


# sudah
# disableTorque()

# # Close port
# portHandler.closePort()