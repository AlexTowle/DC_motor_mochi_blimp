import pygame
import time
import socket
import struct
import math
from NatNetClient import NatNetClient
from util import quaternion_to_euler
from simple_pid import PID
import numpy as np
from scipy.spatial.transform import Rotation as R

positions = {}
rotations = {}
#rotational_matrices = {}

# udp params
UDP_IP_TOW = "192.168.0.5"
UDP_IP_SECOND_TOW = "192.168.0.29"
UDP_IP_LEAD = "192.168.0.27"
UDP_PORT = 1234
print("UDP target IP (tow): %s" % UDP_IP_TOW)
print("UDP target IP (second tow): %s" % UDP_IP_SECOND_TOW)
print("UDP target IP (lead): %s" % UDP_IP_LEAD)
print("UDP target port: %s" % UDP_PORT)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(id, position, rotation_quaternion):
    positions[id] = position
    # # The rotation is in quaternion. We need to convert it to euler angles
    rotx, roty, rotz = quaternion_to_euler(rotation_quaternion)
    # # Store the roll pitch and yaw angles
    rotations[id] = (rotx, roty, rotz)


def udp_init():
    sock = socket.socket(
        socket.AF_INET, # Internet
        socket.SOCK_DGRAM
    ) # UDP
    return sock

"""def joystick_init():
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    # Prints the values for axis0
    joystick = pygame.joystick.Joystick(0)
    return joystick"""

def init():
    sock = udp_init()
    #joystick = joystick_init()
    return sock #, joystick

def udp_send_tow(sock, ip, port, message):
    sock.sendto(message, (ip, port))


if __name__ == "__main__":
    sock = init()
    l = 0.2 # meters
    try:
        while True:
            time_start = time.time()

            clientAddress = "192.168.0.63"
            optitrackServerAddress = "192.168.0.4"
            #tow_robot_id = 372
            lead_robot_id = 380

            # This will create a new NatNet client
            streaming_client = NatNetClient()
            streaming_client.set_client_address(clientAddress)
            streaming_client.set_server_address(optitrackServerAddress)
            streaming_client.set_use_multicast(True)
            # Configure the streaming client to call our rigid body handler on the emulator to send data out.
            streaming_client.rigid_body_listener = receive_rigid_body_frame

            # Start up the streaming client now that the callbacks are set up.
            # This will run perpetually, and operate on a separate thread.
            is_running = streaming_client.run()
            #tow_z_pid = PID(3, 0, 1, setpoint = 1, sample_time=0.01)
            lead_z_pid = PID(3, 0, 1, setpoint = 1.5, sample_time=0.01)
            yaw_pid = PID(0.001, 0, 0.001, setpoint = 0)
            x_pid = PID(0.1, 0.01, 0.5, setpoint = 2)
            y_pid = PID(0.1, 0, 0.5, setpoint = 3)
            last_x = 0
            last_y = 0

            while is_running:
                #print(positions)
                if lead_robot_id in positions:
                    #print(positions[lead_robot_id])
                    #xy_distance = math.sqrt((x_pid.setpoint - tow_positions[tow_robot_id][0])**2 + (y_pid.setpoint - tow_positions[tow_robot_id][1])**2)
                    #print(xy_distance)

                    '''tow_fx = 0.1#x_pid(tow_positions[tow_robot_id][0])
                    #taux = joystick.get_axis(0) # left handler: left-right torque
                    #taux = -rotT[1]
                    #print(fx)
                    tow_taux = 0
                    #print(taux)

                    tow_fz = tow_z_pid(positions[tow_robot_id][2]) + 2#0.11 * (target_height - positions[robot_id][2])#-joystick.get_axis(4) # right handler: up-down, inverted
                    #print(fz)
                    if tow_fz > 1:
                        tow_fz = 1
                    elif tow_fz <= 0:
                        tow_fz = 0.001
                    #print(fz)

                    #print(yaw_pid.setpoint)
                    tow_tauz = 0#yaw_pid(tow_rotations[tow_robot_id][2])# right handler: left-right
                    #print(tauz)
                    #print(fx, taux, fz, tauz)
        
                    tow_f1x = (tow_fx - tow_tauz/l)/2
                    tow_f2x = (tow_fx + tow_tauz/l)/2
                    tow_f1z = (tow_fz + tow_taux/l)/2
                    tow_f2z = (tow_fz - tow_taux/l)/2
                    #print(f1x, f2x, f1z, f2z)'''

                    world_x_velocity = (positions[lead_robot_id][0] - last_x) / 0.01
                    last_x = positions[lead_robot_id][0]
                    world_y_velocity = (positions[lead_robot_id][1] - last_y) / 0.01
                    last_y = positions[lead_robot_id][1]
                    body_x_velocity = world_x_velocity * math.cos(rotations[lead_robot_id][2] * math.pi/180) - world_y_velocity * math.sin(rotations[lead_robot_id][2] * math.pi/180)

                    MAX_THRUST = 20#17
                    tow1_f1 = 3#math.sqrt(tow_f1z**2) * MAX_THRUST
                    tow1_f2 = 3#math.sqrt(tow_f2z**2) * MAX_THRUST
                    tow1_t1 = (145.0)
                    tow1_t2 = (35.0)
                    #print(f1, f2, t1, t2)
                    #print(t1, t2)
                    #print(f1, f2)

                    tow2_f1 = 4#math.sqrt(tow_f1z**2) * MAX_THRUST
                    tow2_f2 = 4#math.sqrt(tow_f2z**2) * MAX_THRUST
                    tow2_t1 = (145.0)
                    tow2_t2 = (35.0)


                    lead_fx = x_pid(positions[lead_robot_id][0])
                    lead_taux = 0
                    lead_fz = lead_z_pid(positions[lead_robot_id][2]) + 2
                    if lead_fz > 1:
                        lead_fz = 1
                    elif lead_fz <= 0:
                        lead_fz = 0.001
                    lead_tauz = yaw_pid(rotations[lead_robot_id][2])

                    lead_f1x = (lead_fx - lead_tauz/l)/2
                    lead_f2x = (lead_fx + lead_tauz/l)/2
                    lead_f1z = (lead_fz + lead_taux/l)/2
                    lead_f2z = (lead_fz - lead_taux/l)/2

                    lead_f1 = math.sqrt(lead_f1x**2 + lead_f1z**2) * MAX_THRUST
                    lead_f2 = math.sqrt(lead_f2x**2 + lead_f2z**2) * MAX_THRUST
                    lead_t1 = 180 - math.atan2(lead_f1z, lead_f1x)*180/math.pi
                    lead_t2 = math.atan2(lead_f2z, lead_f2x)*180/math.pi


                    # f1 = 300
                    message_tow1 = struct.pack('<ffff', tow1_f1, tow1_f2, tow1_t1, tow1_t2)
                    message_tow2 = struct.pack('<ffff', tow2_f1, tow2_f2, tow2_t1, tow2_t2)
                    message_lead = struct.pack('<ffff', lead_f1, lead_f2, lead_t1, lead_t2)
        
                    #print(f1, f2, t1, t2)
                    udp_send_tow(sock, UDP_IP_TOW, UDP_PORT, message_tow1)
                    udp_send_tow(sock, UDP_IP_SECOND_TOW, UDP_PORT, message_tow2)
                    #udp_send_tow(sock, UDP_IP_LEAD, UDP_PORT, message_lead)
                    # print(message)
                time.sleep(0.01)
    except:
        message_tow1 = struct.pack('<ffff', 0, 0, 90, 90)
        message_tow2 = struct.pack('<ffff', 0, 0, 90, 90)
        message_lead = struct.pack('<ffff', 0, 0, 90, 90)
        udp_send_tow(sock, UDP_IP_TOW, UDP_PORT, message_tow1)
        udp_send_tow(sock, UDP_IP_SECOND_TOW, UDP_PORT, message_tow2)
        udp_send_tow(sock, UDP_IP_LEAD, UDP_PORT, message_lead)