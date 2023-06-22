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
rotational_matrices = {}

# udp params
UDP_IP = "192.168.0.36"
UDP_PORT = 1234
print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(id, position, rotation_quaternion):
    # Position and rotation received
    #print(rotation_quaternion)
    q0 = rotation_quaternion[0]
    q1 = rotation_quaternion[1]
    q2 = rotation_quaternion[2]
    q3 = rotation_quaternion[3]
    #first row of rot matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
    #second row of rot matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
    #third row of rot matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    #full rot matrix
    rot_matrix = np.array([[r00, r01, r02],
                          [r10, r11, r12],
                          [r20, r21, r22]])
    rotational_matrices[id] = rot_matrix
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

def udp_send(sock, ip, port, message):
    sock.sendto(message, (UDP_IP, UDP_PORT))


if __name__ == "__main__":
    sock = init()
    l = 0.2 # meters
    try:
        while True:
            time_start = time.time()

            clientAddress = "192.168.0.63"
            optitrackServerAddress = "192.168.0.4"
            robot_id = 349

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
            z_pid = PID(3, 0, 1, setpoint = 1.5, sample_time=0.01)
            original_yaw = 0
            yaw_pid = PID(0.001, 0, 0.001, setpoint = original_yaw)
            x_pid = PID(0.1, 0, 0.5, setpoint = 1)
            y_pid = PID(0.1, 0, 0.5, setpoint = 3)


            while is_running:
                if robot_id in positions:
                    # last position
                    #print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                    #print(rotations[robot_id][2])
                    # Get the joystick readings
                    #pygame.event.pump()

                    # Each vector is between -1 and 1

                    '''rotT = np.dot(rotational_matrices[robot_id].T,np.array([x_pid(positions[robot_id][0]),
                                                                         y_pid(positions[robot_id][1]),
                                                                         z_pid(positions[robot_id][2])]))'''
                    #print(y_pid(positions[robot_id][1]))
                    #print("Matrix: ", rotational_matrices[robot_id])
                    #print("Flipped Matrix: ", np.flip(rotational_matrices[robot_id], (-0, 1)))
                    #print(rotT)

                    xy_distance = math.sqrt((x_pid.setpoint - positions[robot_id][0])**2 + (y_pid.setpoint - positions[robot_id][1])**2)
                    #print(xy_distance)

                    fx = x_pid(positions[robot_id][0]) #-joystick.get_axis(1) # left handler: up-down, inverted (force of x
                    #taux = joystick.get_axis(0) # left handler: left-right torque
                    #taux = -rotT[1]
                    #print(fx)
                    taux = 0
                    #print(taux)

                    fz = z_pid(positions[robot_id][2]) + 2#0.11 * (target_height - positions[robot_id][2])#-joystick.get_axis(4) # right handler: up-down, inverted
                    #print(fz)
                    if fz > 1:
                        fz = 1
                    elif fz <= 0:
                        fz = 0.001
                    #print(fz)
                    #tauz = -joystick.get_axis(3) # right handler: left-right, manually inverted
                    
                    # If outside a target radius on the xy plane, yaw turns to the target and moves toward it
                    # Once inside the radius, yaw returns to its original setpoint
                    if xy_distance > 0.5:
                        angle = math.acos(abs(x_pid.setpoint - positions[robot_id][0]) / xy_distance) * 180/math.pi
                        #print(angle)
                        if x_pid.setpoint - positions[robot_id][0] < 0:
                            if y_pid.setpoint - positions[robot_id][1] < 0:
                                yaw_pid.setpoint = 180 - angle
                            else:
                                yaw_pid.setpoint = -180 + angle
                        else:
                            if y_pid.setpoint - positions[robot_id][1] < 0:
                                yaw_pid.setpoint = 180 - angle
                            else:
                                yaw_pid.setpoint = -180 + angle
                    else:
                        yaw_pid.setpoint = original_yaw
                    #print(yaw_pid.setpoint)
                    tauz = yaw_pid(rotations[robot_id][2]) # right handler: left-right
                    #print(tauz)
                    #print(fx, taux, fz, tauz)
        
                    f1x = (fx - tauz/l)/2
                    f2x = (fx + tauz/l)/2
                    f1z = (fz + taux/l)/2
                    f2z = (fz - taux/l)/2
                    #print(f1x, f2x, f1z, f2z)
                    MAX_THRUST = 50#17
                    f1 = math.sqrt(f1x**2 + f1z**2) * MAX_THRUST# * 0.05 + 3.95
                    f2 = math.sqrt(f2x**2 + f2z**2) * MAX_THRUST# * 0.05 + 3.95
                    t1 = 180 - math.atan2(f1z, f1x)*180/math.pi
                    t2 = math.atan2(f2z, f2x)*180/math.pi
                    #print(f1, f2, t1, t2)
                    #print(t1, t2)
                    #print(f1, f2)


                    # f1 = 300
                    message = struct.pack('<ffff', f1, f2, t1, t2)
        
                    #print(f1, f2, t1, t2)
                    udp_send(sock, UDP_IP, UDP_PORT, message)
                    # print(message)
                time.sleep(0.01)
    except:
        message = struct.pack('<ffff', 0, 0, 90, 90)
        udp_send(sock, UDP_IP, UDP_PORT, message)