#!/usr/bin/env python
import numpy as np
import math



def ssa(angle):
    """
    Returns the smalles signed angle in [-pi, pi)
    Args:
        angle (float): Tha angle that will be wrapped inside [-pi, pi)
    Returns:
        float: The wrapped angle
    """
    angle = (angle + math.pi) % (2*math.pi) - math.pi
    return angle

def rad2pipi(x):
    y = np.arctan2(np.sin(x),np.cos(x))
    return y
    
def Rzyx(psi):
    """
    Rzyx(psi) computes the rotation matrix, R in SO(3), using the
    zyx convention and Euler angle representation.
    """

    R = np.array([[math.cos(psi), -math.sin(psi), 0],
                  [math.sin(psi), math.cos(psi), 0],
                  [0, 0, 1]])
    return R

def yaw2quat(psi):
    """
    Return the quternions of yaw
    """
    q1 = np.cos(psi/2)
    q4 = np.sin(psi/2)
    quat = np.array((q1, 0, 0, q4))
    return quat

def euler2quat(roll, pitch, yaw):
    """Return quaternions as an array from euler angles

    Args:
        Roll (float): 
        Pitch (float): 
        Yaw (float): 
    """
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    quaternion = np.array([x, y, z, w])
    return quaternion

def quat2eul(w, x, y, z):
    """
    Returns the ZYX roll-pitch-yaw angles from a quaternion.
    """
    q = np.array((w, x, y, z))
    #if np.abs(np.linalg.norm(q) - 1) > 1e-6:
    #   raise RuntimeError('Norm of the quaternion must be equal to 1')

    eta = q[0]
    eps = q[1:]

    S = np.array([
        [0, -eps[2], eps[1]],
        [eps[2], 0, -eps[0]],
        [-eps[1], eps[0], 0]
    ])

    R = np.eye(3) + 2 * eta * S + 2 * np.linalg.matrix_power(S, 2)

    if np.abs(R[2, 0]) > 1.0:
        raise RuntimeError('Solution is singular for pitch of +- 90 degrees')

    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = -np.arcsin(R[2, 0])
    yaw = np.arctan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])

def lowPassFilter(signal, cutOffFrequency):
    filteredSignal = 1/(signal/cutOffFrequency + 1)
    return filteredSignal


def sign(x):
    """Gives the sign  of the input (single value or array). Returns either signle value or array."""
    y = np.zeros(np.size(x))
    if np.size(x)==1:
        if x < 0:
            y = -1
        elif x > 0:
            y = 1
    else:
        count = 0
        for i in x:
            if i < 0:
                y[count] = -1
            elif i > 0:
                y[count] = 1

            count = count + 1
    return y

def sqrt(x):
    """
    Gives the absolute value of the root of either a single value or an array.
    If the input is an array, an array is also returned.
    """
    y = np.zeros(np.size(x))
    if np.size(x)==1:
        y = np.math.sqrt(np.abs(x))
    else:
        count = 0
        for i in range(np.size(x)):
            y[count] = np.math.sqrt(np.abs(x[count]))
            count = count + 1
            
    return y

def transformationMatrix(eta):
        phi = eta[3]
        theta = eta[4]
        psi = eta[5]
        
        cphi = math.cos(phi)
        sphi = math.sin(phi)
        cth  = math.cos(theta)
        sth  = math.sin(theta)
        cpsi = math.cos(psi)
        spsi = math.sin(psi)
        
        J1 = np.array([[1.0, sphi*sth/cth, cphi*sth/cth],
                       [0.0, cphi, -sphi],
                       [0.0, sphi/cth, cphi/cth]])
        J2 = np.array([[cpsi*cth, -spsi*cphi+cpsi*sth*sphi, spsi*sphi+cpsi*cphi*sth],
                       [spsi*cth, cpsi*cphi+sphi*sth*spsi, -cpsi*sphi+sth*spsi*cphi],
                       [-sth, cth*sphi, cth*cphi]])
        
        # J1 = np.array([[math.cos(psi)*math.cos(theta), -math.sin(phi)+math.cos(theta)+math.cos(phi)*math.sin(theta)*math.sin(phi), math.sin(psi)*math.sin(phi)+math.cos(psi)*math.cos(phi)*math.sin(theta)],
        #                [math.sin(psi)*math.cos(theta), math.cos(psi)*math.cos(phi)+math.sin(phi)*math.sin(theta)*math.sin(psi), -math.cos(psi)*math.sin(phi)+math.sin(theta)*math.sin(psi)*math.cos(phi)],
        #                [-math.sin(theta), math.cos(theta)*math.sin(phi), math.cos(theta)*math.cos(phi)]])
        
        
        # J2 = np.array([[1, math.sin(phi)*math.tan(theta), math.cos(phi)*math.tan(theta)],
        #                [0, math.cos(phi), -math.sin(phi)],
        #                [0, math.sin(phi)/math.cos(theta), math.cos(phi)/math.cos(theta)]])
        zero = np.zeros([3, 3])
        Jcol1 = np.concatenate((J1, zero), axis=0)
        Jcol2 = np.concatenate((zero, J2), axis=0)
        J = np.concatenate((Jcol1, Jcol2), axis=1)
        return J

def skewSymmetricMatrix(r):
    if np.size(r)==1:
        r = [0, 0, r]
    S = np.array([[0, -r[2], r[1]],
                  [r[2], 0, -r[0]],
                  [-r[1], r[0], 0]])
    return S


            
def three2sixDof(x):
    y = np.zeros([6,1])
    y[0] = x[0]
    y[1] = x[1]
    y[5] = x[2]
    return y

def six2threeDof(x):
    y = np.array([x[0], x[1], x[5]])
    return y
    


def firstOrderLowPass(signal, signal_previous, filteredSignal_previous, omega_c, dT):
    alpha = (2.0-dT*omega_c)/(2.0+dT*omega_c)
    beta = dT*omega_c/(2.0+dT*omega_c)
    filteredSignal = alpha*filteredSignal_previous + beta*(signal+signal_previous)
    return filteredSignal
    