from __future__ import division
import math
import numpy as np
from numpy import linalg as LA
import casadi as ca
import yaml
import os
# from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

X = 0
Y = 1
Z = 2

VX = 3
VY = 4
VZ = 5

QX = 9
QY = 10
QZ = 11
QW = 12

WX = 6
WY = 7
WZ = 8

Roll = 9
Pitch = 10
Yaw = 11

DLT_TMPC_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROLLERS_PATH = DLT_TMPC_PATH + '/controllers/'
MODELS_PATH = DLT_TMPC_PATH + '/models/'

def euler_to_quaternion(roll, pitch, yaw):
    """
    Perform a conversion from euler angle to quaternion
    :param roll: roll angle
    :type roll: float
    :param pitch: pitch angle
    :type pitch: float
    :param yaw: yaw angle 
    :type yaw: float
    :return: Quaternion
    :rtype: list
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    # qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    # qy = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    # qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    # qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def quaternion_to_euler(quaternion):
    """
    Perform a conversion from quaternion to euler angle 
    :param quaternion: quaternion
    :type quaternion: ca.DM, list, np.array
    :return: Euler angles [roll, pitch, yaw]
    :rtype: list of float
    """
    [x, y, z, w] = quaternion
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return [roll, pitch, yaw]

def CA_Norm_Quaternions(quaternion): 
    """
    Normalize quaternion
    :param quaternion: quaternion
    :type q_t: ca.DM, list, np.array
    :return: Quaternions components
    :rtype: ca.DM, ca.DM, ca.Dm, ca.DM
    """
    if ca.norm_1(quaternion) == 0 : 
        return 0,0,0,1

    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]
    
    x = x/ca.norm_1(quaternion)
    y = y/ca.norm_1(quaternion)
    z = z/ca.norm_1(quaternion)
    w = w/ca.norm_1(quaternion)

    return x,y,z,w

def r_mat(q):
    """
    Generate rotation matrix from unit quaternion
    :param q: unit quaternion
    :type q: ca.MX
    :return: rotation matrix, SO(3)
    :rtype: ca.MX
    """

    Rmat = ca.MX(3, 3)

    # Extract states
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]

    Rmat[0, 0] = qw**2 + qx**2 - qy**2 - qz**2
    Rmat[0, 1] = 2*qx*qy - 2*qz*qw
    Rmat[0, 2] = 2*qx*qz + 2*qy*qw

    Rmat[1, 0] = 2*qx*qy + 2*qz*qw
    Rmat[1, 1] = qw**2 - qx**2 + qy**2 - qz**2
    Rmat[1, 2] = 2*qy*qz - 2*qx*qw

    Rmat[2, 0] = 2*qx*qz - 2*qy*qw
    Rmat[2, 1] = 2*qy*qz + 2*qx*qw
    Rmat[2, 2] = qw**2 - qx**2 - qy**2 + qz**2

    return Rmat

def r_mat_euler(angle):
    """
    Generate rotation matrix from euler angle
    """

    Rmat = ca.MX(3, 3)

    # Extract states
    phi = angle[0]
    theta = angle[1]
    psi = angle[2]

    c_phi = ca.cos(phi)
    c_theta = ca.cos(theta)
    c_psi = ca.cos(psi)

    s_phi = ca.sin(phi)
    s_theta = ca.sin(theta)
    s_psi = ca.sin(psi)

    Rmat[0, 0] = c_psi * c_theta
    Rmat[0, 1] = c_psi * s_theta * s_phi - s_psi * c_phi
    Rmat[0, 2] = s_psi * s_phi + c_psi * c_phi * s_theta

    Rmat[1, 0] = s_psi * c_theta
    Rmat[1, 1] = c_psi * c_phi + s_phi * s_theta * s_psi
    Rmat[1, 2] = s_theta * s_psi * s_phi - c_psi * s_phi

    Rmat[2, 0] = - c_theta
    Rmat[2, 1] = c_theta * s_phi
    Rmat[2, 2] = c_theta * c_phi 


    return Rmat.T
    
def inv_skew(sk):
    """
    Retrieve the vector from the skew-symmetric matrix.
    :param sk: skew symmetric matrix
    :type sk: ca.MX
    :return: vector corresponding to SK matrix
    :rtype: ca.MX
    """

    v = ca.MX.zeros(3, 1)

    v[0] = sk[2, 1]
    v[1] = sk[0, 2]
    v[2] = sk[1, 0]

    return v
    
def skew(v):
    """
    Returns the skew matrix of a vector v
    :param v: vector
    :type v: ca.MX
    :return: skew matrix of v
    :rtype: ca.MX
    """

    sk = ca.MX.zeros(3, 3)

    # Extract vector components
    x = v[0]
    y = v[1]
    z = v[2]

    sk[0, 1] = -z
    sk[1, 0] = z
    sk[0, 2] = y
    sk[2, 0] = -y
    sk[1, 2] = -x
    sk[2, 1] = x

    return sk

def q_err(q_t, q_r):
    """
    Compute angular error between two unit quaternions.
    :param q_t: New quaternion
    :type q_t: ca.DM, list, np.array
    :param q_r: Reference quaternion
    :type q_r: ca.DM, list, np.array
    :return: vector corresponding to SK matrix
    :rtype: ca.DM
    """
    q_upper_t = [q_r[3],-q_r[0],-q_r[1],-q_r[2]]
    q_lower_t = [q_t[3],q_t[0],q_t[1],q_t[2]]

    qd_t = [q_upper_t[0]*q_lower_t[0] - q_upper_t[1]*q_lower_t[1] - q_upper_t[2]*q_lower_t[2] - q_upper_t[3]*q_lower_t[3],
            q_upper_t[1]*q_lower_t[0] + q_upper_t[0]*q_lower_t[1] + q_upper_t[2]*q_lower_t[3] - q_upper_t[3]*q_lower_t[2],
            q_upper_t[0]*q_lower_t[2] - q_upper_t[1]*q_lower_t[3] + q_upper_t[2]*q_lower_t[0] + q_upper_t[3]*q_lower_t[1],
            q_upper_t[0]*q_lower_t[3] + q_upper_t[1]*q_lower_t[2] - q_upper_t[2]*q_lower_t[1] + q_upper_t[3]*q_lower_t[0]]

    phi_t   = ca.atan2( 2 * (qd_t[0] * qd_t[1] + qd_t[2] * qd_t[3]), 1 - 2 * (qd_t[1]**2 + qd_t[2]**2) )
    theta_t = ca.asin ( 2 * (qd_t[0] * qd_t[2] - qd_t[3] * qd_t[1]) )
    psi_t   = ca.atan2( 2 * (qd_t[0] * qd_t[3] + qd_t[1] * qd_t[2]), 1 - 2 * (qd_t[2]**2 + qd_t[3]**2) )

    return ca.vertcat(phi_t,theta_t,psi_t)

    def xi_mat(q):
        """
        Generate the matrix for quaternion dynamics Xi,
        from Trawney's Quaternion tutorial.
        :param q: unit quaternion
        :type q: ca.MX
        :return: Xi matrix
        :rtype: ca.MX
        """
        Xi = ca.MX(4, 3)

        # Extract states
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]

        # Generate Xi matrix
        Xi[0, 0] = qw
        Xi[0, 1] = -qz
        Xi[0, 2] = qy

        Xi[1, 0] = qz
        Xi[1, 1] = qw
        Xi[1, 2] = -qx

        Xi[2, 0] = -qy
        Xi[2, 1] = qx
        Xi[2, 2] = qw

        Xi[3, 0] = -qx
        Xi[3, 1] = -qy
        Xi[3, 2] = -qz

        return Xi

    def xi_mat_euler(q):
        """
        """
        Xi = ca.MX(3, 3)

        # Extract states
        phi = angle[0]
        theta = angle[1]
        psi = angle[2]

        c_phi = ca.cos(phi)
        c_theta = ca.cos(theta)
        c_psi = ca.cos(psi)

        s_phi = ca.sin(phi)
        s_theta = ca.sin(theta)
        s_psi = ca.sin(psi)

        # Generate Xi matrix
        Xi[0, 0] = 1
        Xi[0, 1] = s_phi * s_theta / c_theta
        Xi[0, 2] = c_phi * s_theta / c_theta

        Xi[1, 0] = 0
        Xi[1, 1] = c_phi
        Xi[1, 2] = -s_phi

        Xi[2, 0] = 0
        Xi[2, 1] = s_phi / c_theta
        Xi[2, 2] = c_phi / c_theta

        return Xi

def pose2state(pose, dynamic):
    
    if "Euler" : 
        new_ref = np.zeros(12)
        p = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        a = quaternion_to_euler([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
       
        new_ref[0:3] = p
        new_ref[9:] = a

    elif "Quaternion":
        new_ref = np.zeros(13)
        p = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        a = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        
        new_ref[0:3] = p
        new_ref[9:] = a

    return new_ref


def read_default_params_mpc(path):
    data = {}

    with open(path) as file_d:
        data = yaml.load(file_d, Loader=yaml.FullLoader)
    file_d.close()

    Q = np.diag(data['Q']['position'] + data['Q']['velocity'] + data['Q']['angular_velocity'] + data['Q']['attitude'])
    R = np.diag(data['R']['thrust'] + data['R']['torque'])
    uub = np.array(data['const']['uu']["thrust"] + data['const']['uu']["torque"])
    lub = np.array(data['const']['ul']["thrust"] + data['const']['ul']["torque"])
    uxb = np.array(data['const']['xu']["position"] + data['const']['xu']["velocity"] + data['const']['xu']["angular_velocity"] + data['const']['xu']["attitude"])
    lxb = np.array(data['const']['xl']["position"] + data['const']['xl']["velocity"] + data['const']['xl']["angular_velocity"] + data['const']['xl']["attitude"])
    xt = np.array(data['const']['xt']["position"] + data['const']['xt']["velocity"] + data['const']['xt']["angular_velocity"] + data['const']['xt']["attitude"])
    horizon = data['horizon']

    return Q, R, Q*100, uub, lub, uxb, lxb, xt, horizon

def read_default_params_quad(path):
    data = {}

    with open(path) as file_d:
        data = yaml.load(file_d, Loader=yaml.FullLoader)
    file_d.close()

    g = data['g']
    m = data['mass']
    Ixx = data['inertia']['xx']
    Iyy = data['inertia']['yy']
    Izz = data['inertia']['zz']
    dynamics = data['dynamic']

    return g, m, Ixx, Iyy, Izz, dynamics
