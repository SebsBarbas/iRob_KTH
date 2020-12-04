#! /usr/bin/env python3

"""
    # {Sebastian Barbas Laina}
    # {ssbl@kth.se}
"""
from math import atan2, pow, sin, cos, sqrt, pi
import numpy

L=0.39
M=0.4


def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    l0=0.07
    l1=0.3
    l2=0.35
    robot_frame_point=[x-l0,y,z]
    aid=(pow(robot_frame_point[0],2)+pow(robot_frame_point[1],2)-pow(l1,2)-pow(l2,2))/(2*l1*l2)
    q[1]=atan2(-sqrt(1-pow(aid,2)),aid)
    q[0]=atan2(robot_frame_point[1],robot_frame_point[0])-atan2((l2*sin(q[1])),(l1+l2*cos(q[1])))
    q[2]=robot_frame_point[2]

    return q

def Trans_mat(alpha, a, d, theta):
   
    trans_mat = numpy.array([[ cos(theta),     -sin(theta)*cos(alpha),      sin(theta)*sin(alpha),     a*cos(theta)],
                    [  sin(theta),      cos(theta)*cos(alpha),     -cos(theta)*sin(alpha),     a*sin(theta)],
                    [           0,                 sin(alpha),                 cos(alpha),                d],
                    [           0,                          0,                          0,                1]])

    return trans_mat

def Jacobian_ps_inv(q):
    T1 = Trans_mat(pi/2, 0, 0.311, q[0])
    T2 = Trans_mat(-pi/2, 0, 0, q[1])
    T3 = Trans_mat(-pi/2, 0, M, q[2])
    T4 = Trans_mat(pi/2, 0, 0, q[3])
    T5 = Trans_mat(pi/2, 0, L, q[4])
    T6 = Trans_mat(-pi/2, 0, 0, q[5])
    T7 = Trans_mat(0, 0, 0.078, q[6])

    T02 = numpy.dot(T1, T2)
    T03 = numpy.dot(T02, T3)
    T04 = numpy.dot(T03, T4)
    T05 = numpy.dot(T04, T5)
    T06 = numpy.dot(T05, T6)
    T07 = numpy.dot(T06, T7)  

    all_mat = [T1, T02, T03, T04, T05, T06, T07]

    pos_end_eff = T07[0:3, 3]

    z_vect = [0, 0, 1]
    pos_joint = [0, 0, 0]
    aid=numpy.cross(z_vect, numpy.subtract(pos_end_eff, pos_joint))
    Jacobian = numpy.append(aid, z_vect)
    
    for i in range(0, 6):
        mat = all_mat[i]
        z_vect = mat[0:3, 2]
        pos_joint = mat[0:3, 3]
        aid=numpy.cross(z_vect, numpy.subtract(pos_end_eff, pos_joint))
        Jacobian_i = numpy.append(aid, z_vect)
        Jacobian = numpy.c_[Jacobian, Jacobian_i]

    Jacobian_trans=numpy.transpose(Jacobian)
    Jacobian_inv = numpy.linalg.inv(numpy.dot(Jacobian,Jacobian_trans))
    Jacobian_ps_inv = numpy.dot(Jacobian_trans, Jacobian_inv)

    return Jacobian_ps_inv


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    tolerances = [0.02, 0.02, 0.02, 0.01, 0.01, 0.01]
    q_ans = q
    R = numpy.array(R)
    print(point)

    while True:

        T1 = Trans_mat(pi/2, 0, 0.311, q_ans[0])
        T2 = Trans_mat(-pi/2, 0, 0, q_ans[1])
        T3 = Trans_mat(-pi/2, 0, M, q_ans[2])
        T4 = Trans_mat(pi/2, 0, 0, q_ans[3])
        T5 = Trans_mat(pi/2, 0, L, q_ans[4])
        T6 = Trans_mat(-pi/2, 0, 0, q_ans[5])
        T7 = Trans_mat(0, 0, 0.078, q_ans[6])

        T02 = numpy.dot(T1, T2)
        T03 = numpy.dot(T02, T3)
        T04 = numpy.dot(T03, T4)
        T05 = numpy.dot(T04, T5)
        T06 = numpy.dot(T05, T6)
        T07 = numpy.dot(T06, T7)  

        vector_n=numpy.cross(T07[0:3, 0], R[0:3, 0])
        vector_o=numpy.cross(T07[0:3, 1], R[0:3, 1])
        vector_a=numpy.cross(T07[0:3, 2], R[0:3, 2])
        erot = numpy.multiply(0.5,(vector_n + vector_o + vector_a))

        epos = numpy.subtract(T07[0:3, 3], [x, y, z])
        etot = numpy.append(epos, erot)
        
        jacobian_ps_inv = Jacobian_ps_inv(q)
        ejoint = numpy.dot(jacobian_ps_inv, etot) 

        print(etot)

        if numpy.all(numpy.absolute(etot) < tolerances):
            break 
        else:
            q_ans = numpy.subtract(q_ans, ejoint)

    print("Aqui estoy")


    return q_ans
