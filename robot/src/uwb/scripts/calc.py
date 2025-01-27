#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt64MultiArray
import sympy as sp
from sympy.solvers import solve

REALTIME = False

def timesCallback(msg):
    p_tx = msg.data[0]
    p_rx = msg.data[1]
    r_tx = msg.data[2]
    r_rx = msg.data[3]
    f_tx = msg.data[4]
    f_rx = msg.data[5]
    if not REALTIME: print("p_tx:", p_tx)
    if not REALTIME: print("p_rx:", p_rx)
    if not REALTIME: print("r_tx:", r_tx)
    if not REALTIME: print("r_rx:", r_rx)
    if not REALTIME: print("t_tx:", f_tx)
    if not REALTIME: print("t_rx:", f_rx)

    r_a = r_rx - p_tx
    r_b = f_rx - r_tx
    d_a = f_tx - r_rx
    d_b = r_tx - p_rx
    if not REALTIME: print("r_a:", r_a)
    if not REALTIME: print("d_b:", d_b)
    if not REALTIME: print("r_b:", r_b)
    if not REALTIME: print("d_a:", d_a)

    if not REALTIME: print("d_a + d_b:", d_a + d_b)

    tof_sim_a = (r_a - d_b) / 2
    if not REALTIME: print("tof_ss-twr_a:", tof_sim_a)

    tof_sim_b = (r_b - d_a) / 2
    if not REALTIME: print("tof_ss-twr_b:", tof_sim_b)

    tof_avg = (tof_sim_a + tof_sim_b) / 2
    if not REALTIME: print("tof_sds-twr:", tof_avg)

    tof_dtu = (r_a * r_b - d_a * d_b) / (r_a + r_b + d_a + d_b)
    if not REALTIME: print("tof_ads-twr:", tof_dtu)

    err_a = 2 * (tof_dtu - tof_sim_a) / d_b
    if not REALTIME: print("e_a - e_b (sstwr):", err_a)

    err_b = 2 * (tof_dtu - tof_sim_b) / d_a
    if not REALTIME: print("e_b - e_a (sstwr):", err_b)

    err_a = 4 * (tof_dtu - tof_avg) / (d_b - d_a)
    if not REALTIME: print("e_a - e_b (sdstwr):", err_a)

    t, a, b = sp.symbols('t, a, b')
    eq1 = t - tof_sim_a + d_b / 2 * (a-b)
    eq2 = t - tof_sim_b + d_a / 2 * (b-a)
    eq3 = t - tof_avg + (d_b - d_a) / 4 * (a-b)
    eq4 = t - tof_dtu + t / 2 * (a + b)
    e_a, e_b, t = solve([eq1, eq2, eq3, eq4], [t, a, b], dict=True)[0].values()
    print("e_a - e_b:", e_a - e_b)
    print("e_b - e_a:", e_b - e_a)
    print("e_a:", e_a, "e_b:", e_b, "t:", t)

    print("r_a > d_b:", r_a > d_b, "r_b > d_a:", r_b > d_a)

if __name__ == '__main__':
    if not REALTIME:
        msg = UInt64MultiArray()
        msg.data = [475785643060, 383905949084, 384169093172, 476048792718, 476339763765, 384460073765]
        timesCallback(msg)
        exit()

    rospy.init_node('calc')
    rospy.Subscriber("/uwb/times", UInt64MultiArray, timesCallback)
    rospy.spin()


# interesting times recorded:
#short distance:
msg.data = [968258005044, 297253391686, 297516538932, 968521152341, 968812017717, 297807404214]
msg.data = [359596744244, 196567298883, 196830341684, 359859786964, 360150856245, 197121410912]
#~2m distance:
msg.data = [649801818676, 201866349002, 202129350196, 650064820785, 650355932725, 202420463667]
#~20m distance:
msg.data = [943532611636, 711864963760, 712128126004, 943795780378, 944086817845, 712419172433]
msg.data = [475785643060, 383905949084, 384169093172, 476048792718, 476339763765, 384460073765]