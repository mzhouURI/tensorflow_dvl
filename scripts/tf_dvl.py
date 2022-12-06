#import ros related stuf
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3Stamped
from mvp_msgs.msg import Float64Stamped
##import tensorflow related stuff
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, regularizers
# from tensorflow.keras.datasets import mnist
import pandas as pd
import numpy as np
# import matplotlib.pyplot as plt
from sys import exit
import time;

np.set_printoptions(precision=3, suppress=True)

x_test = pd.DataFrame({
                        "udot":[], "vdot":[], "wdot":[],
                        "p":[], "q":[], "r":[],
                        "roll":[], "pitch":[], "yaw":[],
                        "volt":[], "sb":[], "hb":[],
                        "hs":[], "s":[],
                        "z_dot":[], "z":[]
                        })

z0 = 0
zt_0 = time.time()*1000.0
model = keras.models.load_model('model1/')


def callback_imu(msg):
    global x_test
    x_test.at[0, 'udot'] = msg.linear_acceleration.x
    x_test.at[0, 'vdot'] = msg.linear_acceleration.y
    x_test.at[0, 'wdot'] = msg.linear_acceleration.z
    x_test.at[0, 'p'] = msg.angular_velocity.x
    x_test.at[0, 'q'] = msg.angular_velocity.y
    x_test.at[0, 'r'] = msg.angular_velocity.z
    # print(x_test)

def callback_euler(msg):
    global x_test
    x_test.at[0, 'roll'] = msg.vector.x
    x_test.at[0, 'pitch'] = msg.vector.y
    x_test.at[0, 'yaw'] = msg.vector.z


def callback_depth(msg):
    global z0
    global x_test
    global zt_0
    dz = msg.data-z0
    zt_now = time.time()*1000.0
    zt_0 = zt_now
    z0 = msg.data
    x_test.at[0, 'z'] = msg.data
    x_test.at[0, 'z_dot'] = dz/zt_now

def callback_s_thrust(msg):
    global x_test
    x_test.at[0, 's'] = msg.data

def callback_sb_thrust(msg):
    global x_test
    x_test.at[0, 'sb'] = msg.data

def callback_hb_thrust(msg):
    global x_test
    x_test.at[0, 'hb'] = msg.data

def callback_hs_thrust(msg):
    global x_test
    x_test.at[0, 'hs'] = msg.data

def callback_volt(msg):
    global x_test
    x_test.at[0, 'volt'] = msg.data

def compute_vel():
    global x_test
    global model
    if not x_test.empty:
            # print(rospy.get_rostime())
            y_pre = model.predict(x_test, verbose='none',use_multiprocessing=True)
            # print(rospy.get_rostime())

            msg = TwistWithCovarianceStamped()
            msg.twist.twist.linear.x = y_pre[0, 0]
            msg.twist.twist.linear.y = y_pre[0, 1]
            msg.twist.twist.linear.z = y_pre[0, 2]
            msg.header.frame_id = "alpha/dvl"
            msg.header.stamp = rospy.get_rostime()

            pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('tensorflow_dvl') 
    pub = rospy.Publisher("/tensor_flow_dvl", TwistWithCovarianceStamped, queue_size=10)
    imu_sub = rospy.Subscriber("/imu/data", Imu, callback_imu)
    euler_sub = rospy.Subscriber("/imu/euler/radians",Vector3Stamped,callback_euler)
    pressure_sub = rospy.Subscriber("/depth", Float64Stamped, callback_depth)
    s_thrust_sub = rospy.Subscriber("/control/thruster/surge", Float64, callback_s_thrust)
    sb_thrust_sub = rospy.Subscriber("/control/thruster/sway_bow", Float64, callback_sb_thrust)
    hb_thrust_sub = rospy.Subscriber("/control/thruster/heave_bow", Float64, callback_hb_thrust)
    hs_thrust_sub = rospy.Subscriber("/control/thruster/heave_stern", Float64, callback_hs_thrust)
    volt_sub = rospy.Subscriber("/power/voltage", Float64Stamped, callback_volt)
    rate = rospy.Rate(50)

    zt_0=time.time()*1000.0
    while not rospy.is_shutdown():
        compute_vel()
        # print(x_test)        
        rate.sleep()
# exit(0)