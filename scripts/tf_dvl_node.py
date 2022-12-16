
#import ros related stuf
import rospy
from std_msgs.msg import Float64
from tensorflow_dvl.msg import TFInputs
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3Stamped
from mvp_msgs.msg import Float64Stamped
##import tensorflow related stuff
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, regularizers
import pandas as pd
import numpy as np
# import matplotlib.pyplot as plt
from sys import exit
import time;

np.set_printoptions(precision=3, suppress=True)

# x_test = pd.DataFrame({
#                         'udot':[], 'vdot':[], 'wdot':[],
#                         'p':[], 'q':[], 'r':[],
#                         'roll':[], 'pitch':[],
#                         'volt':[],'amp':[], 'sb':[], 'hb':[],
#                         'hs':[], 's':[],
#                         'z_dot':[],
#                         'z':[]
#                         })

x_test = pd.DataFrame({
                        'udot':[], 'vdot':[], 'wdot':[],
                        'p':[], 'q':[], 'r':[],
                        'roll':[], 'pitch':[],
                        'volt':[],'amp':[],
                        'sb':[], 'hb':[],
                        'hs':[], 's':[],
                        'z_dot':[], 'z':[]
                        })
z0 = 0
zt_0 = time.time()

u_model = keras.models.load_model( rospy.get_param('/tensorflow_dvl/u_model_dir') )
v_model = keras.models.load_model( rospy.get_param('/tensorflow_dvl/v_model_dir') )
w_model = keras.models.load_model( rospy.get_param('/tensorflow_dvl/w_model_dir') )
pub_hz = rospy.get_param('/tensorflow_dvl/hz') 
enable_depth = rospy.get_param('/tensorflow_dvl/enable_depth') 

dvl_msg = TwistWithCovarianceStamped()
test_msg = TFInputs()

def callback_imu_accel(msg):
    global x_test
    x_test.at[0, 'udot'] = msg.vector.x/10.0
    x_test.at[0, 'vdot'] = msg.vector.y/10.0
    x_test.at[0, 'wdot'] = (msg.vector.z+9.8)/10.0
    # test_msg.udot = msg.vector.x
    # test_msg.vdot = msg.vector.y
    # test_msg.wdot = msg.vector.z

    # test_msg.header.stamp = msg.header.stamp
    dvl_msg.header.stamp = msg.header.stamp


def callback_imu_pqr(msg):
    global x_test
    x_test.at[0, 'p'] = msg.vector.x
    x_test.at[0, 'q'] = msg.vector.y
    x_test.at[0, 'r'] = msg.vector.z
    # test_msg.p = msg.vector.x
    # test_msg.q = msg.vector.y
    # test_msg.r = msg.vector.z
    # dvl_msg.header.stamp = msg.header.stamp
    
def callback_euler(msg):
    global x_test
    x_test.at[0, 'roll'] = msg.vector.x/4.0
    x_test.at[0, 'pitch'] = msg.vector.y/4.0
    # test_msg.roll = msg.vector.x
    # test_msg.pitch = msg.vector.y
    # x_test.at[0, 'yaw'] = msg.vector.z


def callback_depth(msg):
    global z0
    global x_test
    global zt_0
    msg2 = Float64Stamped()
    dz = msg.data-z0
    zt_now = time.time()
    dt = zt_now-zt_0
    zt_0 = zt_now
    z0 = msg.data
    x_test.at[0, 'z'] = msg.data/5.0
    x_test.at[0, 'z_dot'] = dz/dt
    # test_msg.zdot = dz/dt
    # msg2.data = dz/dt
    # zdot_pub.publish(msg2)

def callback_s_thrust(msg):
    global x_test
    x_test.at[0, 's'] = msg.data
    # test_msg.s = msg.data

def callback_sb_thrust(msg):
    global x_test
    x_test.at[0, 'sb'] = msg.data
    # test_msg.sb = msg.data

def callback_hb_thrust(msg):
    global x_test
    x_test.at[0, 'hb'] = msg.data
    # test_msg.hb = msg.data

def callback_hs_thrust(msg):
    global x_test
    x_test.at[0, 'hs'] = msg.data
    # test_msg.hs = msg.data

def callback_volt(msg):
    global x_test
    x_test.at[0, 'volt'] = msg.data/20.00

def callback_amp(msg):
    global x_test
    x_test.at[0, 'amp'] = msg.data/20000.00
    
def compute_vel():
    global x_test
    global z0
    global model
    global dvl_msg
    global test_msg
    if not x_test.empty:
        # print(x_test.isnull().values.any())
            #update the velocity if vehicle is below the surface
        # print(x_test.iloc[[0],0:3])
        # print(x_test.iloc[[0],3:6])
        # print(x_test.iloc[[0],6:9])
        # print(x_test.iloc[[0],9:12])
        # print(x_test.iloc[[0],12:15])
        # print(x_test.iloc[[0],15:16])
        # print(x_test.rank())
        # print(list(x_test))
        if z0 > enable_depth:
            # print(x_test)
            # u_pre = u_model.predict(x_test, verbose='none',use_multiprocessing=True)
            # v_pre = v_model.predict(x_test, verbose='none',use_multiprocessing=True)
            # w_pre = v_model.predict(x_test, verbose='none',use_multiprocessing=True)
            u_pre = u_model(x_test.astype("float32").to_numpy(), training=False)
            v_pre = v_model(x_test.astype("float32").to_numpy(), training=False)
            w_pre = w_model(x_test.astype("float32").to_numpy(), training=False)
            # dvl_msg.twist.twist.linear.x = u_pre[0, 0]
            # dvl_msg.twist.twist.linear.y = v_pre[0, 0]
            # dvl_msg.twist.twist.linear.z = w_pre[0, 0]
            dvl_msg.twist.twist.linear.x = u_pre
            dvl_msg.twist.twist.linear.y = v_pre
            dvl_msg.twist.twist.linear.z = w_pre
        dvl_msg.header.frame_id = "alpha/dvl"
        pub.publish(dvl_msg)
        # test_pub.publish(test_msg)

def callback_dvl(msg):
    global x_test
    global z0
    if not x_test.empty:
    ## only update the velocity from dvl if the vehicle is above the surface
    #  if x_test.at[0, 'z']<enable_depth:
        if z0 < enable_depth:
            dvl_msg.twist.twist.linear.x = msg.twist.twist.linear.x
            dvl_msg.twist.twist.linear.y = msg.twist.twist.linear.y
            dvl_msg.twist.twist.linear.z = msg.twist.twist.linear.z


if __name__ == '__main__':
    rospy.init_node('tf_dvl_node') 
    pub = rospy.Publisher("/tensorflow_dvl/twist", TwistWithCovarianceStamped, queue_size=10)
    # zdot_pub = rospy.Publisher("/tensorflow_dvl/zdot",Float64Stamped, queue_size=10)
    # test_pub = rospy.Publisher("/xtest", TFInputs, queue_size=10)
    accel_sub = rospy.Subscriber("/imu/acceleration", Vector3Stamped, callback_imu_accel)
    pqr_sub = rospy.Subscriber("/imu/angular_velocity", Vector3Stamped, callback_imu_pqr)
    euler_sub = rospy.Subscriber("/imu/euler/radians",Vector3Stamped,callback_euler)
    pressure_sub = rospy.Subscriber("/depth", Float64Stamped, callback_depth)
    s_thrust_sub = rospy.Subscriber("/control/thruster/surge", Float64, callback_s_thrust)
    sb_thrust_sub = rospy.Subscriber("/control/thruster/sway_bow", Float64, callback_sb_thrust)
    hb_thrust_sub = rospy.Subscriber("/control/thruster/heave_bow", Float64, callback_hb_thrust)
    hs_thrust_sub = rospy.Subscriber("/control/thruster/heave_stern", Float64, callback_hs_thrust)
    volt_sub = rospy.Subscriber("/power/voltage", Float64Stamped, callback_volt)
    amp_sub = rospy.Subscriber("/power/current", Float64Stamped, callback_amp)
    dvl_sub = rospy.Subscriber("/dvl/twist", TwistWithCovarianceStamped, callback_dvl)
    rate = rospy.Rate(pub_hz)

    zt_0=time.time()
    while not rospy.is_shutdown():
        compute_vel()
        rate.sleep()
# exit(0)