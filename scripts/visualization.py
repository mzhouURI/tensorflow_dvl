import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, regularizers
# from tensorflow.keras.datasets import mnist
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sys import exit
import time;

np.set_printoptions(precision=3, suppress=True)

#load models
model = keras.models.load_model('model1/')
# model2 = keras.models.load_model('error_model/')
#define which columns of data will be used.
input_use_cols = ['udot','vdot','wdot','p','q','r','volt','sb','hb','hs','s','roll','pitch','yaw','z_dot','z']
# input_use_cols = ['udot','vdot','wdot','q','r','volt','sb','hb','hs','s','pitch','yaw','z_dot']

output_use_cols = ['u','v','w']


#load data setting
test_file_name = "09-27-17.csv"


x_test = pd.read_csv(test_file_name, usecols=input_use_cols)
y_test = pd.read_csv(test_file_name, usecols=output_use_cols)
x_test = x_test.astype("float32")
y_test = y_test.astype("float32")

x_test = x_test.iloc[[0], :]
x_test = x_test.astype("float32")
print(x_test)
# eval = model.evaluate(x_test, y_test, batch_size=32, verbose=2)
#predict velocities and uncertainty
# ms = time.time()*1000.0
y_pre = model.predict(x_test)
# sig_pre = model2.predict(x_test)
# ms2 = time.time()*1000.0

exit(0)
# print(ms2-ms)
# print(ms2)

### convert to tensorflow lite model
converter = tf.lite.TFLiteConverter.from_saved_model('value_model/') # path to the SavedModel directory
tflite_model = converter.convert()
with open('vel_predict.tflite', 'wb') as f:
  f.write(tflite_model)

converter = tf.lite.TFLiteConverter.from_saved_model('error_model/') # path to the SavedModel directory
tflite_model = converter.convert()
with open('sig_predict.tflite', 'wb') as f:
  f.write(tflite_model)
######
exit(0)


#visualize
#compare predict and actual
fig, ax = plt.subplots(2, 3)
p_i = 0
name_i = 'u'

ax[0, 0].plot(y_pre[:, p_i], "-r")
# ax[0, 0].fill_between(np.arange(y_pre.shape[0]), y_pre[:, p_i]-sig_pre[:, p_i], y_pre[:, p_i]+sig_pre[:, p_i])
ax[0, 0].plot(y_test[name_i], "-b")
ax[0, 0].set_title('surge [m/s]')
# ax[0, 0].legend(["predicted", "uncertainty", "actual"])
ax[0, 0].legend(["predicted", "actual"])
ax[1, 0].plot(y_test[name_i] - y_pre[:, p_i], "-k")
ax[1, 0].set_title('difference')
#
p_i = 1
name_i = 'v'
ax[0, 1].plot(y_pre[:, p_i], "-r")
# ax[0, 1].fill_between(np.arange(y_pre.shape[0]), y_pre[:, p_i]-sig_pre[:, p_i], y_pre[:, p_i]+sig_pre[:, p_i])
ax[0, 1].plot(y_test[name_i], "-b")
ax[0, 1].set_title('sway [m/s]')
# ax[0, 1].legend(["predicted", "uncertainty", "actual"])
ax[0, 0].legend(["predicted", "actual"])
ax[1, 1].plot(y_test[name_i] - y_pre[:, p_i], "-k")
ax[1, 1].set_title('difference')

p_i = 2
name_i = 'w'
ax[0, 2].plot(y_pre[:, p_i], "-r")
# ax[0, 2].fill_between(np.arange(y_pre.shape[0]), y_pre[:, p_i]-sig_pre[:, p_i], y_pre[:, p_i]+sig_pre[:, p_i])
ax[0, 2].plot(y_test[name_i], "-b")
ax[0, 2].set_title('heave [m/s]')
# ax[0, 2].legend(["predicted", "uncertainty", "actual"])
ax[0, 0].legend(["predicted", "actual"])
ax[1, 2].plot(y_test[name_i] - y_pre[:, p_i], "-k")
ax[1, 2].set_title('difference')

plt.show()
exit(0)