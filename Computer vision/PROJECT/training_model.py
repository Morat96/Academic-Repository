from __future__ import absolute_import, division, print_function
from matplotlib.pyplot import imshow
import matplotlib.pyplot as plt

# TensorFlow and tf.keras
import tensorflow as tf
from tensorflow import keras

# Helper libraries
import numpy as np
import scipy.io as sio
import os

from sklearn.metrics import confusion_matrix
from keras.models import model_from_json
from numpy import genfromtxt

# load the dataset ARIAL.csv
data = genfromtxt('ARIAL.csv',delimiter = ',')
img_shape = 20

# load the first 8950 sample
stop_idx = 8950
# save the labels
label = data[1:stop_idx,2]
# save the features (grayscale intensities from 0 to 255)
im_buf = data[1:stop_idx,12:]

k=0
# keep only numbers and capital letters from the dataset
for i in range(stop_idx-1):
    # numbers have ascii codes between 48 and 57
    if(label[i-k] > 47 and label[i-k] < 58):
        label[i-k] = label[i-k] - 48
    else:
        # capital letters have ascii codes between 65 and 90
        if(label[i-k] > 64 and label[i-k] < 91):
            label[i-k] = label[i-k] - 55
        else:
            # discard not useful data
            im_buf = np.delete(im_buf, i-k , axis=0)
            label = np.delete(label, i-k, axis=0)
            k = k+1

# reshape the data
train_images = im_buf.reshape((-1, img_shape, img_shape, 1), order="F")
train_labels = label.reshape((-1,))

train_images = train_images/255
num_classes = 36

input = keras.layers.Input(shape=(img_shape, img_shape, 1), name='input0')

# PARAMETERS
# Convolutional Layer 1
kernel_size1 = 5          # Convolution filters are 5 x 5 pixels
num_filters1 = 18         # There are 18 of these filters

# Convolutional Layer 2
filter_size2 = 5          # Convolution filters are 5 x 5 pixels
num_filters2 = 26         # There are 26 of these filters

# Convolutional Layer 3
filter_size3 = 5          # Convolution filters are 5 x 5 pixels
num_filters3 = 24         # There are 24 of these filters

# Fully connected layer
fc_size = 144

# NEURAL NETWORK ARCHITECTURE
# First convolutional layer
conv1 = keras.layers.Conv2D(filters = num_filters1, kernel_size=kernel_size1, strides=(1, 1), padding='same',
                            data_format='channels_last', activation=tf.nn.relu, use_bias=True,
                            kernel_initializer='glorot_uniform', bias_initializer='zeros', name='conv1')(input)

# 2x2 pooling layer
pool1 = keras.layers.MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid',
                                  data_format='channels_last', name='pool1')(conv1)

# Second convolutional layer
conv2 = keras.layers.Conv2D(filters = num_filters2, kernel_size=filter_size2, strides=(1, 1), padding='same',
                            data_format='channels_last', activation=tf.nn.relu, input_shape=(img_shape, img_shape),
                            use_bias=True, kernel_initializer='glorot_uniform', bias_initializer='zeros', name='conv2')(pool1)

# 2x2 pooling layer
pool2 = keras.layers.MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', data_format='channels_last', name='pool2')(conv2)

# Third convolutional layer
conv3 = keras.layers.Conv2D(filters = num_filters3, kernel_size=filter_size3, strides=(1, 1), padding='same',
                            data_format='channels_last', activation=tf.nn.relu, input_shape=(img_shape, img_shape),
                            use_bias=True, kernel_initializer='glorot_uniform', bias_initializer='zeros', name='conv3')(pool2)

# 2x2 pooling layer
pool3 = keras.layers.MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', data_format='channels_last', name='pool3')(conv3)

flat = keras.layers.Flatten()(pool3)

# Dense layer
fc1 = keras.layers.Dense(fc_size, activation=tf.nn.relu,
                         kernel_initializer='glorot_uniform', bias_initializer='zeros', name='fc1')(flat)

# final layer that returns the probability for each class that the input image belongs to that class
output = keras.layers.Dense(num_classes, activation=tf.nn.softmax,
                            kernel_initializer='glorot_uniform', bias_initializer='zeros', name='fc2')(fc1)

model = keras.models.Model(inputs=[input], outputs=[output])

model.summary()

adam = keras.optimizers.Adam(lr=0.001, beta_1=0.9, beta_2=0.999)

model.compile(optimizer=adam,
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

# dataset is divided into training and validation set (10% of the total)
# train for 8 epochs
history = model.fit(train_images, train_labels,
                    batch_size = 128, epochs = 8, validation_split = 0.1)

# save the neural network model
model_json = model.to_json()
with open("neural_network.json","w") as json_file:
    json_file.write(model_json)

model.save_weights("neural_network.h5")
print("Model saved to disk")

keras.backend.clear_session()
