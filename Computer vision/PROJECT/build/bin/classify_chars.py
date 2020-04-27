from __future__ import absolute_import, division, print_function

# TensorFlow and tf.keras
import tensorflow as tf
import keras
from keras.utils import CustomObjectScope
from keras.initializers import glorot_uniform
from keras.preprocessing import image
from keras.models import Sequential, load_model, model_from_json
# Helper libraries
import numpy as np
import glob
import cv2
import scipy.io as sio
import os

print(tf.__version__)

def main():

    class_names = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                   'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
                   'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
                   'U', 'V', 'W', 'X', 'Y', 'Z']

    img_shape = 20

    # load a file that cointain the structure of the trained model
    json_file = open('model/neural_network.json', 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    with CustomObjectScope({'GlorotUniform': glorot_uniform()}):
        model = model_from_json(loaded_model_json)
    # load the weights of the trained model
    model.load_weights("model/neural_network.h5")

    # open file that will contain the license plate numbers (strings)
    f = open('licencePlates.txt', 'w' )

    # path that contains the images of licence plate chars, each image contain chars (20x20 images)
    # concatenate each other (the dimension of the image will be #ofchars x 20)
    fn = "licence_plates/*.jpg"

    # extract image names from the path
    filenames = glob.glob(fn)
    filenames.sort()
    images = []

    # load images and save them in a vector of images
    for img in filenames:
        image = cv2.imread(img)
        images.append(image)

    for img in images:
        S = ''
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)/255
        # extract each char (20x20) from the image
        for j in range(int(img.size/(img_shape*img_shape))):
            char = img[:,img_shape*j:img_shape*(j+1)]
            cv2.transpose(char,char)
            char = char.reshape((-1, img_shape, img_shape, 1), order="F")
            # predict the label of the char
            predictor = model.predict(char)
            max_prob = np.argmax(predictor)
            # concatenate chars in order to obtain a string with the number of the licence plate
            S = S + class_names[max_prob]

        S = S + "\n"
        # the plates are in the same order of the images of the dataset
        print("Plate detected: " + S)
        # save the string, it will then be loaded in c++ program
        f.write(S)

    f.close()
    keras.backend.clear_session()

if __name__ == '__main__':
    main()
