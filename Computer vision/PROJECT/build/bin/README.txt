How the program works:

The program load images of the dataset then proceed in this way:
For each image,
Phase 1: find a window that contain the licence plate,
Phase 2: find the licence plate,
Phase 3: segment the plate and save its digits. 
At the end of each phase it shows a confirmation that the process has been successful.
In the second part, the program load the licencePlates.txt file (its content is described below) and allows the user to choose an image with a number, showing the found licence plate (with a green bounding box) and its number (above the plate, in green). After pressing a key, the user can decide whether to choose other cars or exit the program.


This folder contain:

Project: Executable (for test the program digit: ./Project), the program load all images of dataset folder.

dataset: Folder that contains cars images.

obj.png: Sample of a license plate: used in the first process of the program.

licence_plates: In this folder the program save for each car image the chars of its license plate (image 1.jpg -> image plate1.jpg).

model: Neural network model for chars classification, it contains neural_network.json: structure of the network and neural_network.h5: weights).

classify_chars.py: Python program that using the Neural Network model in the "model" folder, classify images of folder "licence_plates" and save in the file "licencePlates.txt" the number of the license plates (a string for each plate, in the same order in which the images are found in the dataset). 

licencePlates.txt: Contain number (string) of each license plate in the dataset.