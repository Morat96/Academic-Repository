//  MORATELLO MATTEO
//  1205720
//  Project - Automatic Licence Plate Reading


// include openCV and standard headers
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "PlateDetector.h"

using namespace cv;
using namespace std;

// function that load images of the provided path
bool loadImages(string path, vector<Mat>& input_imgs) {
    
    vector<String> fn;
    
    try {
        glob(path, fn, false);
    } catch (cv::Exception) {
        cerr << "The path \"" << path << "\" is not correct, please check the path of the folder" << endl;
    }
    
    size_t count = fn.size(); //number of files in images folder
    // check if the foler is empty
    if (count==0)
    {
        cerr << "The images can't be loaded!" << endl;
        return false;
    }
    // load images
    for (size_t i=0; i<count; i++) {
        input_imgs.push_back(imread(fn[i]));
    }
    return true;
}

int main()
{
    
    vector<Mat> inputDataset;
    
    // load images of cars
    bool isFolderNotEmpty = loadImages("dataset/*.jpg", inputDataset);
    if(isFolderNotEmpty==false) return -1;
    
    // load a sample of licence plate used in the first phase of plate detection
    Mat samplePlate = imread("obj.png");
    if( samplePlate.empty() )
    {
        cout <<  "Could not open or find obj.png image" << endl ;
        return -1;
    }
    
    Ptr<PlateDetector> plateDetector = new PlateDetector(samplePlate, inputDataset);
    
    // ------------------------- PHASE I : Identication of the license plate location ------------------------
    
    vector<Mat> plateLocations;
    vector<int> xLoc;
    vector<int> yLoc;
    
    // for all images, find a window that cointain the licence plate
    for(int i=0; i<inputDataset.size(); i++) {
        Mat location;
        int x=0, y=0;
        // using sliding window approach and orb features find a window that contain the licence plate
        plateDetector -> findPlateLocationInImage(i, 220, 70, location, x, y);
        plateLocations.push_back(location);
        // save x and y position of the window
        xLoc.push_back(x);
        yLoc.push_back(y);
    }
    
    cout << "Plate locations of all images are found" << endl;
    
    // ------------------------- PHASE II : Extract the license plate from the window found ------------------
    
    vector<Mat> plateBoundings;
    vector<int> xBound;
    vector<int> yBound;
    
    // for all window found, find the bounding of the licence plate
    for(int i=0; i<plateLocations.size(); i++) {
        Mat bounding;
        int x=0, y=0, err=0;
        // using canny algorithm find the smallest bounding of the licence plate
        plateDetector -> findBoundingOfPlate(plateLocations[i], bounding, x, y, err);
        // if even one licence plate is not found, the program ends
        if(err==1) {
            cout << "The license plate of the image " << i+1 << ".jpg could not be found " << endl;
            return -1;
        }
        plateBoundings.push_back(bounding);
        
        // save x and y position of the plate w.r.t window
        xBound.push_back(x);
        yBound.push_back(y);
    }
    
    cout << "Plate boundings of all images are found" << endl;
    
    // ------------------------- PHASE III : Extract digits from licence plate image -------------------------
    
    vector<vector<Mat>> chars;
    
    // for all plates found, find all chars in the plate
    for(int i=0; i<plateBoundings.size(); i++) {
        vector<Mat> characters;
        // using canny algorithm find and segment all chars in the plate
        plateDetector -> findCharsInPlate(plateBoundings[i], characters);
        chars.push_back(characters);
    }
    
    cout << "Characters of all licence plates are found" << endl;
    
    // ------------------ Save all chars found, will be used as input for the Neural Network ------------------
    
    char buffer[50];
    
    // save all chars in a specific folder, will then be classified by the ML model
    for(int i=0;i<chars.size();i++) {
        sprintf(buffer, "licence_plates/plate%d.jpg", i+1);
        vector<Mat> plate;
        for(int j=0;j<chars[i].size();j++)
        {
            // resize the chars in 20x20 in order to use them with ML model
            resize(chars[i][j], chars[i][j], Size(20,20), INTER_CUBIC);
            // the ML model want in input a negative image
            chars[i][j] = ~ chars[i][j];
            plate.push_back(chars[i][j]);
        }
        Mat write_plate;
        // concatenate chars in a single image of size #ofchars x 20
        hconcat(plate, write_plate);
        imwrite(buffer, write_plate);
    }
    
    // --------------- Retrieve digits classified by neural network for printing them in output ---------------
    
    vector<string> licencePlates;
    ifstream inFile;
  
    // open the txt file that contain the number of plates (in the same order of images in dataset) 
    inFile.open("licencePlates.txt");
    if (!inFile) {
        cerr << "Unable to open file licencePlates.txt" << endl;
        exit(1);   // call system to stop
    }
    if (inFile.peek() == EOF )   // file is empty
    {
        cout << "The file licencePlates.txt is empty" << endl;
        return -1;
    }
    
    // save digits of the licence plates in a vector of strings
    string s;
    while (std::getline(inFile, s)) {
        licencePlates.push_back(s);
    }
    
    // ----------------------------------------------   OUTPUT -----------------------------------------------
    
    cout << endl << "Choose an image with a number from 1 to " << inputDataset.size() << " for detect its licence plate or any other number to exit" << endl;
    
    while(true) {
        
        // select a car for which to knows the license plate number
        int selCar;
        cin >> selCar;
        selCar -= 1;
        if(selCar>inputDataset.size()-1 || selCar<0) break;
        
        // draw in the car selected a bounding box of the plate found
        rectangle(inputDataset[selCar],Point(xLoc[selCar]+xBound[selCar], yLoc[selCar]+yBound[selCar]), Point(xLoc[selCar]+xBound[selCar]+plateBoundings[selCar].cols, yLoc[selCar]+yBound[selCar]+plateBoundings[selCar].rows), Scalar(0,255,0), 2, LINE_4, 0);
    
        // draw over the licence plate the number of licence plate found
        putText(inputDataset[selCar], licencePlates[selCar], Point(xLoc[selCar]+xBound[selCar], yLoc[selCar]+yBound[selCar]-3), FONT_HERSHEY_DUPLEX , 0.7, Scalar(0,255,0));
    
        // write also in output the number of licence plate
        cout << "The number of the selected licence plate is: " << licencePlates[selCar] << endl;
        
        imshow("Automatic Licence Plate Reading", inputDataset[selCar]);
        cv::waitKey(0);
        
        cout << endl << "Choose another image with a number from 1 to " << inputDataset.size() << " or any other number to exit" << endl;
    }
    
    cv::destroyAllWindows();
    return 0;
}
