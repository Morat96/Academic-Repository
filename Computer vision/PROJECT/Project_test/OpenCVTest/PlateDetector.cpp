#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "PlateDetector.h"

bool isWhite(cv::Mat img);

// constructor
PlateDetector::PlateDetector(cv::Mat samplePlate, std::vector<cv::Mat> inputDataset) {
    
    sample = samplePlate;
    for (int i = 0; i < inputDataset.size(); i++) dataSet.push_back(inputDataset[i]);
    
}

// this function using a sliding window approach find the best window that contain a licence plate.
// this is possible by computing orb features of either sample plate and the current window and keeping
// the window that have smallest mean distance of matching points
void PlateDetector::findPlateLocationInImage(int imgIndex, int winWidth, int winHeight, cv::Mat& plateLocation, int& finalX, int& finalY) {
    
    // create an ORB object for ORB features (use of ORG extractor is convenient for its speed)
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500);
    
    // compute keyPoints and descriptors of the sample plate image
    std::vector<cv::KeyPoint> objKeyPoints;
    cv::Mat objDescriptors;
    orb -> detectAndCompute(sample, cv::Mat(), objKeyPoints, objDescriptors, false);
    
    int x = 5, y = 5;
    float bestMean = std::numeric_limits<float>::max();
    
    cv::Mat img = dataSet[imgIndex].clone();
    std::vector<cv::KeyPoint> currentKeyPoints;
    cv::Mat currentDescriptors;
    
    //                             ------------ SLIDING WINDOW ------------
    // scan in x direction
    while (x + 220 < img.cols) {
        y = 5;
        // scan in y direction
        while (y + 70 < img.rows) {
            // create a window of the original image with a fixed dimension
            cv::Rect rect(x, y, winWidth, winHeight);
            cv::Mat currentImg(img, rect);
            // double the size of the window in order to extract better the features
            resize(currentImg, currentImg, cv::Size(currentImg.cols * 2, currentImg.rows * 2));
            
            // detect keypoints and descriptors of the image selected by the current window
            orb -> detectAndCompute(currentImg, cv::Mat(), currentKeyPoints, currentDescriptors, false);
            
            // create a matcher and a vector to store them
            cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
            std::vector<cv::DMatch> currentMatch;
            
            
            float mean = std::numeric_limits<float>::max();
            float sum = 0;
            
            // if the descriptors are less then 50 don't consider the currend image
            if (currentDescriptors.rows > 50) {
                // compute the matches
                matcher -> match(objDescriptors, currentDescriptors, currentMatch);
                // compute the mean distance between keypoints on the object image and the current image
                for (int j = 0; j < currentMatch.size(); j++) sum += currentMatch[j].distance;
                
                mean = sum / currentMatch.size();
            }
            
            // update the mean if the mean distance of the current image is better
            if (mean < bestMean) {
                // save the x and y position of the window computed
                finalX = x;
                finalY = y;
                bestMean = mean;
            }
            // translation of 15 pixel in y direction
            y += 15;
        }
        // translation of 20 pixel in x direction
        x += 20;
    }
    
    // create and save the final window
    cv::Rect finalRect(finalX, finalY, winWidth+50, winHeight);
    cv::Mat finalImg(img, finalRect);
    plateLocation = finalImg.clone();
    
}

// this function using Canny algorithm on a window computed with findPlateLocationInImage, find a rectangle that best
// approximate the shape of the original licence plate
void PlateDetector::findBoundingOfPlate(cv::Mat plateRegion, cv::Mat& finalPlate, int& finalX, int& finalY, int& err) {
    
    cv::Mat edges;
    
    // apply Canny algorithm to the window that contain the plate
    cv::Canny(plateRegion, edges, 220, 300, 3);
    
    // vector to store contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    
    // find contours
    findContours(edges, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // vectors to store the position of the plate respect to the window coordinates
    std::vector<int> posX;
    std::vector<int> posY;
    
    // vector to store the rectangles found
    std::vector<cv::Rect> rectangles;
    
    for (int i = 0; i < contours.size(); i++) {
        // draw a rectangle that contain the contour
        cv::Rect rect = boundingRect(contours[i]);
        
        // keep only contour that have shape of a licence plate
        float dim = (float)rect.width / (float)rect.height;
        // if the rectangle is not too small and if width/height is between 2.7 and 5.5
        if ((rect.area() > 1000) && (dim <= 5.5) && (dim >= 2.7)) {
            rectangles.push_back(rect);
            posX.push_back(rect.x);
            posY.push_back(rect.y);
        }
    }
    
    // if no license plate has been found, report an error
    if(rectangles.size()==0) err = 1;
    else {
        int maxArea = std::numeric_limits<int>::max();
        // if more rectangle are found, keep the smallest one
        if (rectangles.size() > 1) {
            for (int k = 0; k < rectangles.size(); k++) {
                if (rectangles[k].area() < maxArea) {
                    maxArea = rectangles[k].area();
                    cv::Mat plate(plateRegion, rectangles[k]);
                    // save the plate
                    finalPlate = plate.clone();
                    // save the x and y position of the plate w.r.t window coordinates
                    finalX = posX[k];
                    finalY = posY[k];
                }
            }
        }
        else {
            cv::Mat plate(plateRegion, rectangles[0]);
            // save all if there is only one rectangle
            finalPlate = plate.clone();
            finalX = posX[0];
            finalY = posY[0];
        }
    }
}

// this function given the image of the licence plate, segment the plate and extract each digit of the plate
// and return them as a vector of images
void PlateDetector::findCharsInPlate(cv::Mat plate, std::vector<cv::Mat>& sorted_letters) {
    
    std::vector<cv::Mat> chars;
    
    // convert the image in grayscale and apply a threshold in order to work better with segmentation
    cvtColor(plate, plate, cv::COLOR_BGR2GRAY);
    threshold(plate, plate, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    // vector to store the contours
    std::vector<std::vector<cv::Point>> lettersContours;
    std::vector<cv::Vec4i> lettersHierarchy;
    // find the contours
    findContours(plate, lettersContours, lettersHierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // vector to store positions of chars in order to sort them later
    std::vector<double> position;
    
    for (int l = 0; l < lettersContours.size(); l++) {
        // draw a rectangle that contain the contour
        cv::Rect rect = boundingRect(lettersContours[l]);
        
        // keep only contour that have shape of a digit
        float dim = (float)rect.height / (float)rect.width;
        // if height/width is between 1.4 and 4.6 it's a letter
        if ((rect.area() > 50) && (dim > 1.4) && (dim < 4.6)) {
            cv::Mat currentChar(plate, rect);
            // check if the contour found is not a 'hole' of a digit (like 8 or B)
            if (!isWhite(currentChar)) {
                // save char and its position
                chars.push_back(currentChar);
                position.push_back(rect.x);
            }
        }
    }
    
    // sort digits based on them position
    std::vector<double> sorted;
    sorted = position;
    sort(sorted.begin(), sorted.end());
    
    for(int i=0;i<position.size();i++) {
        for(int j=0;j<position.size();j++) {
            if(sorted[i]==position[j]) sorted_letters.push_back(chars[j]);
        }
    }

}

// this function check if an image it can be a digit if it not contain a number of white pixel greater than
// 77% of its size. It is used because Canny find as a contour also 'hole' of a digit (like in digit 8)
bool isWhite(cv::Mat img) {
    
    float countw = 0;
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            if (img.at<uchar>(i, j) == 255) countw++;
        }
    }
    
    float numPixel = img.rows * img.cols;
    if (countw / numPixel > 0.77) return true;
    else return false;
}
