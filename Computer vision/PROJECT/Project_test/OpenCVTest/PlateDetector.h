#include <opencv2/imgproc.hpp>
#include <vector>

class PlateDetector
{
public:
    
    // constructor
    PlateDetector(cv::Mat samplePlate, std::vector<cv::Mat> inputDataset);
    
    // find a window that contain the licence plate in the image
    void findPlateLocationInImage(int imgIndex, int winWidth, int winHeight, cv::Mat &plateLocation, int& finalX, int& finalY);
    
    // find the bounding of licence plate
    void findBoundingOfPlate(cv::Mat plateRegion, cv::Mat &finalPlate, int& finalX, int& finalY, int& err);
    
    // segment characters in the plate and save them in a vector
    void findCharsInPlate(cv::Mat plate, std::vector<cv::Mat> &chars);
    
private:
    
    // sample of licence plate
    cv::Mat sample;
    
    // car images
    std::vector<cv::Mat> dataSet;
};
    
    
