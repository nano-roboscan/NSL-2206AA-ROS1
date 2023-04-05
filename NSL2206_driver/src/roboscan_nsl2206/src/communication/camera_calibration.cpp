
#include "camera_calibration.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

#include <opencv2/core/utility.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;


CameraCalibration::CameraCalibration()
{
    calibZeroTangentDist = false;   // Assume zero tangential distortion
    calibFixPrincipalPoint = false; // Fix the principal point at the center
    fixK1 = false;                  // fix K1 distortion coefficient
    fixK2 = false;                  // fix K2 distortion coefficient
    fixK3 = false;                  // fix K3 distortion coefficient
    fixK4 = false;                  // fix K4 distortion coefficient
    fixK5 = false;                  // fix K5 distortion coefficient

    distCoeffs = Mat::zeros(8, 1, CV_64F);
    cameraMatrix = Mat::eye(3, 3, CV_64F);

   /* cameraMatrix.at<double>(0,0) = 181.29630252644728;
    cameraMatrix.at<double>(0,1) = 0.0;
    cameraMatrix.at<double>(0,2) = 80.0;

    cameraMatrix.at<double>(1,0) = 0.0;
    cameraMatrix.at<double>(1,1) = 181.07933981564878;
    cameraMatrix.at<double>(1,2) = 30.0;

    cameraMatrix.at<double>(2,0) = 0.0;
    cameraMatrix.at<double>(2,1) = 0.0;
    cameraMatrix.at<double>(2,2) = 1.0;

    distCoeffs.at<double>(0,0) = -0.39705898796411354;
    distCoeffs.at<double>(1,0) =  0.14337713547773262;
    distCoeffs.at<double>(2,0) =  0.0;
    distCoeffs.at<double>(3,0) =  0.0;
    distCoeffs.at<double>(4,0) =  0.0;
    distCoeffs.at<double>(5,0) =  0.0;
    distCoeffs.at<double>(6,0) =  0.0;
    distCoeffs.at<double>(7,0) =  0.0;*/

   /* cameraMatrix.at<double>(0,0) = 182.16445699317075;
    cameraMatrix.at<double>(0,1) = 0.0;
    //cameraMatrix.at<double>(0,2) = 66.64607330720742;
    cameraMatrix.at<double>(0,2) = 80.0;
    cameraMatrix.at<double>(1,0) = 0.0;
    cameraMatrix.at<double>(1,1) = 182.35111359599628;
    cameraMatrix.at<double>(1,2) = 30.372855460922676;
    cameraMatrix.at<double>(2,0) = 0.0;
    cameraMatrix.at<double>(2,1) = 0.0;
    cameraMatrix.at<double>(2,2) = 1.0;

    distCoeffs.at<double>(0,0) = -0.39588674655320016;
    distCoeffs.at<double>(1,0) =  0.18814797749599346;
    distCoeffs.at<double>(2,0) =  -0.001018333293759121;
    distCoeffs.at<double>(3,0) =  0.00037542620170120114;
    distCoeffs.at<double>(4,0) =  0.0;
    distCoeffs.at<double>(5,0) =  0.0;
    distCoeffs.at<double>(6,0) =  0.0;
    distCoeffs.at<double>(7,0) =  0.0;*/

    setFlags();
}


void CameraCalibration::addImage(cv::Mat &image){
    images.push_back(image);
}

void CameraCalibration::clearImages(){
    images.clear();
}

int CameraCalibration::getNumImages(){
    return images.size();
}

cv::Mat CameraCalibration::getCameraMatrix(){
    return cameraMatrix;
}

cv::Mat CameraCalibration::getDistortionCoeffs(){
    return distCoeffs;
}

void CameraCalibration::setCameraMatrix(cv::Mat cameraMatrix){
    this->cameraMatrix = cameraMatrix;
}

void CameraCalibration::setDistortionCoeffs(cv::Mat distCoeffs){
    this->distCoeffs = distCoeffs;
}


void CameraCalibration::createKnownBoardPosition(float boardSquareLength, vector<Point3f>& corners)
{
    for(int y=0; y< boardSize.height; y++)
        for(int x=0; x< boardSize.width; x++)
            corners.push_back(Point3f(x * boardSquareLength, y * boardSquareLength, 0.0f));
}

void CameraCalibration::getBoardCorners(vector<vector<Point2f>>& allFoundCorners, bool showResults)
{
    for(vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        vector<Point2f> pointBuf;
        bool found = findCirclesGrid(*iter, boardSize, pointBuf);

        if(found)  allFoundCorners.push_back(pointBuf);

        if(showResults) {
            Mat image = iter->clone();
            drawChessboardCorners(image, boardSize, pointBuf, found);
            imshow("Looking for Corners", image);
            waitKey(0);
        }

    } //end for

}

void CameraCalibration::setFlags(){

    flag = 0;
    if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
    if(fixK1)                  flag |= CALIB_FIX_K1;
    if(fixK2)                  flag |= CALIB_FIX_K2;
    if(fixK3)                  flag |= CALIB_FIX_K3;
    if(fixK4)                  flag |= CALIB_FIX_K4;
    if(fixK5)                  flag |= CALIB_FIX_K5;

    //aspectRatio = 328/252;

}

bool CameraCalibration::calibration(Size imageSize, Size boardSize, float boardSquareLength )
{

    if(images.size() < 1){
        return false;
    }

    this->imageSize = imageSize;
    this->boardSize = boardSize;

    vector<vector<Point2f>> boardImagePoints;
    getBoardCorners(boardImagePoints, true);

    vector<vector<Point3f>> boardPoints3D(1);
    createKnownBoardPosition( boardSquareLength, boardPoints3D[0]);
    boardPoints3D.resize(images.size(), boardPoints3D[0]);

    vector<Mat> rVectors, tVectors;

    distCoeffs = Mat::zeros(8, 1, CV_64F);
    cameraMatrix = Mat::eye(3, 3, CV_64F);

    //if(flag & CALIB_FIX_ASPECT_RATIO )
    //    cameraMatrix.at<double>(0,0) = aspectRatio;

    calibrateCamera(boardPoints3D, boardImagePoints, imageSize, cameraMatrix, distCoeffs, rVectors, tVectors, flag);

    return true;
}


bool CameraCalibration::loadCameraCalibration(std::string name){

    ifstream inStream(name);

    if(inStream)
    {
        string str;
        double value;
        int rows = 3;  //cameraMatrix.rows;
        int columns = 3; //cameraMatrix.cols;

        distCoeffs = Mat::zeros(8, 1, CV_64F);
        cameraMatrix = Mat::eye(3, 3, CV_64F);

        for(int r=0; r<rows; r++){
            for(int c=0; c<columns; c++){
                std::getline (inStream, str);
                value = std::stod(str);
                cameraMatrix.at<double>(r, c) = value;
            }
        }

        rows = 5; //distCoeffs.rows;
        columns =1; // distCoeffs.cols;

        for(int r=0; r<rows; r++){
            for(int c=0; c<columns; c++){
                std::getline (inStream, str);
                value = std::stod(str);
                distCoeffs.at<double>(r, c) = value;
            }
        }

        inStream.close();
        return true;
    }

    return false;
}


bool CameraCalibration::saveCameraCalibration(string name)
{
    ofstream outStream(name);
    if(outStream)
    {
        int rows = cameraMatrix.rows;
        int columns = cameraMatrix.cols;

        for(int r=0; r<rows; r++){
            for(int c=0; c<columns; c++){
                double value = cameraMatrix.at<double>(r, c);
                outStream << value << endl;
            }
        }

        rows = distCoeffs.rows;
        columns = distCoeffs.cols;

        for(int r=0; r<rows; r++){
            for(int c=0; c<columns; c++){
                double value = distCoeffs.at<double>(r, c);
                outStream << value << endl;
            }
        }

        outStream.close();
        return true;
    }

    return false;
}

void CameraCalibration::undistortion(Mat& inImage, Mat& outImage, Size imageSize){
    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), imageSize,  CV_32FC1,  map1, map2);
    remap(inImage, outImage, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
}


void CameraCalibration::undistortionPoints(int width, int height, vector<cv::Point2f> &outPoints_){

    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::noArray(), cv::noArray(), cv::Size(width, height), CV_32FC1,  map1, map2);
    outPoints_.resize(width * height);

    int x,y,l;
    for(l=0, y=0; y<height; y++)
        for(x=0; x< width; x++, l++){
            outPoints_[l].x = map1.at<float>(y, x);
            outPoints_[l].y = map2.at<float>(y, x);
        }
}


void CameraCalibration::undistortionPoints(vector<cv::Point2f> &outPoints, int width, int height){

    int x,y,l;
    int numPoints = width * height;
    vector<cv::Point2f> inPoints;
    inPoints.resize(numPoints);
    outPoints.resize(numPoints);

    cv::Mat inPoint(1,  2, CV_32FC2);
    cv::Mat outPoint(1, 2, CV_32FC2);

    for(l=0, y=0; y<height; y++)
        for(x=0; x< width; x++, l++){

            inPoint.at<float>(0,0) = x;
            inPoint.at<float>(0,1) = y;

            cv::undistortPoints(inPoint, outPoint, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
            outPoints[l].x = outPoint.at<float>(0,0);
            outPoints[l].y = outPoint.at<float>(0,1);
        }
}
