#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class CameraCalibration
{

public:
    explicit CameraCalibration();
    void clearImages();
    void addImage(cv::Mat &image);
    int  getNumImages();
    bool calibration(cv::Size imageSize, cv::Size boardSize, float boardSquareLength);
    bool saveCameraCalibration(std::string name);
    bool loadCameraCalibration(std::string name);
    void undistortion(cv::Mat &inImage, cv::Mat &outImage, cv::Size imageSize);
    void undistortionPoints(int width, int height, std::vector<cv::Point2f> &outPoints_);
    void undistortionPoints(std::vector<cv::Point2f> &outPoints, int width, int height);
    void setCameraMatrix(cv::Mat cameraMatrix);
    void setDistortionCoeffs(cv::Mat distCoeffs);

    cv::Mat getCameraMatrix();
    cv::Mat getDistortionCoeffs();



private:

    cv::Mat map1;
    cv::Mat map2;

    cv::Size boardSize;
    std::vector<cv::Mat> images;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Size imageSize;

    int flag;
    float aspectRatio;

    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool fixK1;                  // fix K1 distortion coefficient
    bool fixK2;                  // fix K2 distortion coefficient
    bool fixK3;                  // fix K3 distortion coefficient
    bool fixK4;                  // fix K4 distortion coefficient
    bool fixK5;                  // fix K5 distortion coefficient


    void setFlags();
    void createKnownBoardPosition(float boardSquareLength, std::vector<cv::Point3f>& corners);
    void getBoardCorners(std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResults=false);

};



#endif // CAMERA_CALIBRATION_H
