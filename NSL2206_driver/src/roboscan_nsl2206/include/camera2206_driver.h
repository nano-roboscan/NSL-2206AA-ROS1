#ifndef CAMERA635_DRIVER_H
#define CAMERA635_DRIVER_H

#include "roboscan_nsl2206_image.h"
#include "communication_2206.h"
#include "camera_calibration.h"

#include <dynamic_reconfigure/server.h>
#include <roboscan_nsl2206/roboscan_nsl2206Config.h>

#include <cmath>
#include <unistd.h>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Int32MultiArray.h>

#include <image_transport/image_transport.h>

using std::vector;
using namespace cv;

struct Settings{

  bool startStream;
  bool triggerSingleShot;
  bool runVideo;
  bool updateParam;
  int  mode;
  int  integrationTimeATOF1;
  int  integrationTimeATOF2;
  int  integrationTimeATOF3;
  int  integrationTimeATOF4;
  int  integrationTimeBTOF1;
  int  integrationTimeBTOF2;
  int  integrationTimeGray;

  int modFrequency;
  double kalmanFactor;
  int kalmanThreshold;
  bool averageFilter;
  int minAmplitude1;
  int minAmplitude2;
  int minAmplitude3;
  int minAmplitude4;
  int minAmplitude5;
  int offsetDistance;  
  double angle;
  bool calibrate;
  bool enableCartesian;
  bool enableUndistortion;
  bool enableImages;
  bool enablePointCloud;
  bool enableImageHeader;

  int roi_leftX;
  int roi_topY;
  int roi_rightX;
  int roi_bottomY;

  double maxDistance;
  double maxAmplitude = 2890;

  
  	
  ComLib::Nsl2206Image::Nsl2206ImageType_e iType;

};



class Nsl2206Driver
{
public:
    Nsl2206Driver(image_transport::Publisher &imagePublisher_, const ros::Publisher &imagePublisher1_, const ros::Publisher &imagePublisher2_, const ros::Publisher &imageHeaderPublisher_,
                    const ros::Publisher &pointCloud2Publisher_, sensor_msgs::CameraInfo &cameraInfo_, Settings &set_);

    ~Nsl2206Driver();

    void update();    
    void initCommunication();
    void setAngle(double angle_);
    void updateCameraCalibration();

private:

    double koefX[60][160];
    double koefY[60][160];
    double koefZ[60][160];

    double angle;
    bool lastSingleShot;
    unsigned int frameSeq;
    static Settings *gSettings;


	//lens_transform
	int distortionTableSize = 46;
	int numCols;
	int numRows;
	double sensorPointSizeMM = 0.02f;
	double lens_angleH;
	double lens_angle[101];
	double lens_rp[101];
	double lens_xUA[160][60];
	double lens_yUA[160][60];
	double lens_zUA[160][60];

    const ros::Publisher &imagePublisher1;
    const ros::Publisher &imagePublisher2;
    const ros::Publisher &imageHeaderPublisher;
    const ros::Publisher &pointCloud2Publisher;

	image_transport::Publisher &imagePublisher;	

    int imageSize8;
    int imageSize16_1;
    int imageSize16_2;
    std::string strFrameID;
    sensor_msgs::Image img8;
    sensor_msgs::Image img16_1;
    sensor_msgs::Image img16_2;

    sensor_msgs::CameraInfo &cameraInfo;
    CameraCalibration cameraCalibration;
    ComLib::Communication2206 communication;
    std_msgs::Int32MultiArray imageHeaderMsg;

	vector<Vec3b> colorVector;

	void updateData();
    void setParameters();
    void updateLensCalibrationData(std::vector<uint8_t> data);
    void publishImageHeader(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateCameraInfo(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateGrayscaleFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateDistanceFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateDistanceAmplitudeFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void updateDistanceGrayscaleFrame(std::shared_ptr<ComLib::Nsl2206Image> image);
    void transformKoef(double angleGrad);
    void transformPixel(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width, double height, double angleGrad);
    void transformPixelOpt(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width2, double height2, double step);
    void transformPixelOpt1(int srcX, int srcY, double srcZ, double &destX, double &destY, double &destZ);
	void getDistanceColor(cv::Mat &imageLidar, int x, int y, int value);
	void getAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value);
	void getGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range);
	void createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
	double interpolate(double x, double x0, double y0, double x1, double y1);
	void initLensDistortionTable();
	double lensInterpolate(double x_in, double x0, double y0, double x1, double y1);
	double lensGetAngle(double x, double y, double sensorPointSizeMM);
	void lensTransformPixel(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle);
	void lensInitialisation(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY);
	
};



#endif // CAMERA635_DRIVER_H
