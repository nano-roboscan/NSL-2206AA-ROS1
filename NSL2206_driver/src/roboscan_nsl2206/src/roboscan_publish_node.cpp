#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/Int32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <roboscan_nsl2206/roboscan_nsl2206Config.h>
#include <image_transport/image_transport.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "communication_2206.h"
#include "camera2206_driver.h"
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>
#include "epc_timer.h"
#include "roboscan_nsl2206_image.h"

using namespace ComLib;
using namespace std;

image_transport::Publisher imagePublisher;
ros::Publisher imagePublisher1;
ros::Publisher imagePublisher2;
ros::Publisher imageHeaderPublisher;
ros::Publisher cameraInfoPublisher;
ros::Publisher pointCloud2Publisher;
ros::ServiceServer cameraInfoService;
sensor_msgs::CameraInfo cameraInfo;
Settings settings;

//===================================================================

void updateConfig(roboscan_nsl2206::roboscan_nsl2206Config &config, uint32_t level)
{
    settings.runVideo = false;

    switch(config.image_type){
    case 0: settings.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_GRAYSCALE;
        break;
    case 1: settings.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE;
       break;
    case 2: settings.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_AMPLITUDE;
       break;
    case 3: settings.iType = Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_GRAYSCALE;
       break;
    default: break;
    }

    settings.mode = config.mode;
    settings.integrationTimeATOF1  = config.integration_time_0;
    settings.integrationTimeATOF2  = config.integration_time_1;
    settings.integrationTimeATOF3  = config.integration_time_2;
    settings.integrationTimeATOF4  = config.integration_time_3;
    settings.integrationTimeBTOF1  = config.integration_time_4;
    settings.integrationTimeBTOF2  = config.integration_time_5;
    settings.integrationTimeGray   = config.integration_time_gray;
    settings.modFrequency = config.mod_frequency;
    settings.kalmanThreshold = config.temporal_filter_threshold;
    settings.kalmanFactor  = config.temporal_filter_factor;
    settings.averageFilter = config.spatial_average_filter;
    settings.minAmplitude1 = config.min_amplitude_0;
    settings.minAmplitude2 = config.min_amplitude_1;
    settings.minAmplitude3 = config.min_amplitude_2;
    settings.minAmplitude4 = config.min_amplitude_3;
    settings.minAmplitude5 = config.min_amplitude_4;
    settings.offsetDistance = config.offset_distance;
    settings.angle = config.lens_angle;
    settings.enableCartesian = config.enable_cartesian;    
    settings.enableUndistortion = config.enable_undistortion;
    settings.enableImages = config.enable_images;
    settings.enablePointCloud = config.enable_point_cloud;
    settings.enableImageHeader = config.enable_image_header;
    settings.roi_leftX = config.roi_left_x;
    settings.roi_topY = config.roi_top_y;
    settings.roi_rightX = config.roi_right_x;
    settings.roi_bottomY = config.roi_bottom_y;
	
    settings.roi_rightX  -= (settings.roi_rightX - settings.roi_leftX + 1) % 4;
    settings.roi_bottomY -= (settings.roi_bottomY - settings.roi_topY + 1) % 4;

    config.roi_right_x  = settings.roi_rightX;
    config.roi_bottom_y = settings.roi_bottomY;

    if(config.start_stream)
        settings.runVideo = true;

    settings.triggerSingleShot = config.trigger_single_shot;
    settings.updateParam = true;

}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req, sensor_msgs::SetCameraInfo::Response& res)
{
    req.camera_info.width  = cameraInfo.width;
    req.camera_info.height = cameraInfo.height;
    req.camera_info.roi    = cameraInfo.roi;
    req.camera_info.K      = cameraInfo.K;
    req.camera_info.D      = cameraInfo.D;

    cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    return true;
}

void initialise()
{
    ros::NodeHandle nh("~");

    /*nh.getParam("startStream",          settings.startStream);
    nh.getParam("triggerSingleShot",    settings.triggerSingleShot);
    nh.getParam("integrationTimeATOF1", settings.integrationTimeATOF1);
    nh.getParam("integrationTimeATOF2", settings.integrationTimeATOF2);
    nh.getParam("integrationTimeATOF3", settings.integrationTimeATOF3);
    nh.getParam("integrationTimeATOF4", settings.integrationTimeATOF4);
    nh.getParam("integrationTimeBTOF1", settings.integrationTimeBTOF1);
    nh.getParam("integrationTimeTOF",   settings.integrationTimeGray);
    nh.getParam("kalmanFactor",         settings.kalmanFactor);
    nh.getParam("kalmanThreshold",      settings.kalmanThreshold);
    nh.getParam("averageFilter",        settings.averageFilter);
    nh.getParam("minAmplitude",         settings.minAmplitude1);
    nh.getParam("offsetDistance",       settings.offsetDistance);
    nh.getParam("roi_leftX",            settings.roi_leftX);
    nh.getParam("roi_topY",             settings.roi_topY);
    nh.getParam("roi_rightX",           settings.roi_rightX);
    nh.getParam("roi_bottomY",          settings.roi_bottomY);*/

    //advertise publishers
	
    imagePublisher1 = nh.advertise<sensor_msgs::Image>("image_raw1", 1000);
    imagePublisher2 = nh.advertise<sensor_msgs::Image>("image_raw2", 1000);    
    pointCloud2Publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZI> > ("points", 100);
    cameraInfoPublisher  = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);
    imageHeaderPublisher = nh.advertise<std_msgs::Int32MultiArray>("image_header", 1000);

	image_transport::ImageTransport it_(nh);
	imagePublisher = it_.advertise("image_distance", 1000);
	
    //advertise services
    cameraInfoService = nh.advertiseService("set_camera_info", setCameraInfo);

    settings.runVideo = false;
    settings.updateParam = false;
    ROS_INFO("Camera driver version 1.0");
    ROS_INFO("Camera node started");
}

//======================================================================

int main(int argc, char **argv)
{
    //if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) //use for debuging
    //    ros::console::notifyLoggerLevelsChanged();

    ros::init(argc, argv, "roboscan_publish_node");

    dynamic_reconfigure::Server<roboscan_nsl2206::roboscan_nsl2206Config> server;
    dynamic_reconfigure::Server<roboscan_nsl2206::roboscan_nsl2206Config>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);
    server.setCallback(f);

    initialise();
	printf("initialise ok\n");
    Nsl2206Driver cameraDriver(imagePublisher, imagePublisher1, imagePublisher2, imageHeaderPublisher, pointCloud2Publisher, cameraInfo, settings);

    while(ros::ok()){
        cameraDriver.update();
        ros::spinOnce();
    }

    
    imagePublisher1.shutdown();
    imagePublisher2.shutdown();
    pointCloud2Publisher.shutdown();
    cameraInfoPublisher.shutdown();
    cameraInfoService.shutdown();
    imageHeaderPublisher.shutdown();
    ROS_INFO("ROS Finish");
}














