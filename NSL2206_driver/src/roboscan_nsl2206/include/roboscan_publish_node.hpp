#pragma once

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <atomic> 
#include "nanolib.h"
#include <ros/package.h> 
#include <boost/thread/recursive_mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <roboscan_nsl2206/RoboscanNSL2206Config.h>

#define image_transfer_function

#ifdef image_transfer_function
#include <image_transport/image_transport.h>
#endif

namespace fs = std::experimental::filesystem;
namespace nanosys {
	struct ViewerParameter {
		int frameCount;
		int maxDistance;
		int pointCloudEdgeThreshold;
		int imageType;
		double lidarAngleV;
		double lidarAngleH;

		bool cvShow;
		bool grayScale;
		bool changedCvShow;
		bool changedImageType;
		bool changedIpInfo;
		bool reOpenLidar;
		bool saveParam;

		std::string frame_id;
		std::string	devName;
	};

	class roboscanPublisher { 

 	public:
		ros::NodeHandle nh_;
		roboscanPublisher();
		~roboscanPublisher();

		void initialise();
		void threadCallback();

		void setReconfigure();
		void publishFrame(NslPCD *frame);
		void startStreaming();


	private:
		std::string yaml_path_;
		boost::recursive_mutex param_mutex_;
		ros::Publisher imgDistancePub;
		ros::Publisher imgAmplPub;
		ros::Publisher pointcloudPub;
		
	#ifdef image_transfer_function
			image_transport::Publisher imagePublisher;
			std::unique_ptr<image_transport::ImageTransport> it_;
	#endif
			ViewerParameter viewerParam;

			std::unique_ptr<std::thread> publisherThread;
			std::atomic<bool> runThread;
			NslConfig 		nslConfig;
			int 			nsl_handle;

		const std::unordered_map<std::string, int> modeStrMap = {
			{"DISTANCE", 1},
			{"GRAYSCALE", 2},
			{"DISTANCE_AMPLITUDE", 3},
			{"DISTANCE_GRAYSCALE", 4},
			{"RGB", 5},
			{"RGB_DISTANCE", 6},
			{"RGB_DISTANCE_AMPLITUDE", 7},
			{"RGB_DISTANCE_GRAYSCALE", 8}
		};
		
	    const std::unordered_map<int, std::string> modeIntMap = {
	        {1, "DISTANCE"},
	        {2, "GRAYSCALE"},
			{3, "DISTANCE_AMPLITUDE"},
	        {4, "DISTANCE_GRAYSCALE"},
	        {5, "RGB"},
	        {6, "RGB_DISTANCE"},
	        {7, "RGB_DISTANCE_AMPLITUDE"},
	        {8, "RGB_DISTANCE_GRAYSCALE"}
	    };

		void load_params(roboscan_nsl2206::RoboscanNSL2206Config &cfg_out)
		{
			ROS_INFO("Loaded params: path=%s\n", yaml_path_.c_str());
			
			if (std::ifstream(yaml_path_))
			{
				YAML::Node config = YAML::LoadFile(yaml_path_);
				viewerParam.devName = config["devName"] ? config["devName"].as<std::string>() : "/dev/ttyNsl2206";
				viewerParam.frame_id = config["FrameID"] ? config["FrameID"].as<std::string>() : "roboscan_frame";
				viewerParam.maxDistance = config["MaxDistance"] ? config["MaxDistance"].as<int>() : 12500;
				viewerParam.pointCloudEdgeThreshold = config["PointColud EDGE"] ? config["PointColud EDGE"].as<int>() : 200;
				std::string tmpModeStr = config["ImageType"] ? config["ImageType"].as<std::string>() : "DISTANCE_AMPLITUDE";
				auto itMode = modeStrMap.find(tmpModeStr);
				viewerParam.imageType = (itMode != modeStrMap.end()) ? itMode->second : 3; // defeault DISTANCE_AMPLITUDE
				viewerParam.lidarAngleV = config["LidarAngleV"] ? config["LidarAngleV"].as<double>() : 0;
				viewerParam.lidarAngleH = config["LidarAngleH"] ? config["LidarAngleH"].as<double>() : 0;

				cfg_out.dev_name = viewerParam.devName;
				cfg_out.frame_id = viewerParam.frame_id;
				cfg_out.max_distance = viewerParam.maxDistance;
				cfg_out.pointcloud_edge = viewerParam.pointCloudEdgeThreshold;
				cfg_out.image_type = viewerParam.imageType;
				cfg_out.transform_angleV = viewerParam.lidarAngleV;
				cfg_out.transform_angleH = viewerParam.lidarAngleH;

				ROS_INFO("Loaded params: devName=%s, frame_id=%s, max = %d, edge = %d, imgType = %s, Angle = %.2f,%.2f\n", viewerParam.devName.c_str(), viewerParam.frame_id.c_str(), viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, modeIntMap.at(viewerParam.imageType).c_str(), viewerParam.lidarAngleV, viewerParam.lidarAngleH);
			}
			else{
				ROS_INFO("Not found params: devName=%s, frame_id=%s, max = %d, edge = %d, imgType = %s, Angle = %.2f,%.2f\n", viewerParam.devName.c_str(), viewerParam.frame_id.c_str(), viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, modeIntMap.at(viewerParam.imageType).c_str(), viewerParam.lidarAngleV, viewerParam.lidarAngleH);
			}
		}

		void save_params()
		{
	        std::ofstream fout(yaml_path_);
	        fout << "DevName: " << viewerParam.devName << "\n";
	        fout << "FrameID: " << viewerParam.frame_id << "\n";
	        fout << "MaxDistance: " << viewerParam.maxDistance << "\n";
	        fout << "PointColud EDGE: " << viewerParam.pointCloudEdgeThreshold << "\n";
			fout << "ImageType: " << modeIntMap.at(viewerParam.imageType)<< "\n";
	        fout << "LidarAngleV: " << viewerParam.lidarAngleV << "\n";
	        fout << "LidarAngleH: " << viewerParam.lidarAngleH << "\n";

	        fout.close();
	        ROS_INFO("Params saved to %s", yaml_path_.c_str());
		}

		void setMatrixColor(cv::Mat image, int x, int y, NslOption::NslVec3b color);
		void timeDelay(int milli);
		void renewParameter();
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		void paramDump(const std::string & filename);
		void paramLoad();
		cv::Mat addDistanceInfo(cv::Mat distMat, NslPCD *ptNslPCD, int lidarWidth, int lidarHeight);
		void setWinName();

		std::unique_ptr<dynamic_reconfigure::Server<roboscan_nsl2206::RoboscanNSL2206Config>> dr_server_;
		
		void dynamicReconfigureCallback(roboscan_nsl2206::RoboscanNSL2206Config &config, uint32_t level);

		int mouseXpos, mouseYpos;
		std::atomic<bool> reconfigure;
		char winName[100];
	};
} //end namespace nanosys
