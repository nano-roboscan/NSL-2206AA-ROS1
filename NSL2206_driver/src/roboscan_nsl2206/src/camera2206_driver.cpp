
#include "camera2206_driver.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>


#include <iostream>
#include <fstream>
#include <sstream>


using namespace ComLib;
using namespace std;
using namespace cv;
using std::vector;

Settings *Nsl2206Driver::gSettings;

Nsl2206Driver::Nsl2206Driver(image_transport::Publisher &imagePublisher_, const ros::Publisher &imagePublisher1_, const ros::Publisher &imagePublisher2_, const ros::Publisher &imageHeaderPublisher_,
                                 const ros::Publisher &pointCloud2Publisher_, sensor_msgs::CameraInfo &cameraInfo_, Settings &set_):
imagePublisher(imagePublisher_),
imagePublisher1(imagePublisher1_),
imagePublisher2(imagePublisher2_),
imageHeaderPublisher(imageHeaderPublisher_),
pointCloud2Publisher(pointCloud2Publisher_),
cameraInfo(cameraInfo_)
{
    angle= 60.0;
    frameSeq = 0;
    imageSize8 = 0;
    imageSize16_1 = 0;
    imageSize16_2 = 0;
    gSettings = &set_;    
    gSettings->runVideo = false;
    gSettings->updateParam = false;
    lastSingleShot = false;
    strFrameID = "roboscan_frame";
	
    cameraInfo.D.resize(8);
    initCommunication();
	
    communication.sigReceivedGrayscale.connect(boost::bind(&Nsl2206Driver::updateGrayscaleFrame, this, _1));
    communication.sigReceivedDistance.connect(boost::bind(&Nsl2206Driver::updateDistanceFrame, this, _1));
    communication.sigReceivedDistanceAmplitude.connect(boost::bind(&Nsl2206Driver::updateDistanceAmplitudeFrame, this, _1));
    communication.sigReceivedDistanceGrayscale.connect(boost::bind(&Nsl2206Driver::updateDistanceGrayscaleFrame, this, _1));
    communication.sigReceivedLensCalibrationData.connect(boost::bind(&Nsl2206Driver::updateLensCalibrationData, this, _1));

    imageHeaderMsg.data.resize(42);

    communication.getLensCalibrationData();
}


Nsl2206Driver::~Nsl2206Driver()
{
    communication.close();
}

void Nsl2206Driver::setAngle(double angle_)
{
    angle = angle_;
    transformKoef(angle);
}

void Nsl2206Driver::update()
{
   if(gSettings->runVideo && !gSettings->updateParam){
        updateData(); //streaming
    }else if(gSettings->updateParam){
        setParameters(); //update parameters

        if(gSettings->triggerSingleShot && gSettings->triggerSingleShot != lastSingleShot)
            updateData(); //trigger single shot

        lastSingleShot = gSettings->triggerSingleShot;
    }
}


void Nsl2206Driver::setParameters()
{
    if(gSettings->updateParam)
    {
        gSettings->updateParam = false;
        ROS_INFO("SET PARAMETERS");

        if(gSettings->mode == 2) communication.setMode(4); //TODO... temporay
        else communication.setMode(gSettings->mode);

        communication.setIntegrationTime3d(0, gSettings->integrationTimeATOF1);
        communication.setIntegrationTime3d(1, gSettings->integrationTimeATOF2);
        communication.setIntegrationTime3d(2, gSettings->integrationTimeATOF3);
        communication.setIntegrationTime3d(3, gSettings->integrationTimeATOF4);
        communication.setIntegrationTime3d(4, gSettings->integrationTimeBTOF1);
        communication.setIntegrationTimeGrayscale(gSettings->integrationTimeGray);

        setAngle(gSettings->angle);

        if(gSettings->modFrequency == 0) communication.setModulationFrequency( ModulationFrequency_e::MODULATION_FREQUENCY_10MHZ);
        else communication.setModulationFrequency(ModulationFrequency_e::MODULATION_FREQUENCY_20MHz);

		gSettings->maxDistance = gSettings->modFrequency == 0 ? 15000.0f : gSettings->modFrequency == 1 ? 7500.0f : 50000.0f;

		communication.setMinimalAmplitude(0, gSettings->minAmplitude1);
        communication.setMinimalAmplitude(1, gSettings->minAmplitude2);
        communication.setMinimalAmplitude(2, gSettings->minAmplitude3);
        communication.setMinimalAmplitude(3, gSettings->minAmplitude4);
        communication.setMinimalAmplitude(4, gSettings->minAmplitude5);

        communication.setOffset(gSettings->offsetDistance);

        communication.setFilter(gSettings->kalmanThreshold, (uint)(gSettings->kalmanFactor*1000.0));
        communication.setRoi(gSettings->roi_leftX, gSettings->roi_topY, gSettings->roi_rightX, gSettings->roi_bottomY);
        communication.setDcsFilter(gSettings->averageFilter);

    }

}

void Nsl2206Driver::updateLensCalibrationData(std::vector<uint8_t> data)
{
    unsigned int numberOfElements = data.size() / sizeof(double);   
    int numD = numberOfElements - 9; //9 ->3x3 cameraMatrix
    cameraInfo.D.resize(numD);
    uint8_t *p = (uint8_t *)data.data();

    for(unsigned int i = 0; i < numD; i++)
    {
        cameraInfo.D.at(i) = *((double*)(&p[i * sizeof(double)]));
        //double value = cameraInfo.D.at(i);
        //ROS_INFO("Nsl2206Driver::updateLensCalibrationData D[%d]= %f", i, value);
    }

    for(unsigned int i = numD; i < numberOfElements; i++)
    {
        cameraInfo.K.at(i-numD) = *((double*)(&p[i * sizeof(double)]));
        //double value = cameraInfo.K.at(i-numD);
        //ROS_INFO("Nsl2206Driver::updateLensCalibrationData K[%d]= %f", i-numD, value);
    }

    updateCameraCalibration();
}

void Nsl2206Driver::updateData()
{
    switch(gSettings->iType){
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_GRAYSCALE:
              communication.getGrayscale();
              break;
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE:
              communication.getDistance();
              break;
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_AMPLITUDE:
              communication.getDistanceAmplitude();
              break;
        case Nsl2206Image::Nsl2206ImageType_e::NSL2206_IMAGE_DISTANCE_GRAYSCALE:
              communication.getDistanceGrayscale();
              break;
        default: break;
    }

}

void Nsl2206Driver::initCommunication(){

  communication.open(); //open serial port

  ROS_INFO("set mode 0");
  communication.setMode(0);

  ROS_INFO("ROI(%d, %d, %d, %d)", gSettings->roi_leftX, gSettings->roi_topY, gSettings->roi_rightX, gSettings->roi_bottomY);
  communication.setRoi(gSettings->roi_leftX, gSettings->roi_topY, gSettings->roi_rightX, gSettings->roi_bottomY);

  unsigned int minor, major;
  communication.getFirmwareRelease(major, minor);
  printf("Firmware release:  major= %d  minor= %d \n",major, minor);

  unsigned char red, green, blue;
  colorVector = vector<Vec3b>();
  for(int i = 0; i < CommunicationConstants::PixelNsl2206::NUM_COLORS; i++)
  {
  	createColorMap(CommunicationConstants::PixelNsl2206::NUM_COLORS, i, red, green, blue);	  
	colorVector.push_back(Vec3b(blue, green, red));	
  }

  //lenstransform
  lensInitialisation(sensorPointSizeMM, CommunicationConstants::PixelNsl2206::IMAGE_WIDTH, CommunicationConstants::PixelNsl2206::IMAGE_HEIGHT, 0, 0);
  
  
}


void Nsl2206Driver::publishImageHeader(std::shared_ptr<Nsl2206Image> image)
{

    if(gSettings->enableImageHeader)
    {
        imageHeaderMsg.data.at(0)  = image->getHeaderVersion();
        imageHeaderMsg.data.at(1)  = image->getFrameCounter();
        imageHeaderMsg.data.at(2)  = image->getTimestamp();
        imageHeaderMsg.data.at(3)  = image->getFirmwareVersion();
        imageHeaderMsg.data.at(4)  = image->getHardwareVersion();
        imageHeaderMsg.data.at(5)  = image->getChipID();
        imageHeaderMsg.data.at(6)  = image->getWidth();
        imageHeaderMsg.data.at(7)  = image->getHeight();
        imageHeaderMsg.data.at(8)  = image->getOriginX();
        imageHeaderMsg.data.at(9)  = image->getOriginY();
        imageHeaderMsg.data.at(10) = image->getCurrentIntegrationTime3DWF();
        imageHeaderMsg.data.at(11) = image->getCurrentIntegrationTime3DNF();
        imageHeaderMsg.data.at(12) = image->getCurrentIntegrationTimeGrayscale();
        imageHeaderMsg.data.at(13) = image->getIntegrationTimeGrayscale();
        imageHeaderMsg.data.at(14) = image->getIntegrationTime3d(0);
        imageHeaderMsg.data.at(15) = image->getIntegrationTime3d(1);
        imageHeaderMsg.data.at(16) = image->getIntegrationTime3d(2);
        imageHeaderMsg.data.at(17) = image->getIntegrationTime3d(3);
        imageHeaderMsg.data.at(18) = image->getIntegrationTime3d(4);
        imageHeaderMsg.data.at(19) = image->getIntegrationTime3d(5);
        imageHeaderMsg.data.at(20) = image->getIntegrationTime3d(6);
        imageHeaderMsg.data.at(21) = image->getIntegrationTime3d(7);
        imageHeaderMsg.data.at(22) = image->getAmplitudeLimit(0);
        imageHeaderMsg.data.at(23) = image->getAmplitudeLimit(1);
        imageHeaderMsg.data.at(24) = image->getAmplitudeLimit(2);
        imageHeaderMsg.data.at(25) = image->getAmplitudeLimit(3);
        imageHeaderMsg.data.at(26) = image->getAmplitudeLimit(4);
        imageHeaderMsg.data.at(27) = image->getOffset();
        imageHeaderMsg.data.at(28) = image->getBinningType();
        imageHeaderMsg.data.at(29) = image->getTemporalFilterDistance().getFactor();
        imageHeaderMsg.data.at(30) = image->getTemporalFilterDistance().getThreshold();

        imageHeaderMsg.data.at(31) = image->getTemporalFilterSingleValue().getFactor();
        imageHeaderMsg.data.at(32) = image->getTemporalFilterSingleValue().getThreshold();
        imageHeaderMsg.data.at(33) = image->getModulation().getFrequencyMhz();
        imageHeaderMsg.data.at(34) = image->getModulation().getChannel();
        imageHeaderMsg.data.at(35) = image->getHeaderFlags().getFlags();
        imageHeaderMsg.data.at(36) = image->getTemperature();
        imageHeaderMsg.data.at(37) = image->getBeamType();
        imageHeaderMsg.data.at(38) = image->getSingleValueDistance();
        imageHeaderMsg.data.at(39) = image->getSingleValueAmplitude();
        imageHeaderMsg.data.at(40) = image->getSingleValue(0);
        imageHeaderMsg.data.at(41) = image->getSingleValue(1);

        imageHeaderPublisher.publish(imageHeaderMsg);
    }

}


void Nsl2206Driver::updateGrayscaleFrame(std::shared_ptr<ComLib::Nsl2206Image> image){

    publishImageHeader(image);

    if(gSettings->enableImages)
    {
        img8.header.seq = frameSeq++;
        img8.header.stamp = ros::Time::now();
        img8.header.frame_id = strFrameID;
        img8.height = static_cast<uint32_t>(image->getHeight());
        img8.width = static_cast<uint32_t>(image->getWidth());
        img8.encoding = sensor_msgs::image_encodings::MONO8;
        img8.step = img8.width;
        img8.is_bigendian = 0;
        int numPix = img8.width * img8.height;

        if(imageSize8 != numPix){
            imageSize8 = numPix;
            img8.data.resize(numPix);
        }

        if(gSettings->enableUndistortion)
        {
            cv::Mat inImage;
            cv::Mat outImage;
            inImage.create(img8.height, img8.width, CV_8U);
            outImage.create(img8.height, img8.width, CV_8U);
            int x,y,l;

            for(l=0, y=0; y< img8.height; y++ )
                for(x=0; x< img8.width; x++, l++)
                    inImage.at<uint8_t>(y,x)= image->getGrayscaleOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img8.width,  img8.height));

            for(l=0, y=0; y< img8.height; y++ )
                for(x=0; x< img8.width; x++, l++)
                    img8.data[l] = outImage.at<uint8_t>(y,x);

        }else{

            for(int i=0; i< numPix; i++ )
                img8.data[i] = image->getGrayscaleOfPixel(i);
        }

        imagePublisher1.publish(img8);

    } //end if enableImages

}


void Nsl2206Driver::updateDistanceFrame(std::shared_ptr<ComLib::Nsl2206Image> image){

    int x, y, i, l;
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	
    publishImageHeader(image);
    updateCameraInfo(image);

	cv::Mat inImage;
    cv::Mat outImage;
	
    if(gSettings->enableImages)
    {
        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());        
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        int numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(numPix * 2);
        }

        uint16_t val = 0;

        if(gSettings->enableUndistortion){

           
            inImage.create(img16_1.height, img16_1.width, CV_16U);
            outImage.create(img16_1.height, img16_1.width, CV_16U);            

            for(l=0, y=0; y< img16_1.height; y++ )
                for(x=0; x< img16_1.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_1.width,  img16_1.height));

            for(i=0, y=0; y< img16_1.height; y++ )
                for(x=0; x< img16_1.width; x++, i+=2){
                    val = outImage.at<uint16_t>(y,x);
                    img16_1.data[i] =  val & 0xff;
                    img16_1.data[i+1] = (val>>8) & 0xff;
                }
        }else{

            for(i=0, l=0; l< numPix; l++, i+=2){
                val = image->getDistanceOfPixel(l);
                img16_1.data[i] =  val & 0xff;
                img16_1.data[i+1] = (val>>8) & 0xff;
            }
        }
		
	int p;
	for(p=0, y=0; y< image->getHeight(); y++)
	{
		for(x=0; x< image->getWidth(); x++, p++)
		{
			if(gSettings->enableUndistortion) val = outImage.at<uint16_t>(y,x);
           	else val = image->getDistanceOfPixel(p);
			
			getDistanceColor(imageLidar, x, y, val);
		}
	}
		
    imagePublisher1.publish(img16_1);
	cv_ptr->header.stamp = ros::Time::now();
	cv_ptr->header.frame_id = "roboscan_frame";
	cv_ptr->image = imageLidar;
	cv_ptr->encoding = "bgr8";
	imagePublisher.publish(cv_ptr->toImageMsg());
		
    }


    if(gSettings->enablePointCloud)
    {
        static int sz_pc = 0;
        const size_t nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        if(sz_pc != nPixel){
            cloud->points.resize(nPixel);
            sz_pc = nPixel;
      	}

      	int x,y,k;
      	double px, pz, py;

      	//double width2 =  image->getWidth()/2;
      	//double height2 = image->getHeight()/2;
      	//double alfa0 = (lens_angle * 3.14159265359) / 360.0;  // grad -> rad
      	//double step = alfa0 / width2;

      	uint16_t val;
      	cv::Mat inImage;
      	cv::Mat outImage;

      	if(gSettings->enableUndistortion){
        	inImage.create(cloud->height, cloud->width, CV_16U);
         	outImage.create(cloud->height, cloud->width, CV_16U);
          	int x,y,l;

          	for(l=0, y=0; y< cloud->height; y++ )
              	for(x=0; x< cloud->width; x++, l++)
                  	inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

          	cameraCalibration.undistortion(inImage, outImage, cv::Size(cloud->width,  cloud->height));
      	}

      	for(k=0, y=0; y < image->getHeight(); y++){
          	for(x=0; x < image->getWidth(); x++, k++){
              	pcl::PointXYZI &p = cloud->points[k];

              	if(gSettings->enableUndistortion) val = outImage.at<uint16_t>(y,x);
              	else val = image->getDistanceOfPixel(k);


				if(val > 0 && val < gSettings->maxDistance)
				{
					if(gSettings->enableCartesian){
                		//transformPixelOpt(x, y, val, px, py, pz, width2, height2, step);
                  		//transformPixelOpt1(x, y, val, px, py, pz);
						lensTransformPixel(x, y, val, px, py, pz, 0, 1);
						p.x = pz / 1000.0;
                  		p.y = px / 1000.0;
                  		p.z = -py / 1000.0;
                  		p.intensity = pz / 1000.0;
              		}else{
                  		p.x = val/1000.0;
                  		p.y = -(80-x) * 100.0 * 5;
                  		p.z = (30-y) * 100.0 * 5;
                  		p.intensity = val / 1000.0;
              		}
				}
				else{
					p.x = std::numeric_limits<float>::quiet_NaN();
					p.y = std::numeric_limits<float>::quiet_NaN();
					p.z = std::numeric_limits<float>::quiet_NaN();
				}

          } //ensd for x
      } //end for y

      pointCloud2Publisher.publish(cloud);

    } //end if enablePointCloud

}

void Nsl2206Driver::updateDistanceAmplitudeFrame(std::shared_ptr<Nsl2206Image> image)
{
    int x,y,i,k,l;
    uint16_t val;
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	
    publishImageHeader(image);

	if(gSettings->enableImages){
		cv::Mat inImage;
    	cv::Mat outImage;
        img16_1.header.seq = frameSeq++;
        img16_1.header.stamp = ros::Time::now();
        img16_1.header.frame_id = strFrameID;
        img16_1.height = static_cast<uint32_t>(image->getHeight());
        img16_1.width = static_cast<uint32_t>(image->getWidth());
        img16_1.encoding = sensor_msgs::image_encodings::MONO16;
        img16_1.step = img16_1.width * 2;
        img16_1.is_bigendian = 0;
        int numPix = img16_1.width * img16_1.height;

        if(imageSize16_1 != numPix){
            imageSize16_1 = numPix;
            img16_1.data.resize(numPix * 2);
        }


        if(gSettings->enableUndistortion)
        {
            inImage.create(img16_1.height, img16_1.width, CV_16U);
            outImage.create(img16_1.height, img16_1.width, CV_16U);
            int x,y,l;

            for(l=0, y=0; y< img16_1.height; y++ )
                for(x=0; x< img16_1.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getAmplitudeOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_1.width,  img16_1.height));

            for(i=0, y=0; y< img16_1.height; y++ )
                for(x=0; x< img16_1.width; x++, i+=2){
                    uint16_t val = outImage.at<uint16_t>(y,x);
                    img16_1.data[i] =  val & 0xff;
                    img16_1.data[i+1] = (val>>8) & 0xff;
                }

        }else{

            for(i=0, l=0; l< numPix; l++, i+=2 ){
                val = image->getAmplitudeOfPixel(l);
                img16_1.data[i] =  val & 0xff;
                img16_1.data[i+1] = (val>>8) & 0xff;
            }
        }
		
        imagePublisher1.publish(img16_1);


        img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = ros::Time::now();
        img16_2.header.frame_id = strFrameID;
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2;
        img16_2.is_bigendian = 0;
        numPix = img16_2.width * img16_2.height;

        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(numPix * 2);
        }

		

        if(gSettings->enableUndistortion)
        {
            cv::Mat inImage;
            cv::Mat outImage;
            inImage.create(img16_2.height, img16_2.width, CV_16U);
            outImage.create(img16_2.height, img16_2.width, CV_16U);
            int x,y,l;

            for(l=0, y=0; y< img16_2.height; y++ )
                for(x=0; x< img16_2.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_2.width,  img16_2.height));

            for(i=0, y=0; y< img16_2.height; y++ )
                for(x=0; x< img16_2.width; x++, i+=2){
                    val = outImage.at<uint16_t>(y,x);
                    img16_2.data[i] =  val & 0xff;
                    img16_2.data[i+1] = (val>>8) & 0xff;
                }
        }else{

            for(i=0, l=0; l< numPix; l++, i+=2 ){
                uint16_t val = image->getDistanceOfPixel(l);
                img16_2.data[i] =  val & 0xff;
                img16_2.data[i+1] = (val>>8) & 0xff;
            }
        }

        imagePublisher2.publish(img16_2);

		int p;
		for(p=0, y=0; y< image->getHeight(); y++)
		{
			for(x=0; x< image->getWidth(); x++, p++)
			{
				if(gSettings->enableUndistortion) val = outImage.at<uint16_t>(y,x);
           		else val = image->getDistanceOfPixel(p);

				getDistanceColor(imageLidar, x, y, val);
			}
		}	
		
		cv_ptr->header.stamp = ros::Time::now();
		cv_ptr->header.frame_id = "roboscan_frame";
		cv_ptr->image = imageLidar;
		cv_ptr->encoding = "bgr8";
		imagePublisher.publish(cv_ptr->toImageMsg());
		
    }


    if(gSettings->enablePointCloud)
    {
        const size_t nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        static int szAmp= 0;
        if(szAmp != nPixel){
            szAmp = nPixel;
            cloud->points.resize(nPixel);
        }

        double px, pz, py;
        uint16_t val, ampl;
        cv::Mat inImage, outImage;
        cv::Mat inAmplImage, outAmplImage;

        if(gSettings->enableUndistortion)
        {
            inImage.create(cloud->height, cloud->width, CV_16U);
            outImage.create(cloud->height, cloud->width, CV_16U);
            inAmplImage.create(cloud->height, cloud->width, CV_16U);
            outAmplImage.create(cloud->height, cloud->width, CV_16U);

            for(l=0, y=0; y< cloud->height; y++ )
                for(x=0; x< cloud->width; x++, l++){
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);
                    inAmplImage.at<uint16_t>(y,x)= image->getAmplitudeOfPixel(l);
                }

            cameraCalibration.undistortion(inImage, outImage, cv::Size(cloud->width,  cloud->height));
            cameraCalibration.undistortion(inAmplImage, outAmplImage, cv::Size(cloud->width,  cloud->height));
        }

        for(k=0, y=0; y < image->getHeight(); y++){
            for(x=0; x < image->getWidth(); x++, k++){
                pcl::PointXYZI &p = cloud->points[k];

                if(gSettings->enableUndistortion){
                    val = outImage.at<uint16_t>(y,x);
                    ampl=  outAmplImage.at<uint16_t>(y,x);
                } else {
                    val = image->getDistanceOfPixel(k);
                    ampl= image->getAmplitudeOfPixel(k);
                }

				if(val > 0 && val < gSettings->maxDistance)
				{
                	if(gSettings->enableCartesian){
                    	//transformPixelOpt1(x, y, val, px, py, pz);
						lensTransformPixel(x, y, val, px, py, pz, 0, 1);
                    	p.x = pz / 1000.0;
                    	p.y = px / 1000.0;
                   		p.z = -py / 1000.0;
                    	p.intensity = ampl / 1000.0;
                	}else{
                    	p.x = val / 1000.0;
                    	p.y = -(80-x) / 100.0 * 5;
                    	p.z = (30-y) /100.0 * 5;
                    	p.intensity = ampl / 1000.0;
                	}
				}
				else{
					p.x = std::numeric_limits<float>::quiet_NaN();
					p.y = std::numeric_limits<float>::quiet_NaN();
					p.z = std::numeric_limits<float>::quiet_NaN();
				}
            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);

    } //end if enable point cloud

}


void Nsl2206Driver::updateDistanceGrayscaleFrame(std::shared_ptr<Nsl2206Image> image)
{
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv::Mat imageLidar(image->getHeight(), image->getWidth(), CV_8UC3, Scalar(255, 255, 255));
	
    publishImageHeader(image);

    if(gSettings->enableImages)
    {
		cv::Mat inImage;
		cv::Mat outImage;
        img8.header.seq = frameSeq;
        img8.header.stamp = ros::Time::now();
        img8.header.frame_id = strFrameID; //std::to_string(0);
        img8.height = static_cast<uint32_t>(image->getHeight());
        img8.width = static_cast<uint32_t>(image->getWidth());
        img8.encoding = sensor_msgs::image_encodings::MONO8;
        img8.step = img8.width;
        img8.is_bigendian = 0;
        int numPix = img8.width * img8.height;
        int x,y,i,l;

        if(imageSize8 != numPix){
            imageSize8 = numPix;
            img8.data.resize(numPix);
        }

        if(gSettings->enableUndistortion)
        {

            inImage.create(img8.height, img8.width, CV_8U);
            outImage.create(img8.height, img8.width, CV_8U);

            for(l=0, y=0; y< img8.height; y++ )
                for(x=0; x< img8.width; x++, l++)
                    inImage.at<uint8_t>(y,x)= image->getGrayscaleOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img8.width,  img8.height));

            for(l=0, y=0; y< img8.height; y++ )
                for(x=0; x< img8.width; x++, l++)
                    img8.data[l] = outImage.at<uint8_t>(y,x);

        }else{

          for(l=0; l< numPix; l++)
              img8.data[l] = image->getGrayscaleOfPixel(l);
        }

        imagePublisher1.publish(img8);


        img16_2.header.seq = frameSeq++;
        img16_2.header.stamp = ros::Time::now();
        img16_2.header.frame_id = strFrameID;  //std::to_string(0);
        img16_2.height = static_cast<uint32_t>(image->getHeight());
        img16_2.width = static_cast<uint32_t>(image->getWidth());
        img16_2.encoding = sensor_msgs::image_encodings::MONO16;
        img16_2.step = img16_2.width * 2; //f->px_size;
        img16_2.is_bigendian = 0;
        numPix = img16_2.width * img16_2.height;
        uint16_t val;

        if(imageSize16_2 != numPix){
            imageSize16_2 = numPix;
            img16_2.data.resize(numPix * 2);
        }

        if(gSettings->enableUndistortion)
        {
            cv::Mat inImage;
            cv::Mat outImage;
            inImage.create(img16_2.height, img16_2.width, CV_16U);
            outImage.create(img16_2.height, img16_2.width, CV_16U);

            for(l=0, y=0; y< img16_2.height; y++ )
                for(x=0; x< img16_2.width; x++, l++)
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);

            cameraCalibration.undistortion(inImage, outImage, cv::Size(img16_2.width,  img16_2.height));

            for(i=0, y=0; y< img16_2.height; y++)
                for(x=0; x< img16_2.width; x++, i+=2){
                    val = outImage.at<uint16_t>(y,x);
                    img16_2.data[i] =  val & 0xff;
                    img16_2.data[i+1] = (val>>8) & 0xff;
                }
        }else{

            for(i=0, l=0; l< numPix; l++, i+=2){
                val = image->getDistanceOfPixel(l);
                img16_2.data[i] =  val & 0xff;
                img16_2.data[i+1] = (val>>8) & 0xff;
            }
        }

        imagePublisher2.publish(img16_2);

		int p;
		for(p=0, y=0; y< image->getHeight(); y++)
		{
			for(x=0; x< image->getWidth(); x++, p++)
			{
				if(gSettings->enableUndistortion) val = outImage.at<uint16_t>(y,x);
           		else val = image->getDistanceOfPixel(p);

				getDistanceColor(imageLidar, x, y, val);
			}
		}
		
		cv_ptr->header.stamp = ros::Time::now();
		cv_ptr->header.frame_id = "roboscan_frame";
		cv_ptr->image = imageLidar;
		cv_ptr->encoding = "bgr8";
		imagePublisher.publish(cv_ptr->toImageMsg());
		
				
    }

    if(gSettings->enablePointCloud)
    {
        const size_t nPixel = image->getWidth() * image->getHeight();
        static pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cloud->header.frame_id = strFrameID;
        cloud->header.stamp = pcl_conversions::toPCL(ros::Time::now());
        cloud->width = static_cast<uint32_t>(image->getWidth());
        cloud->height = static_cast<uint32_t>(image->getHeight());
        cloud->is_dense = false;

        static int szAmp= 0;
        if(szAmp != nPixel){
            szAmp = nPixel;
            cloud->points.resize(nPixel);
        }

        int x,y,k,l;
        double px, pz, py;

        uint16_t val;
        uint8_t gray;
        cv::Mat inImage, outImage;
        cv::Mat inAmplImage, outAmplImage;

        if(gSettings->enableUndistortion)
        {
            inImage.create(cloud->height, cloud->width, CV_16U);
            outImage.create(cloud->height, cloud->width, CV_16U);
            inAmplImage.create(cloud->height, cloud->width, CV_16U);
            outAmplImage.create(cloud->height, cloud->width, CV_16U);

            for(l=0, y=0; y< cloud->height; y++ )
                for(x=0; x< cloud->width; x++, l++){
                    inImage.at<uint16_t>(y,x)= image->getDistanceOfPixel(l);
                    inAmplImage.at<uint16_t>(y,x)= image->getAmplitudeOfPixel(l);
                }

            cameraCalibration.undistortion(inImage, outImage, cv::Size(cloud->width,  cloud->height));
            cameraCalibration.undistortion(inAmplImage, outAmplImage, cv::Size(cloud->width,  cloud->height));
        }

        for(k=0, y=0; y < image->getHeight(); y++){
            for(x=0; x < image->getWidth(); x++, k++){
                pcl::PointXYZI &p = cloud->points[k];

                if(gSettings->enableUndistortion){
                    val  = outImage.at<uint16_t>(y,x);
                    gray =  outAmplImage.at<uint16_t>(y,x);
                }else{
                    val  = image->getDistanceOfPixel(k);
                    gray = image->getGrayscaleOfPixel(k);
                }

                if(gSettings->enableCartesian){
                    //transformPixelOpt1(x, y, val, px, py, pz);
					lensTransformPixel(x, y, val, px, py, pz, 0, 1);

					p.x = px;
                    p.y = py;
                    p.z = pz;
                    p.intensity = gray;
                }else{
                    p.x = x * 5;
                    p.y = y * 5;
                    p.z = val;
                    p.intensity = gray;
                }

            } //ensd for x
        } //end for y

        pointCloud2Publisher.publish(cloud);
    }


}


void Nsl2206Driver::transformPixel(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width, double height, double angleGrad )
{
    double alfa0 = (angleGrad * 3.14159265359) / 360.0;  // grad -> rad
    double step = alfa0 / (width/2);
    double beta = (srcY - height/2) * step;
    double alfa = (srcX - width/2) * step;
    destX = srcZ * cos(beta) * sin(alfa) + width/2;
    destY = srcZ * sin(beta) + height/2;
    destZ = srcZ * cos(alfa) * cos(beta);
}


void Nsl2206Driver::transformPixelOpt(double srcX, double srcY, double srcZ, double &destX, double &destY, double &destZ, double width2, double height2, double step)
{
    double beta = (srcY - height2) * step;
    double alfa = (srcX -  width2) * step;
    double cos_beta = cos(beta);
    destX = srcZ * cos_beta * sin(alfa) + width2;
    destY = srcZ * sin(beta) + height2;
    destZ = srcZ * cos(alfa) * cos_beta;
}


void Nsl2206Driver::transformPixelOpt1(int srcX, int srcY, double srcZ, double &destX, double &destY, double &destZ)
{
    destX = srcZ * koefX[srcY][srcX] + 80;
    destY = srcZ * koefY[srcY][srcX] + 30;
    destZ = srcZ * koefZ[srcY][srcX];
}


void Nsl2206Driver::transformKoef(double angleGrad)
{
    double alfa0 = ((angleGrad/2) * 3.14159265359) / 360.0;  // grad -> rad
    double step = alfa0 / (160/2);

    for(int y=0; y<60; y++){
        double beta = (y - 60/2) * step;
        for(int x=0; x<160; x++){
            double alfa = (x - 160/2) * step;
            koefX[y][x] = cos(beta) * sin(alfa);
            koefY[y][x] = sin(beta);
            koefZ[y][x] = cos(alfa) * cos(beta);
        }
    }

}

void Nsl2206Driver::updateCameraInfo(std::shared_ptr<ComLib::Nsl2206Image> image){

    //cameraInfo.header = header;
    //The image dimensions with which the camera was calibrated.
    cameraInfo.width = image->getWidth();
    cameraInfo.height = image->getHeight();
    cameraInfo.roi.x_offset = 0;
    cameraInfo.roi.y_offset = 0;
    cameraInfo.roi.width  = image->getWidth();
    cameraInfo.roi.height = image->getHeight();
}

void Nsl2206Driver::updateCameraCalibration()
{
    cv::Mat cameraMatrix = cameraCalibration.getCameraMatrix();
    cv::Mat distCoeffs = cameraCalibration.getDistortionCoeffs();

    int rows, cols, l;
    for(l=0, rows = 0; rows<3; rows++)
        for(cols=0; cols<3; cols++, l++)
            cameraMatrix.at<double>(rows, cols) = cameraInfo.K.at(l);

    for(l=0; l< cameraInfo.D.size(); l++)
        distCoeffs.at<double>(l, 0) = cameraInfo.D.at(l);

    cameraCalibration.setCameraMatrix(cameraMatrix);
    cameraCalibration.setDistortionCoeffs(distCoeffs);
}

void Nsl2206Driver::getDistanceColor(cv::Mat &imageLidar, int x, int y, int value)
{	
	double indexDistFactorColor = CommunicationConstants::PixelNsl2206::NUM_COLORS / gSettings->maxDistance;
	if (value == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == CommunicationConstants::PixelNsl2206::VALUE_LOW_AMPLITUDE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == CommunicationConstants::PixelNsl2206::INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(colorVector.size()-1);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > gSettings->maxDistance)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = colorVector.size() - (value*indexDistFactorColor);
		if( index < 0 || index > colorVector.size() ){
			printf("error index = %d\n", index);
			index = 0;
		}
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}
}


void Nsl2206Driver::getAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value)
{	
	double indexAmplitudeFactorColor = CommunicationConstants::PixelNsl2206::NUM_COLORS / gSettings->maxAmplitude;
	if (value == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == CommunicationConstants::PixelNsl2206::INTERFERENCE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::EDGE_DETECTED)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > gSettings->maxAmplitude)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int idx = (int)(value * indexAmplitudeFactorColor-1);
//		printf("index = %d size = %lu\n", idx, colorVector.size());
		imageLidar.at<Vec3b>(y, x) = colorVector.at(idx);
	}
}


void Nsl2206Driver::getGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range)
{	
	if (value == CommunicationConstants::PixelNsl2206::VALUE_SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == CommunicationConstants::PixelNsl2206::VALUE_ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else
	{
		int color = value * (255/end_range);
		//printf("color index = %d\n", color);
		imageLidar.at<Vec3b>(y, x)[0] = color;
		imageLidar.at<Vec3b>(y, x)[1] = color;
		imageLidar.at<Vec3b>(y, x)[2] = color; 
	}
}



void Nsl2206Driver::createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    /*double B0 = -0.125;
    double B1 = B0 + 0.25;
    double B2 = B1 + 0.25;
    double B3 = B2 + 0.25;
    double G0 = B1;
    double G1 = G0 + 0.25;
    double G2 = G1 + 0.25;
    double G3 = G2 + 0.25;
    double R0 = B2;
    double R1 = R0 + 0.25;
    double R2 = R1 + 0.25;
    double R3 = R2 + 0.25;*/
    double k = 1;
    double BB0 = -0.125 * k - 0.25;
    double B1 = BB0 + 0.25 * k;
    double B2 = B1 + 0.25 * k;
    double B3 = B2 + 0.25 * k;
    double G0 = B1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;
    double R0 = B2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;
    double i = (double)indx/(double)numSteps - 0.25 * k;
    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }
    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }
    if( i>= BB0 && i < B1 ){
        blue = interpolate(i, BB0, 0, B1, 255);
    } else if((i >= B1)&&(i < B2)){
        blue = 255;
    } else if((i >= B2)&&(i < B3)) {
        blue = interpolate(i, B2, 255, B3, 0);
    } else{
        blue = 0;
    }
}


double Nsl2206Driver::interpolate(double x, double x0, double y0, double x1, double y1)
{  
	if( x1 == x0 )
    {
    	return y0;
  	}
  	else
  	{
    	return ((x-x0)*(y1-y0)/(x1-x0) + y0);
  	}
}


void Nsl2206Driver::initLensDistortionTable()
{
    if(gSettings->mode == 1) // 50
    {
        distortionTableSize = 46;

        lens_angle[	0	] = 	0	;
        lens_angle[	1	] = 	1.546388367	;
        lens_angle[	2	] = 	3.092776734	;
        lens_angle[	3	] = 	4.6391651	;
        lens_angle[	4	] = 	6.185553467	;
        lens_angle[	5	] = 	7.731941834	;
        lens_angle[	6	] = 	9.278330201	;
        lens_angle[	7	] = 	10.82471857	;
        lens_angle[	8	] = 	12.37110693	;
        lens_angle[	9	] = 	13.9174953	;
        lens_angle[	10	] = 	15.46388367	;
        lens_angle[	11	] = 	17.01027203	;
        lens_angle[	12	] = 	18.5566604	;
        lens_angle[	13	] = 	20.10304877	;
        lens_angle[	14	] = 	21.64943713	;
        lens_angle[	15	] = 	23.1958255	;
        lens_angle[	16	] = 	24.74221387	;
        lens_angle[	17	] = 	26.28860223	;
        lens_angle[	18	] = 	27.8349906	;
        lens_angle[	19	] = 	29.38137897	;
        lens_angle[	20	] = 	30.92776734	;
        lens_angle[	21	] = 	32.4741557	;
        lens_angle[	22	] = 	34.02054407	;
        lens_angle[	23	] = 	35.56693244	;
        lens_angle[	24	] = 	37.1133208	;
        lens_angle[	25	] = 	38.65970917	;
        lens_angle[	26	] = 	40.20609754	;
        lens_angle[	27	] = 	41.7524859	;
        lens_angle[	28	] = 	43.29887427	;
        lens_angle[	29	] = 	44.84526264	;
        lens_angle[	30	] = 	46.391651	;
        lens_angle[	31	] = 	47.93803937	;
        lens_angle[	32	] = 	49.48442774	;
        lens_angle[	33	] = 	51.0308161	;
        lens_angle[	34	] = 	52.57720447	;
        lens_angle[	35	] = 	54.12359284	;
        lens_angle[	36	] = 	55.6699812	;
        lens_angle[	37	] = 	57.21636957	;
        lens_angle[	38	] = 	58.76275794	;
        lens_angle[	39	] = 	60.3091463	;
        lens_angle[	40	] = 	61.85553467	;
        lens_angle[	41	] = 	63.40192304	;
        lens_angle[	42	] = 	64.9483114	;
        lens_angle[	43	] = 	66.49469977	;
        lens_angle[	44	] = 	68.04108814	;
        lens_angle[	45	] = 	69.5874765	;

        //size mm
        lens_rp[	0	] = 	0	;
        lens_rp[	1	] = 	0.1	;
        lens_rp[	2	] = 	0.2	;
        lens_rp[	3	] = 	0.3	;
        lens_rp[	4	] = 	0.4	;
        lens_rp[	5	] = 	0.5	;
        lens_rp[	6	] = 	0.6	;
        lens_rp[	7	] = 	0.7	;
        lens_rp[	8	] = 	0.8	;
        lens_rp[	9	] = 	0.9	;
        lens_rp[	10	] = 	1	;
        lens_rp[	11	] = 	1.1	;
        lens_rp[	12	] = 	1.2	;
        lens_rp[	13	] = 	1.3	;
        lens_rp[	14	] = 	1.4	;
        lens_rp[	15	] = 	1.5	;
        lens_rp[	16	] = 	1.6	;
        lens_rp[	17	] = 	1.7	;
        lens_rp[	18	] = 	1.8	;
        lens_rp[	19	] = 	1.9	;
        lens_rp[	20	] = 	2	;
        lens_rp[	21	] = 	2.1	;
        lens_rp[	22	] = 	2.2	;
        lens_rp[	23	] = 	2.3	;
        lens_rp[	24	] = 	2.4	;
        lens_rp[	25	] = 	2.5	;
        lens_rp[	26	] = 	2.6	;
        lens_rp[	27	] = 	2.7	;
        lens_rp[	28	] = 	2.8	;
        lens_rp[	29	] = 	2.9	;
        lens_rp[	30	] = 	3	;
        lens_rp[	31	] = 	3.1	;
        lens_rp[	32	] = 	3.2	;
        lens_rp[	33	] = 	3.3	;
        lens_rp[	34	] = 	3.4	;
        lens_rp[	35	] = 	3.5	;
        lens_rp[	36	] = 	3.6	;
        lens_rp[	37	] = 	3.7	;
        lens_rp[	38	] = 	3.8	;
        lens_rp[	39	] = 	3.9	;
        lens_rp[	40	] = 	4	;
        lens_rp[	41	] = 	4.1	;
        lens_rp[	42	] = 	4.2	;
        lens_rp[	43	] = 	4.3	;
        lens_rp[	44	] = 	4.4	;
        lens_rp[	45	] = 	4.5	;
    }
    else if(gSettings->mode == 0) //110
    {
        distortionTableSize = 46;

        lens_angle[	0	] = 	0	;
        lens_angle[	1	] = 	3.458333333	;
        lens_angle[	2	] = 	6.916666667	;
        lens_angle[	3	] = 	10.375	;
        lens_angle[	4	] = 	13.83333333	;
        lens_angle[	5	] = 	17.29166667	;
        lens_angle[	6	] = 	20.75	;
        lens_angle[	7	] = 	24.20833333	;
        lens_angle[	8	] = 	27.66666667	;
        lens_angle[	9	] = 	31.125	;
        lens_angle[	10	] = 	34.58333333	;
        lens_angle[	11	] = 	38.04166667	;
        lens_angle[	12	] = 	41.5	;
        lens_angle[	13	] = 	44.95833333	;
        lens_angle[	14	] = 	48.41666667	;
        lens_angle[	15	] = 	51.875	;
        lens_angle[	16	] = 	55.33333333	;
        lens_angle[	17	] = 	58.79166667	;
        lens_angle[	18	] = 	62.25	;
        lens_angle[	19	] = 	65.70833333	;
        lens_angle[	20	] = 	69.16666667	;
        lens_angle[	21	] = 	72.625	;
        lens_angle[	22	] = 	76.08333333	;
        lens_angle[	23	] = 	79.54166667	;
        lens_angle[	24	] = 	83	;
        lens_angle[	25	] = 	86.45833333	;
        lens_angle[	26	] = 	89.91666667	;
        lens_angle[	27	] = 	93.375	;
        lens_angle[	28	] = 	96.83333333	;
        lens_angle[	29	] = 	100.2916667	;
        lens_angle[	30	] = 	103.75	;
        lens_angle[	31	] = 	107.2083333	;
        lens_angle[	32	] = 	110.6666667	;
        lens_angle[	33	] = 	114.125	;
        lens_angle[	34	] = 	117.5833333	;
        lens_angle[	35	] = 	121.0416667	;
        lens_angle[	36	] = 	124.5	;
        lens_angle[	37	] = 	127.9583333	;
        lens_angle[	38	] = 	131.4166667	;
        lens_angle[	39	] = 	134.875	;
        lens_angle[	40	] = 	138.3333333	;
        lens_angle[	41	] = 	141.7916667	;
        lens_angle[	42	] = 	145.25	;
        lens_angle[	43	] = 	148.7083333	;
        lens_angle[	44	] = 	152.1666667	;
        lens_angle[	45	] = 	155.625	;



        //size mm
        lens_rp[	0	] = 	0	;
        lens_rp[	1	] = 	0.1	;
        lens_rp[	2	] = 	0.2	;
        lens_rp[	3	] = 	0.3	;
        lens_rp[	4	] = 	0.4	;
        lens_rp[	5	] = 	0.5	;
        lens_rp[	6	] = 	0.6	;
        lens_rp[	7	] = 	0.7	;
        lens_rp[	8	] = 	0.8	;
        lens_rp[	9	] = 	0.9	;
        lens_rp[	10	] = 	1	;
        lens_rp[	11	] = 	1.1	;
        lens_rp[	12	] = 	1.2	;
        lens_rp[	13	] = 	1.3	;
        lens_rp[	14	] = 	1.4	;
        lens_rp[	15	] = 	1.5	;
        lens_rp[	16	] = 	1.6	;
        lens_rp[	17	] = 	1.7	;
        lens_rp[	18	] = 	1.8	;
        lens_rp[	19	] = 	1.9	;
        lens_rp[	20	] = 	2	;
        lens_rp[	21	] = 	2.1	;
        lens_rp[	22	] = 	2.2	;
        lens_rp[	23	] = 	2.3	;
        lens_rp[	24	] = 	2.4	;
        lens_rp[	25	] = 	2.5	;
        lens_rp[	26	] = 	2.6	;
        lens_rp[	27	] = 	2.7	;
        lens_rp[	28	] = 	2.8	;
        lens_rp[	29	] = 	2.9	;
        lens_rp[	30	] = 	3	;
        lens_rp[	31	] = 	3.1	;
        lens_rp[	32	] = 	3.2	;
        lens_rp[	33	] = 	3.3	;
        lens_rp[	34	] = 	3.4	;
        lens_rp[	35	] = 	3.5	;
        lens_rp[	36	] = 	3.6	;
        lens_rp[	37	] = 	3.7	;
        lens_rp[	38	] = 	3.8	;
        lens_rp[	39	] = 	3.9	;
        lens_rp[	40	] = 	4	;
        lens_rp[	41	] = 	4.1	;
        lens_rp[	42	] = 	4.2	;
        lens_rp[	43	] = 	4.3	;
        lens_rp[	44	] = 	4.4	;
        lens_rp[	45	] = 	4.5	;
    }
    else
    {
        distortionTableSize = 46;

        lens_angle[	0	] = 	0	;
        lens_angle[	1	] = 	1.546388367	;
        lens_angle[	2	] = 	3.092776734	;
        lens_angle[	3	] = 	4.6391651	;
        lens_angle[	4	] = 	6.185553467	;
        lens_angle[	5	] = 	7.731941834	;
        lens_angle[	6	] = 	9.278330201	;
        lens_angle[	7	] = 	10.82471857	;
        lens_angle[	8	] = 	12.37110693	;
        lens_angle[	9	] = 	13.9174953	;
        lens_angle[	10	] = 	15.46388367	;
        lens_angle[	11	] = 	17.01027203	;
        lens_angle[	12	] = 	18.5566604	;
        lens_angle[	13	] = 	20.10304877	;
        lens_angle[	14	] = 	21.64943713	;
        lens_angle[	15	] = 	23.1958255	;
        lens_angle[	16	] = 	24.74221387	;
        lens_angle[	17	] = 	26.28860223	;
        lens_angle[	18	] = 	27.8349906	;
        lens_angle[	19	] = 	29.38137897	;
        lens_angle[	20	] = 	30.92776734	;
        lens_angle[	21	] = 	32.4741557	;
        lens_angle[	22	] = 	34.02054407	;
        lens_angle[	23	] = 	35.56693244	;
        lens_angle[	24	] = 	37.1133208	;
        lens_angle[	25	] = 	38.65970917	;
        lens_angle[	26	] = 	40.20609754	;
        lens_angle[	27	] = 	41.7524859	;
        lens_angle[	28	] = 	43.29887427	;
        lens_angle[	29	] = 	44.84526264	;
        lens_angle[	30	] = 	46.391651	;
        lens_angle[	31	] = 	47.93803937	;
        lens_angle[	32	] = 	49.48442774	;
        lens_angle[	33	] = 	51.0308161	;
        lens_angle[	34	] = 	52.57720447	;
        lens_angle[	35	] = 	54.12359284	;
        lens_angle[	36	] = 	55.6699812	;
        lens_angle[	37	] = 	57.21636957	;
        lens_angle[	38	] = 	58.76275794	;
        lens_angle[	39	] = 	60.3091463	;
        lens_angle[	40	] = 	61.85553467	;
        lens_angle[	41	] = 	63.40192304	;
        lens_angle[	42	] = 	64.9483114	;
        lens_angle[	43	] = 	66.49469977	;
        lens_angle[	44	] = 	68.04108814	;
        lens_angle[	45	] = 	69.5874765	;

        //size mm
        lens_rp[	0	] = 	0	;
        lens_rp[	1	] = 	0.1	;
        lens_rp[	2	] = 	0.2	;
        lens_rp[	3	] = 	0.3	;
        lens_rp[	4	] = 	0.4	;
        lens_rp[	5	] = 	0.5	;
        lens_rp[	6	] = 	0.6	;
        lens_rp[	7	] = 	0.7	;
        lens_rp[	8	] = 	0.8	;
        lens_rp[	9	] = 	0.9	;
        lens_rp[	10	] = 	1	;
        lens_rp[	11	] = 	1.1	;
        lens_rp[	12	] = 	1.2	;
        lens_rp[	13	] = 	1.3	;
        lens_rp[	14	] = 	1.4	;
        lens_rp[	15	] = 	1.5	;
        lens_rp[	16	] = 	1.6	;
        lens_rp[	17	] = 	1.7	;
        lens_rp[	18	] = 	1.8	;
        lens_rp[	19	] = 	1.9	;
        lens_rp[	20	] = 	2	;
        lens_rp[	21	] = 	2.1	;
        lens_rp[	22	] = 	2.2	;
        lens_rp[	23	] = 	2.3	;
        lens_rp[	24	] = 	2.4	;
        lens_rp[	25	] = 	2.5	;
        lens_rp[	26	] = 	2.6	;
        lens_rp[	27	] = 	2.7	;
        lens_rp[	28	] = 	2.8	;
        lens_rp[	29	] = 	2.9	;
        lens_rp[	30	] = 	3	;
        lens_rp[	31	] = 	3.1	;
        lens_rp[	32	] = 	3.2	;
        lens_rp[	33	] = 	3.3	;
        lens_rp[	34	] = 	3.4	;
        lens_rp[	35	] = 	3.5	;
        lens_rp[	36	] = 	3.6	;
        lens_rp[	37	] = 	3.7	;
        lens_rp[	38	] = 	3.8	;
        lens_rp[	39	] = 	3.9	;
        lens_rp[	40	] = 	4	;
        lens_rp[	41	] = 	4.1	;
        lens_rp[	42	] = 	4.2	;
        lens_rp[	43	] = 	4.3	;
        lens_rp[	44	] = 	4.4	;
        lens_rp[	45	] = 	4.5	;
    }
}


double Nsl2206Driver::lensInterpolate(double x_in, double x0, double y0, double x1, double y1){

    if(fabs(x1 - x0) < std::numeric_limits<double>::epsilon())  return y0;
    else return ((x_in-x0)*(y1-y0)/(x1-x0) + y0);
}

double Nsl2206Driver::lensGetAngle(double x, double y, double sensorPointSizeMM)
{
    double radius = sensorPointSizeMM * sqrt(x*x + y*y);
    double alfaGrad = 0;

    for(unsigned int i=1; i<distortionTableSize; i++)
    {
        if(radius >= lens_rp[i-1] && radius <= lens_rp[i]){

            alfaGrad = lensInterpolate(radius, lens_rp[i-1], lens_angle[i-1], lens_rp[i], lens_angle[i]);
        }
    }

    return alfaGrad;
}


void Nsl2206Driver::lensTransformPixel(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ, double sin_angle, double cos_angle)
{
    double y = srcZ * lens_yUA[srcX][srcY];
    double z = srcZ * lens_zUA[srcX][srcY];
    destX    = srcZ * lens_xUA[srcX][srcY];
    destY = z * sin_angle + y * cos_angle;
    destZ = z * cos_angle - y * sin_angle;
}


//void LensTransform::initialisation(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY, bool lensType, std::string fname, int fov)
void Nsl2206Driver::lensInitialisation(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY)
{
    int x, y, row, col;
    numCols = width;
    numRows = height;

    initLensDistortionTable();

    int r0 = 1 - numRows/2 + offsetY; //lens optical center offset
    int c0 = 1 - numCols/2 + offsetX;

    for(y=0, row = r0; y < numRows; row++, y++){
        for(x=0, col = c0; x < numCols; col++, x++){

            double c = col - 0.5;
            double r = row - 0.5;

            double angleGrad = lensGetAngle(c, r, sensorPointSizeMM);
            double angleRad =  angleGrad * 3.14159265 / 180.0;

            double lens_rp = sqrt(c * c + r * r);
            double rUA = sin(angleRad);

            lens_xUA[x][y] = c * rUA / lens_rp;
            lens_yUA[x][y] = r * rUA / lens_rp;
            lens_zUA[x][y] = cos(angleRad);
        }
    }

}


















