//ZED Include
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <PointCloud.hpp>

using namespace sl;
using namespace std;

int frmCnt;

ros::NodeHandle nh; // ROS node handler
sensor_msgs::CameraInfo camInfo;                // Camera information message
sensor_msgs::Image imgMsg;                      // Camera image message
std_msgs::Header msgHeader;                     // Message header
image_transport::ImageTransport     imgTr(nh);  // ROS image transportation
image_transport::CameraPublisher    imgPub;     // ROS camera publisher

void initCamInfo(zed::StereoParameters* params , int width, int height);
void publishImg(const zed::Mat& img);

int main(int argc, char** argv)
{
    frmCnt = 0;

    ros::init(argc, argv, "zed_simple_camera_node");

    // >>>>> Camera initialization
    zed::Camera* zedCam;
    zedCam = new zed::Camera( zed::VGA, 15.0f ); // VGA resolution for max speed at 15 FPS

    zed::ERRCODE err = zedCam->init( zed::MODE::PERFORMANCE, 0, true ); // Performance mode, verbose

    if (err != zed::SUCCESS)
    {
        delete zedCam;

        ROS_ERROR_STREAM( "Failed to initialize ZED Camera: " << zed::errcode2str(err) );

        ros::shutdown();

        return EXIT_FAILURE;
    }

    zed::SENSING_MODE dm_type = zed::RAW; // Not fillind depth holes

    int reliabilityIdx = 96; // Confidence Threshold

    int width = zedCam->getImageSize().width;
    int height = zedCam->getImageSize().height;
    // <<<<< Camera initialization

    // >>>>> ROS messages
    initCamInfo( zedCam->getParameters(), width, height );

    imgPub = imgTr.advertiseCamera("stereo/left/image_rect", 1, false);
    // <<<<< ROS messages


    PointCloud *cloud = new PointCloud( width, height );
    zed::Mat depth, imLeft;

    while( ros::ok() )
    {
        // DisparityMap filtering
        zedCam->setConfidenceThreshold( reliabilityIdx );

        // Depth Map image
        depth = zedCam->retrieveMeasure( zed::MEASURE::DEPTH );

        // Left image
        imLeft = zedCam->retrieveImage( zed::SIDE::LEFT );
        publishImg( imLeft );

        // PointCloud
        cloud->fill( imLeft.data, (float*)depth.data, zedCam->getParameters() );

        frmCnt++;

        ros::spinOnce();
    }

    delete zedCam;
    delete cloud;

    ros::shutdown();

    return EXIT_SUCCESS;
}

void publishImg( const zed::Mat& img )
{
    msgHeader.stamp = ros::Time::now();
    msgHeader.seq = frmCnt;
    msgHeader.frame_id = "left";

//    uchar* ros_data_ptr = (uchar*)(&imgMsg.data[0]);
//    uchar* zed_data_ptr = (uchar*)(&img.data[0]);

//    ros_data_ptr = zed_data_ptr; // TODO verify
    int dataSize = imgMsg.step*imgMsg.height;
    imgMsg.data.assign( &img.data[0], &img.data[dataSize] );

    if( imgPub.getNumSubscribers()>0 )
        imgPub.publish( imgMsg, camInfo );
}

void initCamInfo( zed::StereoParameters* params, int width, int height )
{
    // >>>>> Camera info
    camInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    camInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    camInfo.D.resize(5);
    camInfo.D[0] = params->LeftCam.disto[0];
    camInfo.D[1] = params->LeftCam.disto[1];
    camInfo.D[2] = params->LeftCam.disto[2];
    camInfo.D[3] = params->LeftCam.disto[3];
    camInfo.D[4] = params->LeftCam.disto[4];
    camInfo.K.fill( 0.0 );
    camInfo.K[0] = params->LeftCam.fx;
    camInfo.K[2] = params->LeftCam.cx;
    camInfo.K[4] = params->LeftCam.fy;
    camInfo.K[5] = params->LeftCam.cy;
    camInfo.K[8] = 1.0;

    camInfo.R.fill( 0.0 );

    camInfo.P.fill( 0.0 );
    camInfo.P[0] = params->LeftCam.fx;
    camInfo.P[2] = params->LeftCam.cx;
    camInfo.P[5] = params->LeftCam.fy;
    camInfo.P[6] = params->LeftCam.cy;
    camInfo.P[10] = 1.0;

    camInfo.width = width;
    camInfo.height = height;
    // <<<<< Camera info

    // >>>>> Image message
    imgMsg.width = width;
    imgMsg.height = height;
    imgMsg.encoding = sensor_msgs::image_encodings::BGRA8;  // 8U 4C
    imgMsg.step = width * sizeof(uint8_t) * 4;
    int dataSize = height * imgMsg.step;
    imgMsg.data.resize( dataSize );
    // <<<<< Image message
}
