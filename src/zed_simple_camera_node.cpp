//ZED Include
#include <zed/Mat.hpp>
#include <zed/Camera.hpp>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#ifndef DEG2RAD
#define DEG2RAD 0.017444444
#endif

#ifndef RAD2DEG
#define RAD2DEG 57.324842225
#endif


#include <cuda.h>
#include <cuda_runtime.h>

#define PAR_PUB_TF "publish_tf"

using namespace sl;
using namespace std;

int frmCnt;

sensor_msgs::CameraInfo rgbCamInfo;             // Camera information message for RGB data
sensor_msgs::Image rgbImgMsg;                   // Camera image message for RGB data
std_msgs::Header rgbMsgHeader;                  // Message header for RGB data
image_transport::ImageTransport*    rgbImgTr;   // ROS image transportation for RGB data
image_transport::CameraPublisher    rgbImgPub;  // ROS camera publisher for RGB data

sensor_msgs::CameraInfo depthCamInfo;             // Camera information message
sensor_msgs::Image depthImgMsg;                   // Camera image message for depth data
std_msgs::Header depthMsgHeader;                  // Message header for depth data
image_transport::ImageTransport*    depthImgTr;   // ROS image transportation for depth data
image_transport::CameraPublisher    depthImgPub;  // ROS camera publisher for depth data

void initRgbCamInfo(zed::StereoParameters* params , int width, int height);
void publishRgbImg(const unsigned char *image, int dataSize);

void initDepthCamInfo(zed::StereoParameters* params , int width, int height);
void publishDepthImg(const unsigned char *image, int dataSize);

bool publish_tf;

int main(int argc, char** argv)
{
    frmCnt = 0;

    ros::init(argc, argv, "zed_simple_camera_node");

    ros::NodeHandle nh; // ROS node handler

    // >>>>> ROS Params
    string prefix = ros::this_node::getName();

    ROS_INFO_STREAM( "Node: " << prefix );

    if( nh.hasParam( prefix+"/"+PAR_PUB_TF ) )
    {
        nh.getParam( prefix+"/"+PAR_PUB_TF, publish_tf );
    }
    else
    {
        publish_tf = true;
        nh.setParam( prefix+"/"+PAR_PUB_TF, publish_tf );
    }

    ROS_INFO_STREAM( "Publish TF: " << (publish_tf?"Enable":"Disable"));
    // <<<<< ROS Params

    // >>>>> Camera initialization
    ROS_INFO_STREAM( "Initializing ZED camera...");

    zed::Camera* zedCam;
    zedCam = new zed::Camera( zed::VGA, 15.0f ); // VGA resolution for max speed at 15 FPS

    //zed::ERRCODE err = zedCam->init( zed::MODE::PERFORMANCE, 0, true ); // Performance mode, verbose
    zed::ERRCODE err = zedCam->init( zed::MODE::QUALITY, 0, true ); // Quality mode, verbose

    if (err != zed::SUCCESS)
    {
        delete zedCam;

        ROS_ERROR_STREAM( "Failed to initialize ZED Camera: " << zed::errcode2str(err) );

        ros::shutdown();

        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM( "... ZED camera ready!");

    zed::SENSING_MODE dm_type = zed::RAW; // Not fillind depth holes

    int reliabilityIdx = 96; // Confidence Threshold

    int width = zedCam->getImageSize().width;
    int height = zedCam->getImageSize().height;
    // <<<<< Camera initialization

    // >>>>> ROS messages
    rgbImgTr = new image_transport::ImageTransport(nh);
    initRgbCamInfo( zedCam->getParameters(), width, height );
    rgbImgPub = rgbImgTr->advertiseCamera("stereo/left/image_rect", 1, false);

    depthImgTr = new image_transport::ImageTransport(nh);
    initDepthCamInfo( zedCam->getParameters(), width, height );
    depthImgPub = depthImgTr->advertiseCamera( "stereo/depth/image_rect", 1, false );
    // <<<<< ROS messages

    zed::Mat depth, imLeft;

    while( ros::ok() )
    {
        ros::Time startElab = ros::Time::now();

        // DisparityMap filtering
        zedCam->setConfidenceThreshold( reliabilityIdx );

        zedCam->grab( dm_type, true, true ); // Grab frame with disparity and depth

        // >>>>> Depth Map image
        depth = zedCam->retrieveMeasure( zed::MEASURE::DEPTH );
        publishDepthImg( depth.data, height * rgbImgMsg.step );
        // <<<<< Depth Map image

        // >>>>> Left image
        //imLeft = zedCam->retrieveImage_gpu( zed::SIDE::LEFT );
        imLeft = zedCam->retrieveImage( zed::SIDE::LEFT );
        publishRgbImg( imLeft.data, height * rgbImgMsg.step );
        // <<<<< Left image

        // >>>>> At the end of the loop
        ros::Duration elapsed = (ros::Time::now() - startElab);
        ROS_DEBUG_STREAM( "Frame: " << frmCnt << " - FPS: " << 1.0/elapsed.toSec() << " Hz" );

        frmCnt++;

        ros::spinOnce();
        // <<<<< At the end of the loop
    }

    delete zedCam;

    ros::shutdown();

    return EXIT_SUCCESS;
}

void publishRgbImg( const unsigned char* image, int dataSize )
{
    rgbMsgHeader.stamp = ros::Time::now();
    rgbMsgHeader.seq = frmCnt;
    rgbMsgHeader.frame_id = "left";

    uchar* ros_data_ptr = (uchar*)(&rgbImgMsg.data[0]);

    memcpy( ros_data_ptr, image, dataSize );
    //cudaMemcpy( ros_data_ptr, image, dataSize, cudaMemcpyDeviceToHost );

    if( rgbImgPub.getNumSubscribers()>0 )
    {
        rgbImgPub.publish( rgbImgMsg, rgbCamInfo );

        if( publish_tf )
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

            tf::Quaternion q;
            q.setRPY( 90.0*DEG2RAD, 0.0*DEG2RAD, 0.0*DEG2RAD);

            transform.setRotation(q);

            br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", "rgb_frame") );
        }
    }
}

void publishDepthImg( const unsigned char* image, int dataSize )
{
    depthMsgHeader.stamp = ros::Time::now();
    depthMsgHeader.seq = frmCnt;
    depthMsgHeader.frame_id = "depth";

    uchar* ros_data_ptr = (uchar*)(&depthImgMsg.data[0]);

    memcpy( ros_data_ptr, image, dataSize );
    //cudaMemcpy( ros_data_ptr, image, dataSize, cudaMemcpyDeviceToHost );

    if( depthImgPub.getNumSubscribers()>0 )
    {
        depthImgPub.publish( depthImgMsg, depthCamInfo );

        if( publish_tf )
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

            tf::Quaternion q;
            q.setRPY( 90.0*DEG2RAD, 0.0*DEG2RAD, 0.0*DEG2RAD);

            transform.setRotation(q);

            br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", "depth_frame") );
        }
    }
}

void initRgbCamInfo( zed::StereoParameters* params, int width, int height )
{
    // >>>>> Camera info
    rgbCamInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    rgbCamInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    rgbCamInfo.D.resize(5);
    rgbCamInfo.D[0] = params->LeftCam.disto[0];
    rgbCamInfo.D[1] = params->LeftCam.disto[1];
    rgbCamInfo.D[2] = params->LeftCam.disto[2];
    rgbCamInfo.D[3] = params->LeftCam.disto[3];
    rgbCamInfo.D[4] = params->LeftCam.disto[4];
    rgbCamInfo.K.fill( 0.0 );
    rgbCamInfo.K[0] = params->LeftCam.fx;
    rgbCamInfo.K[2] = params->LeftCam.cx;
    rgbCamInfo.K[4] = params->LeftCam.fy;
    rgbCamInfo.K[5] = params->LeftCam.cy;
    rgbCamInfo.K[8] = 1.0;

    rgbCamInfo.R.fill( 0.0 );

    rgbCamInfo.P.fill( 0.0 );
    rgbCamInfo.P[0] = params->LeftCam.fx;
    rgbCamInfo.P[2] = params->LeftCam.cx;
    rgbCamInfo.P[5] = params->LeftCam.fy;
    rgbCamInfo.P[6] = params->LeftCam.cy;
    rgbCamInfo.P[10] = 1.0;

    rgbCamInfo.width = width;
    rgbCamInfo.height = height;
    // <<<<< Camera info

    // >>>>> Image message
    rgbImgMsg.width = width;
    rgbImgMsg.height = height;
    rgbImgMsg.encoding = sensor_msgs::image_encodings::BGRA8;  // 8U 4C
    rgbImgMsg.step = width * sizeof(uint8_t) * 4;
    int dataSize = height * rgbImgMsg.step;
    rgbImgMsg.data.resize( dataSize );
    // <<<<< Image message
}

void initDepthCamInfo( zed::StereoParameters* params, int width, int height )
{
    // >>>>> Camera info
    depthCamInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    depthCamInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    depthCamInfo.D.resize(5);
    depthCamInfo.D[0] = 0.0;
    depthCamInfo.D[1] = 0.0;
    depthCamInfo.D[2] = 0.0;
    depthCamInfo.D[3] = 0.0;
    depthCamInfo.D[4] = 0.0;

    depthCamInfo.K.fill( 0.0 );
    depthCamInfo.K[0] = params->LeftCam.fx;
    depthCamInfo.K[2] = params->LeftCam.cx;
    depthCamInfo.K[4] = params->LeftCam.fy;
    depthCamInfo.K[5] = params->LeftCam.cy;
    depthCamInfo.K[8] = 1.0;

    depthCamInfo.R.fill( 0.0 );

    depthCamInfo.P.fill( 0.0 );
    depthCamInfo.P[0] = params->LeftCam.fx;
    depthCamInfo.P[2] = params->LeftCam.cx;
    depthCamInfo.P[5] = params->LeftCam.fy;
    depthCamInfo.P[6] = params->LeftCam.cy;
    depthCamInfo.P[10] = 1.0;

    depthCamInfo.width = width;
    depthCamInfo.height = height;
    // <<<<< Camera info

    // >>>>> Image message
    depthImgMsg.width = width;
    depthImgMsg.height = height;
    depthImgMsg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;  // 32F 1C
    depthImgMsg.step = width * sizeof(float) * 1;
    int dataSize = height * depthImgMsg.step;
    depthImgMsg.data.resize( dataSize );
    // <<<<< Image message
}
