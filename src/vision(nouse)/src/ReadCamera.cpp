#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "robo_vision/RMVideoCapture.hpp"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_vision_readcam");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    string image_topic = "image";
    string dev_name = "/dev/video0";

    //Define defalut parameter
    int image_width = 1920, image_height = 1080, exposure_time = 70, stream_rate = 30;
    float fx = 0, fy = 0, cx = 0, cy = 0, k1 = 0, k2 = 0, k3 = 0, t1 = 0, t2 = 0;
    int calibration = 0;

    private_nh.getParam("image_topic", image_topic);
    private_nh.getParam("dev_name", dev_name);

    private_nh.getParam("image_width", image_width);
    private_nh.getParam("image_height", image_height);
    private_nh.getParam("exposure_time", exposure_time);
    private_nh.getParam("stream_rate", stream_rate);
    private_nh.getParam("calibration", calibration);

    private_nh.getParam("fx", fx);//Intrinsic matrix
    private_nh.getParam("fy", fy);
    private_nh.getParam("cx", cx);
    private_nh.getParam("cy", cy);
    private_nh.getParam("k1", k1); //Radial Distortion
    private_nh.getParam("k2", k2);
    private_nh.getParam("k3", k3);
    private_nh.getParam("t1", t1);//Tangential Distortion
    private_nh.getParam("t2", t2);


    //display setting
    cout << image_topic << endl;
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub_image = it.advertiseCamera(image_topic, 1);
    image_transport::CameraPublisher pub_image0 = it.advertiseCamera("cam0/image_raw", 1);
    image_transport::CameraPublisher pub_image1 = it.advertiseCamera("cam1/image_raw", 1);      

    
    RMVideoCapture cap(dev_name.c_str(), 3);
    cap.setVideoFormat(image_width, image_height, 1);
    cap.info();
    cap.getCurrentSetting();
    cap.setExposureTime(0, exposure_time); // settings->exposure_time);
    cap.startStream();

    ROS_INFO("[robo_vision_readcam -- %s ]Image Producer Start!", dev_name.c_str());
    cv::Mat img;

    ros::Rate rate(stream_rate); // steraming rate hz
    while (ros::ok())
    {
        ros::Time start = ros::Time::now();
        cap >> img;
        ros::Time end = ros::Time::now();
        ros::Duration dua = end - start;
        // ROS_INFO("TTT  %f",dua.toSec());
        cv_bridge::CvImage img_msg;
        img_msg.header.stamp = ros::Time::now();
        img_msg.header.frame_id = "image";
        img_msg.image = img;
        img_msg.encoding = sensor_msgs::image_encodings::BGR8;

        sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo());
        cam_info->header = img_msg.header;
        cam_info->height = img.rows;
        cam_info->width = img.cols;

        cam_info->K[0] = fx;
        cam_info->K[2] = cx;
        cam_info->K[4] = fy;
        cam_info->K[5] = cy;
        cam_info->K[8] = 1;


        cam_info->distortion_model = "plumb_bob";
		cam_info->D.push_back(k1);
		cam_info->D.push_back(k2);
		cam_info->D.push_back(t1);
		cam_info->D.push_back(t2);
        cam_info->D.push_back(k3);

        cam_info->R[0] = 1.000000;
		cam_info->R[1] = 0.000000;
		cam_info->R[2] = 0.000000;
		cam_info->R[3] = 0.000000;
		cam_info->R[4] = 1.000000;
		cam_info->R[5] = 0.000000;
		cam_info->R[6] = 0.000000;
		cam_info->R[7] = 0.000000;
        cam_info->R[8] = 1.000000;

    	cam_info->P[0] = fx;
		cam_info->P[1] = 0.000000;
		cam_info->P[2] = cx;
		cam_info->P[3] = 0.000000;
		cam_info->P[4] = 0.000000;
		cam_info->P[5] = fy;
		cam_info->P[6] = cy;
		cam_info->P[7] = 0.000000;
		cam_info->P[8] = 0.000000;
		cam_info->P[9] = 0.000000;
		cam_info->P[10] = 1.000000;
        cam_info->P[11] = 0.000000;



        pub_image.publish(img_msg.toImageMsg(), cam_info);
        if(calibration)
        {
            pub_image0.publish(img_msg.toImageMsg(), cam_info);
            pub_image1.publish(img_msg.toImageMsg(), cam_info);            
        }

        rate.sleep();
    }
    cap.closeStream();
}
