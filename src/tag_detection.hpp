#ifndef TAG_DETECTION_H
#define TAG_DETECTION_H


#include <iostream>
#include <fstream>
#include <stdlib.h>

/**************************
 *      ROS includes      *
 **************************/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include "apriltag_roswrap/Tag.h"
#include "apriltag_roswrap/TagArray.h"

#include <apriltag.h>
#include <apriltag_pose.h>

#include <tag25h9.h>
#include <tag36h11.h>

// TODO: define in CMakeList.txt
#define APRILTAGV 3

#if APRILTAGV == 2
    #include <tag36h10.h>
    #include <tag36artoolkit.h>
    #include <tag25h7.h>
#elif APRILTAGV == 3
    #include <tag16h5.h>
    #include <tagCircle21h7.h>
    #include <tagCircle49h12.h>
    #include <tagCustom48h12.h>
    #include <tagStandard41h12.h>
    #include <tagStandard52h13.h>
#endif


#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>



class TagDetection
{
    public:
        TagDetection();
        ~TagDetection();

        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        image_transport::Subscriber camera_sub_;
        image_transport::Publisher detection_img_pub_;
        ros::Publisher tag_array_pub_;


        std::string camera_topic_;
        std::string detection_topic_;
        std::string tag_array_topic_;
        std::string camera_intrinsics_path_;
        std::string tag_family_name_;

        apriltag_detector_t *detector_;
        apriltag_family_t *tag_family_;
        apriltag_detection_info_t det_info_;  // missing includes?

        double fx_;
        double fy_;
        double cx_;
        double cy_;
        double tag_size_;

        double quad_decimate_;
        double quad_sigma_;
        int nthreads_;
        bool debug_;
        bool refine_edges_;
        int tag_black_border_;

        cv::Mat color_image_;
        cv::Mat grayscale_image_;

        void cameraCallback(const sensor_msgs::ImageConstPtr& msg);


};

#endif