#include "tag_detection.hpp"

TagDetection::TagDetection():
    nh_(ros::this_node::getName()),
    it_(nh_)

{
    // ***********************
    // ROS Pub/Sub/Timers
    // ***********************

    nh_.param<std::string>("camera_image_topic", camera_topic_, "camera_image_topic_dummy");
    nh_.param<std::string>("detection_topic", detection_topic_, "detection_dummy");
    nh_.param<std::string>("tag_array_topic", tag_array_topic_, "tag_array_topic_dummy");
    int camera_ros_buffer_size;
    nh_.param<int>(        "camera_ros_buffer_size", camera_ros_buffer_size, 10);
    nh_.param<std::string>("tag_family", tag_family_name_, "dummy_fam");

    nh_.param<double>("fx", fx_, 42.0);
    nh_.param<double>("fy", fy_, 42.0);
    nh_.param<double>("cx", cx_, 42.0);
    nh_.param<double>("cy", cy_, 42.0);
    nh_.param<double>("tag_size", tag_size_, 42.0);

    nh_.param<double>("quad_decimate", quad_decimate_, 42.0);
    nh_.param<double>("quad_sigma", quad_sigma_, 42.0);
    nh_.param<int>("nthreads", nthreads_, 8);
    nh_.param<bool>("debug", debug_, false);
    nh_.param<bool>("refine_edges", refine_edges_, false);
    
    camera_sub_ = it_.subscribe(camera_topic_, camera_ros_buffer_size, &TagDetection::cameraCallback, this);
    tag_array_pub_ = nh_.advertise<apriltag_roswrap::TagArray>(tag_array_topic_, 10);
    detection_img_pub_ = it_.advertise(detection_topic_, 10);


    // configure apriltag detector
    if (tag_family_name_ == "tag36h11")
        tag_family_ = tag36h11_create();
    // else if (tag_family_name_ == "tag36h10")
    //     tag_family_ = tag36h10_create();
    // else if (tag_family_name_ == "tag36artoolkit")
    //     tag_family_ = tag36artoolkit_create();
    // else if (tag_family_name_ == "tag25h9")
    //     tag_family_ = tag25h9_create();
    // else if (tag_family_name_ == "tag25h7")
    //     tag_family_ = tag25h7_create();
    else {
        std::cout << tag_family_name_ << ": Unrecognized tag family name. Use e.g. \"tag36h11\".";
        exit(-1);
    }

    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, tag_family_);

    detector_->quad_decimate     = quad_decimate_;
    detector_->quad_sigma        = quad_sigma_;
    detector_->nthreads          = nthreads_;
    detector_->debug             = debug_;
    detector_->refine_edges      = refine_edges_;

}

TagDetection::~TagDetection(){
    apriltag_detector_destroy(detector_);
    if (tag_family_name_ == "tag36h11")
        tag36h11_destroy(tag_family_);
    // else if (tag_family_name_ == "tag36h10")
    //     tag_family_ = tag36h10_create();
    // else if (tag_family_name_ == "tag36artoolkit")
    //     tag_family_ = tag36artoolkit_create();
    // else if (tag_family_name_ == "tag25h9")
    //     tag_family_ = tag25h9_create();
    // else if (tag_family_name_ == "tag25h7")
    //     tag_family_ = tag25h7_create();
}


void TagDetection::cameraCallback(const sensor_msgs::ImageConstPtr& img_msg){
    std::cout << "size: " << img_msg->width << "x" << img_msg->height << std::endl;
    ros::Time ts1 = ros::Time::now();
    color_image_ = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::cvtColor(color_image_, grayscale_image_, cv::COLOR_BGR2GRAY);   

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {   .width  = grayscale_image_.cols,
                        .height = grayscale_image_.rows,
                        .stride = grayscale_image_.cols,
                        .buf    = grayscale_image_.data
                    };

    zarray_t *detections = apriltag_detector_detect(detector_, &im);
    apriltag_roswrap::TagArray tag_array_msg;

    for (int i = 0; i < zarray_size(detections); i++) {
        std::cout << "detection " << i << std::endl;
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // create a tag ros msg and fill it 
        apriltag_roswrap::Tag tag_msg; 
        tag_msg.id = det->id;
        std::vector<cv::Point2d> corner_vec;
        for (int ci=0; ci < 4; ci++){
            geometry_msgs::Point pt;
            pt.x = det->p[ci][0];
            pt.y = det->p[ci][1];
            tag_msg.corners.push_back(pt);

            corner_vec.push_back(cv::Point2d(det->p[ci][0], det->p[ci][1]));
        }
        tag_msg.is_pose_estimated = false;

        cv::line(color_image_, corner_vec[0], corner_vec[1], CV_RGB(255, 0, 0), 2);
        cv::line(color_image_, corner_vec[1], corner_vec[2], CV_RGB(0, 255, 0), 2);
        cv::line(color_image_, corner_vec[2], corner_vec[3], CV_RGB(0, 0, 255), 2);
        cv::line(color_image_, corner_vec[3], corner_vec[0], CV_RGB(255, 0, 255), 2);
 
        det_info_.det = det;
        det_info_.tagsize = tag_size_;
        det_info_.fx = fx_;
        det_info_.fy = fy_;
        det_info_.cx = cx_;
        det_info_.cy = cy_;

        // LINKING PROBLEM
        // Then call estimate_tag_pose.
        // apriltag_pose_t pose1, pose2;
        // double err1, err2;
        // int n_iter = 10;
        // estimate_tag_pose_orthogonal_iteration(&det_info_, &err1, &pose1, &err2, &pose2, n_iter);

        // apriltag_pose_t pose;
        // double err = estimate_tag_pose(&det_info_, &pose);



        // Do something with det here
        tag_array_msg.tag_array.push_back(tag_msg);
    }
    std::cout << "Total camera callback ts: " << (ros::Time::now() - ts1).toSec() * 1000 << " ms" << std::endl;
    
    tag_array_pub_.publish(tag_array_msg);

    cv_bridge::CvImage out_msg;
    out_msg.header   = img_msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = color_image_;
    detection_img_pub_.publish(out_msg.toImageMsg());
}
