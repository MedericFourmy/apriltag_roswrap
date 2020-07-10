#include "tag_detection.hpp"

TagDetection::TagDetection():
    nh_(ros::this_node::getName()),
    it_(nh_)

{
    // ***********************
    // ROS Pub/Sub/Timers
    // ***********************


    nh_.getParam("camera_image_topic", camera_topic_);
    nh_.getParam("detection_topic", detection_topic_);
    nh_.getParam("tag_array_topic", tag_array_topic_);
    int camera_ros_buffer_size;
    nh_.getParam(        "camera_ros_buffer_size", camera_ros_buffer_size);
    nh_.getParam("tag_family", tag_family_name_);

    nh_.getParam("fx", fx_);
    nh_.getParam("fy", fy_);
    nh_.getParam("cx", cx_);
    nh_.getParam("cy", cy_);
    nh_.getParam("tag_size", tag_size_);

    nh_.getParam("quad_decimate", quad_decimate_);
    nh_.getParam("quad_sigma", quad_sigma_);
    nh_.getParam("nthreads", nthreads_);
    nh_.getParam("debug", debug_);
    nh_.getParam("refine_edges", refine_edges_);
    nh_.getParam("tag_black_border", tag_black_border_);
    
    camera_sub_ = it_.subscribe(camera_topic_, camera_ros_buffer_size, &TagDetection::cameraCallback, this);
    tag_array_pub_ = nh_.advertise<apriltag_roswrap::TagArray>(tag_array_topic_, 10);
    detection_img_pub_ = it_.advertise(detection_topic_, 10);


    // configure apriltag detector
    if (tag_family_name_ == "tag36h11")
        tag_family_ = tag36h11_create();
    else if (tag_family_name_ == "tag25h9")
        tag_family_ = tag25h9_create();
    #if APRILTAGV == 2
        else if (tag_family_name_ == "tag36h10")
            tag_family_ = tag36h10_create();
        else if (tag_family_name_ == "tag36artoolkit")
            tag_family_ = tag36artoolkit_create();
        else if (tag_family_name_ == "tag25h7")
            tag_family_ = tag25h7_create();
    #elif APRILTAGV == 3
        else if (tag_family_name_ == "tag16h5")
            tag_family_ = tag16h5_create();
        else if (tag_family_name_ == "tagCircle21h7")
            tag_family_ = tagCircle21h7_create();
        else if (tag_family_name_ == "tagCircle49h12")
            tag_family_ = tagCircle49h12_create();
        else if (tag_family_name_ == "tagCustom48h12")
            tag_family_ = tagCustom48h12_create();
        else if (tag_family_name_ == "tagStandard41h12")
            tag_family_ = tagStandard41h12_create();
        else if (tag_family_name_ == "tagStandard52h13")
            tag_family_ = tagStandard52h13_create();
    #endif

    else {
        std::cout << tag_family_name_ << ": Unrecognized tag family name. Use e.g. \"tag36h11\".";
        exit(-1);
    }

    #if APRILTAGV == 2
        tag_family_->black_border = tag_black_border_;  // not anymore in apriltag 3
        std::cout << "tag_family_->black_border: " << tag_family_->black_border << std::endl;
    #endif

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
    // std::cout << "size: " << img_msg->width << "x" << img_msg->height << std::endl;
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
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        std::cout << "ID: " << det->id << std::endl;

        // create a tag ros msg and fill it 
        apriltag_roswrap::Tag tag_msg; 
        tag_msg.tag_size = tag_size_;
        tag_msg.id = det->id;
        std::vector<cv::Point2d> corner_vec;
        for (int ci=0; ci < 4; ci++){
            geometry_msgs::Point pt;
            pt.x = det->p[ci][0];
            pt.y = det->p[ci][1];
            tag_msg.corners.push_back(pt);

            corner_vec.push_back(cv::Point2d(det->p[ci][0], det->p[ci][1]));
        }

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

        // POSE ESTIMATION
        tag_msg.is_pose_estimated = true;
        apriltag_pose_t pose1, pose2;
        double err1, err2;
        int n_iter = 10;
        estimate_tag_pose_orthogonal_iteration(&det_info_, &err1, &pose1, &err2, &pose2, n_iter);
        
        // // Alternative Easier API, look into the code to see the diff
        // apriltag_pose_t pose1;
        // double err1 = estimate_tag_pose(&det_info_, &pose1);

        tag_msg.err1 = err1;
        tag_msg.err2 = err2;

        Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> R1(pose1.R->data);
        Eigen::Quaterniond q1(R1);
        Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>> R2(pose2.R->data);
        Eigen::Quaterniond q2(R1);

        geometry_msgs::PoseStamped pose1_msg;
        pose1_msg.header.stamp = img_msg->header.stamp;
        pose1_msg.pose.position.x = matd_get(pose1.t, 0, 0);
        pose1_msg.pose.position.y = matd_get(pose1.t, 0, 1);
        pose1_msg.pose.position.z = matd_get(pose1.t, 0, 2);
        pose1_msg.pose.orientation.x = q1.coeffs()(0);
        pose1_msg.pose.orientation.y = q1.coeffs()(1);
        pose1_msg.pose.orientation.z = q1.coeffs()(2);
        pose1_msg.pose.orientation.w = q1.coeffs()(3);

        geometry_msgs::PoseStamped pose2_msg;
        pose2_msg.header.stamp = img_msg->header.stamp;
        pose2_msg.pose.position.x = matd_get(pose2.t, 0, 0);
        pose2_msg.pose.position.y = matd_get(pose2.t, 1, 0);
        pose2_msg.pose.position.z = matd_get(pose2.t, 2, 0);
        pose2_msg.pose.orientation.x = q2.coeffs()(0);
        pose2_msg.pose.orientation.y = q2.coeffs()(1);
        pose2_msg.pose.orientation.z = q2.coeffs()(2);
        pose2_msg.pose.orientation.w = q2.coeffs()(3);

        tag_msg.pose1 = pose1_msg;
        tag_msg.pose2 = pose2_msg;

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
