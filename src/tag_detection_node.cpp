#include "tag_detection.hpp"

int main(int argc, char **argv)
{
    std::cout << "\n=========== WOLF MAIN ===========\n\n";

    // Init ROS
    ros::init(argc, argv, "tag_detection_node");

    // Wolf ROS node
    TagDetection tag_detection_node;

    ros::AsyncSpinner spinner(1); // Use multiple threads to handle the different callbacks
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

