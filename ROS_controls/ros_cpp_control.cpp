#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;

    // Create publisher object
    ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);

    // Loop to publish messages
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello, world!";
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
