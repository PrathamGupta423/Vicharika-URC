#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void myCallback(const std_msgs::String& msg){
    // Do something with the received message
    // For example, print it to the serial monitor
    Serial.println(msg.data);
}

ros::Subscriber<std_msgs::String> sub("my_topic", myCallback);

/**
 * @brief This function initializes the ROS node and subscribes to a topic.
 * 
 * @details This function initializes the ROS node handle object and subscribes to a topic using the nh.subscribe() function.
 * 
 * @return void
 */
void setup(){
    nh.initNode();
    nh.subscribe(sub);
}

void loop(){
    nh.spinOnce();
    delay(100); // Wait for 100ms
}
