/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/localizer.h>

int main(int argc, char* argv[]) {
    // ros::init(argc, argv, "localizer");
    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");

    rclcpp::init(argc, argv);
    auto node = rclcpp:Node::make_shared("localizer");

    rktl_perception::Localizer localizer(node);

    ros::spin();
    return 0;
}
