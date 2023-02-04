/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/localizer.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "localizer");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    rktl_perception::Localizer localizer(nh, pnh);

    ros::spin();
    return 0;
}
