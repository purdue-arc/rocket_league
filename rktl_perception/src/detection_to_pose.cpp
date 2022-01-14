/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include "rktl_perception/DetectionToPose.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detection_to_pose");
    DetectionToPose converter;
    ros::spin();
    return 0;
}
