/* Node to detect the ball's position on the field.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include "rktl_perception/ball_detection_node.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ball_detection");
    BallDetection detector;
    ros::spin();
    return 0;
}
