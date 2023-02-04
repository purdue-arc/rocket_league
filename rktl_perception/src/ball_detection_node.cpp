/* Node to detect the ball's position on the field.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/ball_detection.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ball_detection");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    rktl_perception::BallDetection ballDetection(nh, pnh);

    ros::spin();
    return 0;
}
