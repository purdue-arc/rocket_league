/* Node to detect the ball's position on the field.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include "rclcpp/rclcpp.hpp"
#include "rktl_perception/ball_detection.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ball_detection");

    auto ballDetection = std::make_shared<rktl_perception::BallDetection>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}