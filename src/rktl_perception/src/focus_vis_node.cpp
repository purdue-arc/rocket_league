/* Node to help focus
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/focus_vis.h>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp:Node::make_shared("focus_vis");
    rktl_perception::FocusVis focusVis(node);

    rclcpp::spin();
    return 0;
}
