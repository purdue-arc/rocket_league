/* Node to help focus
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/focus_vis.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "focus_vis");

    ros::NodeHandle nh;
    rktl_perception::FocusVis focusVis(nh);

    ros::spin();
    return 0;
}
