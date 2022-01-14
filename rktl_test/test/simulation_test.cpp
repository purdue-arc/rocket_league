/* Node to test simulation.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "gtest/gtest.h"

class TargetTest: public ::testing::Test
{
public:
    TargetTest(): spinner(0) {};
    ~TargetTest() {};

    ros::NodeHandle* node;
    ros::AsyncSpinner* spinner;
    ros::Publisher pub;

    void SetUp() override
    {
        ::testing::Test::SetUp();
        this->node = new ros::NodeHandle("~");
        this->pub = this->node->advertise<std_msgs::Float64>("effort/throttle", 1);
        this->spinner = new ros::AsyncSpinner(0);
        this->spinner->start();
    };

    void TearDown() override
    {
        ros::shutdown();
        delete this->spinner;
        delete this->node;
        ::testing::Test::TearDown();
    }
};

TEST_F(TargetTest, test_ok)
{
    std_msgs::Float64 fwd;
    fwd.data = 4.0;
    this->pub.publish(fwd);
    ros::Duration(20.).sleep();
    ASSERT_TRUE(false);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_name");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged(); // To show debug output in the tests
    }
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
