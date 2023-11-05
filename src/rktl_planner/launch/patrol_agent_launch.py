
#import rospy
import rclpy
import roslaunch

def launch_patrol_planner(car_name="car0"):
    #rospy.init_node('patrol_planner_launcher', anonymous=True)
    rclpy.init_node('patrol_planner_launcher', anonymous=True)

    # Set the namespace for the node
    ns = "cars/" + car_name

    # Load the ROS parameters from a YAML file
    #param_file = rospy.get_param('rktl_planner') + "/config/patrol_planner.yaml"
    param_file = self.get_param('rktl_planner') + "/config/patrol_planner.yaml"
    # Create a launch configuration
    node = roslaunch.core.Node(
        package='rktl_planner',
        node_type='patrol_planner',
        namespace=ns,
        name='patrol_planner',
        output='screen'
    )

    # Launch the node
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    launch.launch(node)

if __name__ == '__main__':
    car_name = rospy.get_param('~car_name', 'car0')
    launch_patrol_planner(car_name)
    #rospy.spin()
    rclpy.spin()