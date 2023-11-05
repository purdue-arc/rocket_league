#import rospy
import rclpy
import roslaunch
def launch_agent_nodes(agent_name="agent0", car_name="car0"):
    #rospy.init_node('agent_launcher', anonymous=True)
    self.init_node('agent_launcher', anonymous=True)

    # Set the namespace for the agent
    agent_ns = "agents/" + agent_name

    # Load path follower parameters
    path_follower_node = roslaunch.core.Node(
        package='rktl_planner',
        node_type='path_follower',
        name='path_follower',
        namespace=agent_ns,
        output='screen'
    )

    # Load path follower parameters from a YAML file
    path_follower_param_file = rospy.get_param('rktl_planner') + "/config/path_follower.yaml"
    self.set_param(agent_ns + '/path_follower', path_follower_param_file)
    #rospy.set_param(agent_ns + '/path_follower', path_follower_param_file)

    # Set the car_name parameter for the path follower node
    self.set_param(agent_ns + '/path_follower/car_name', car_name)
    #rospy.set_param(agent_ns + '/path_follower/car_name', car_name)

    # Load path planning parameters
    bezier_path_server_node = roslaunch.core.Node(
        package='rktl_planner',
        node_type='bezier_path_server',
        name='bezier_path_server',
        namespace=agent_ns,
        output='screen'
    )

    # Load path planner parameters from a YAML file
    path_planner_node = roslaunch.core.Node(
        package='rktl_planner',
        node_type='path_planner',
        name='path_planner',
        namespace=agent_ns,
        output='screen'
    )

    # Load path planner parameters from a YAML file
    path_planner_param_file = rospy.get_param('rktl_planner') + "/config/path_planner.yaml"
    self.set_param(agent_ns + '/path_planner', path_planner_param_file)
    #rospy.set_param(agent_ns + '/path_planner', path_planner_param_file)

    # Set the car_name parameter for the path planner node
    self.set_param(agent_ns + '/path_planner/car_name', car_name)
    #rospy.set_param(agent_ns + '/path_planner/car_name', car_name)

    # Create a ROS launch object and launch the nodes
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    launch.launch(path_follower_node)
    launch.launch(bezier_path_server_node)
    launch.launch(path_planner_node)

if __name__ == '__main__':
    agent_name = rospy.get_param('~agent_name', 'agent0')
    car_name = self.get_param('~car_name', 'car0')
    car_name = rospy.get_param('~car_name', 'car0')
    launch_agent_nodes(agent_name, car_name)
    #rospy.spin()
    rclpy.spin()