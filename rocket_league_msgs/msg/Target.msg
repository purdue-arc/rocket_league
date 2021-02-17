Header stamp #(time message was created. Frame_id is what below is w.r.t)
string child_frame_id #(car base_link)

# desired time, position, orientation, and velocity
# may want to merge to send rocket_league_msgs/Waypoint object instead
std_msgs/Float32 delta_t
geometry_msgs/Pose pose
geometry_msgs/Twist twist
