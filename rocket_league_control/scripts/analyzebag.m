pathname = "/home/robert/arc/bags/"; %change this to the directory of YOUR rosbag
%uncomment the right rosbag file
%bagname = "2021-10-31-19-51-40.bag";
%bagname = "2021-10-31-19-53-51.bag";
%bagname = "2021-10-31-19-55-46.bag";
%bagname = "2021-10-31-19-56-14.bag";
%bagname = "2021-10-31-19-56-42.bag";
%bagname = "2021-10-31-19-57-14.bag";
%bagname = "2021-10-31-19-57-30.bag";
%bagname = "2021-10-31-19-58-14.bag";
%bagname = "2021-10-31-19-58-28.bag";
bagname = "2021-10-31-19-58-57.bag";
%bagname = "2021-10-31-19-59-32.bag";
bag = rosbag(strcat(pathname, bagname));
msg = readMessages(bag);

vx = zeros(1, length(msg) - 1);
vy = zeros(1, length(msg) - 1);
vz = zeros(1, length(msg) - 1);
t = zeros(1, length(msg) - 1);
for i = 1:(length(msg) - 1)
    if(msg{i + 1}.MessageType == "geometry_msgs/PoseWithCovarianceStamped" && msg{i}.MessageType == "geometry_msgs/PoseWithCovarianceStamped")
        vx(i) = msg{i + 1}.Pose.Pose.Position.X - msg{i}.Pose.Pose.Position.X;
        vy(i) = msg{i + 1}.Pose.Pose.Position.Y - msg{i}.Pose.Pose.Position.Y;
        vx(i) = msg{i + 1}.Pose.Pose.Position.Z - msg{i}.Pose.Pose.Position.Z;
        t(i) = i;
    end
end
vx2 = vx .^ 2;
vy2 = vy .^ 2;
vz2 = vz .^ 2;
vv = sqrt(vx2 + vy2 + vz2);
plot(t, vv);