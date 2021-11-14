pathname = "../../../bags/"; %change this to the directory of YOUR rosbag
%uncomment the right rosbag file
%bagname = "2021-10-31-19-51-40.bag";
bagname = "2021-10-31-19-53-51.bag";
%bagname = "2021-10-31-19-55-46.bag";
%bagname = "2021-10-31-19-56-14.bag";
%bagname = "2021-10-31-19-56-42.bag";
%bagname = "2021-10-31-19-57-14.bag";
%bagname = "2021-10-31-19-57-30.bag";
%bagname = "2021-10-31-19-58-14.bag";
%bagname = "2021-10-31-19-58-28.bag";
%bagname = "2021-10-31-19-58-57.bag";
%bagname = "2021-10-31-19-59-32.bag";

bag = rosbag(strcat(pathname, bagname));

% velocity_time = [];
% velocity_value = [];
%
% last_pose = [0 0 0];
% last_time = 0;
% 
% msgs = readMessages(select(bag, "Topic", "/car0/pose"));
% for i = 1:numel(msgs)
%     x = msgs{i}.Pose.Pose.Position.X;
%     y = msgs{i}.Pose.Pose.Position.Y;
%     z = msgs{i}.Pose.Pose.Position.Z;
%     pose = [x y z];
%     t = msgs{i}.Header.Stamp.seconds();
%         
%     if i == 1
%         velocity_value = [velocity_value 0];
%         velocity_time = [velocity_time t];
%         last_pose = pose;
%         last_time = t;
%     else
%         if t ~= last_time
%             delta = pose - last_pose;
%             mag = sqrt(sum(delta.^2));
%             v = mag / (t - last_time);
%             velocity_value = [velocity_value v];
%             velocity_time = [velocity_time t];
%             last_pose = pose;
%             last_time = t;
%         end
%     end
% end

msgs = timeseries(select(bag, "Topic", "control_effort/throttle"));
msgs.Time = msgs.Time - msgs.Time(1);
% effort_time = [];
% effort_value = [];
% for i = 1:numel(msgs)
%     t = msgs{i}.Header.Stamp.seconds();
%     v = msgs{i}.data;
% 
%     effort_value = [effort_value v];
%     effort_time = [effort_time t];
% end   


% t0 = min(effort_time(1), velocity_time(1));
% effort_time = effort_time - t0;
velocity_time = velocity_time - velocity_time(1);

hold on
yyaxis left
plot(msgs.Time, msgs.Data);
yyaxis right
plot(velocity_time, velocity_value);