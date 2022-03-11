% Analyze step responses to perform a system ID

% Collect bag files of effort and odom
% Run the following bash command in the folder containing responses:
%   for bag in *.bag; do
%   rostopic echo -b $bag -p effort/throttle > "$bag"_effort.csv
%   rostopic echo -b $bag -p odom/twist/twist/linear/x > "$bag"_velocity.csv
%   done

close all
clear

% Location to get bag files from
bagdir = "../../../../data/bags/step_responses";

files = dir(bagdir);
for i = 1:numel(files)
    file = files(i);
    if ~endsWith(file.name, '.bag')
        continue
    end

    effort = readtable(strcat(file.folder, '/', file.name, '_effort.csv'));
    velocity = readtable(strcat(file.folder, '/', file.name, '_velocity.csv'));

    % ROS toolbox commands:
    % may work on Linux, don't work on Windows due to custom msgs
    % would remove need for bash command above, and allow direct access to bag file
    %   bag = rosbag(strcat(file.folder, '/', file.name));
    %   effort = timeseries(select(bag, 'Topic', 'effort'), 'throttle');
    %   velocity = timeseries(select(bag, 'Topic', 'odom'), 'twist.twist.linear.x');

    % convert time from ns since epoch to seconds since start of bag
    t0 = min(effort.x_time(1), velocity.x_time(1));
    effort.x_time = (effort.x_time - t0) * 10^-9;
    velocity.x_time = (velocity.x_time - t0) * 10^-9;

    figure
    plot(effort.x_time, effort.field), hold
    plot(velocity.x_time, velocity.field)
    xlabel("Time, seconds")
    ylabel("effort | velocity, m/s")
    title(file.name, 'Interpreter', 'none')
    legend('effort', 'velocity')
end

% % Optionally, overwrite time with more precise values
% % poses.Time = poses.Data(:, 1) + poses.Data(:, 1).*10^-9;
% 
% % Isolate position (x, y, z)
% raw_poses.Data = raw_poses.Data(:, 4:6);
% poses.Data = poses.Data(:, 4:6);
% 
% % Calculate velocity
% delta_xyz = poses.Data - [0 0 0; poses.Data(1:end-1, :)];
% delta_t = poses.Time - [0; poses.Time(1:end-1, :)];
% v_xyz = delta_xyz ./ delta_t;
% v = sqrt(sum(v_xyz.^2, 2));
% 
% velocities = timeseries(smoothdata(v), poses.Time);
% 
% % Calculate effort
% efforts = timeseries(select(bag, "Topic", "control_effort/throttle"));
% 
% t0 = min(poses.Time(1), efforts.Time(1));
% raw_poses.Time = raw_poses.Time - t0;
% poses.Time = poses.Time - t0;
% velocities.Time = velocities.Time - t0;
% efforts.Time = efforts.Time - t0;
% 
% hold on
% yyaxis left
% plot(velocities);
% yyaxis right
% plot(efforts);
% 
% figure
% hold on
% plot(raw_poses);
% plot(poses);