pathname = "../../../bags/"; %change this to the directory of YOUR rosbag
bagname = "2021-11-14-23-12-25.bag";

bag = rosbag(strcat(pathname, bagname));

raw_poses = timeseries(select(bag, "Topic", "/car0/pose"));
poses = timeseries(select(bag, "Topic", "/car0/smooth_pose"));

% Optionally, overwrite time with more precise values
% poses.Time = poses.Data(:, 1) + poses.Data(:, 1).*10^-9;

% Isolate position (x, y, z)
raw_poses.Data = raw_poses.Data(:, 4:6);
poses.Data = poses.Data(:, 4:6);

% Calculate velocity
delta_xyz = poses.Data - [0 0 0; poses.Data(1:end-1, :)];
delta_t = poses.Time - [0; poses.Time(1:end-1, :)];
v_xyz = delta_xyz ./ delta_t;
v = sqrt(sum(v_xyz.^2, 2));

velocities = timeseries(smoothdata(v), poses.Time);

% Calculate effort
efforts = timeseries(select(bag, "Topic", "control_effort/throttle"));

t0 = min(poses.Time(1), efforts.Time(1));
raw_poses.Time = raw_poses.Time - t0;
poses.Time = poses.Time - t0;
velocities.Time = velocities.Time - t0;
efforts.Time = efforts.Time - t0;

hold on
yyaxis left
plot(velocities);
yyaxis right
plot(efforts);

figure
hold on
plot(raw_poses);
plot(poses);