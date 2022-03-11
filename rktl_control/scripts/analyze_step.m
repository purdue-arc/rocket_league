% Analyze step responses to perform a system ID

% Collect bag files of effort and odom
% Run the following bash command in the folder containing responses:
%   for bag in *.bag; do
%   rostopic echo -b $bag -p effort/throttle > "$bag"_effort.csv
%   rostopic echo -b $bag -p odom/twist/twist/linear/x > "$bag"_velocity.csv
%   rostopic echo -b $bag -p pose_sync/pose/pose/position > "$bag"_position.csv
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

    % read in data
    effort = readtable(strcat(file.folder, '/', file.name, '_effort.csv'));
    velocity = readtable(strcat(file.folder, '/', file.name, '_velocity.csv'));
    position = readtable(strcat(file.folder, '/', file.name, '_position.csv'));

    % convert time from ns since epoch to seconds since start of bag
    t0 = min(effort.x_time(1), velocity.x_time(1));
    effort.x_time = (effort.x_time - t0) * 10^-9;
    velocity.x_time = (velocity.x_time - t0) * 10^-9;
    position.x_time = (position.x_time - t0) * 10^-9;

    % manually calculate velocity from position to compare to filter output
    xy = [position.field_x position.field_y];
    delta_xy = [0 0; xy(2:end, :) - xy(1:end-1, :)];
    dt = position.x_time(2) - position.x_time(1);
    v_raw = sqrt(sum(delta_xy.^2 / dt, 2));
    v_smooth = smoothdata(v_raw);

    figure
    plot(effort.x_time, effort.field), hold
    plot(velocity.x_time, velocity.field)
    plot(position.x_time, v_raw)
    plot(position.x_time, v_smooth)
    xlabel("Time, seconds")
    ylabel("Effort | Velocity, m/s")
    legend("effor", "filter velocity", "raw velocity", "smoothed velocity")
    title(file.name, 'Interpreter', 'none')
end
