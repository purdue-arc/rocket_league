try
    setenv('ROS_MASTER_URI','http://192.168.11.119:11311')
    setenv('ROS_IP','192.168.11.116')
    rosinit
catch err
end

callback("fake");

sub = rossubscriber('/car0/pose', @callback);
pause(1);

% while true
% [msg2,status,statustext] = receive(sub, 1);
% end

% rosshutdown

function callback(msg)
msg
end