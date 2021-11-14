% rosinit("192.168.11.119");

sub = rossubscriber('car0/pose','DataFormat','struct');
pause(1);

while true
    [msg2,status,statustext] = receive(sub);
end