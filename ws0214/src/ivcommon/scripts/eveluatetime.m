
lidarododata = load('/home/jkj/catkin_ws/logdata/logdata/2017-11-30_17-12-56/trajectory.txt');

plot(lidarododata(:,1),lidarododata(:,8),'b')
xlabel('time/s');
ylabel('costtime/s');
grid on;
legend('time');

