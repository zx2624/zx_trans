gpsdata = load('/home/jkj/catkin_ws/logdata/2017-12-05_22-33-13/gpsmetricdata.txt');
lidarododata = load('/home/jkj/catkin_ws/logdata/2017-12-05_22-33-13/trajectory.txt');
plot(gpsdata(:,2)-gpsdata(1,2),gpsdata(:,3)-gpsdata(1,3),'r')

%plot(gpsdata(:,1),gpsdata(:,2),'r')
hold on;
%plot(lidarododata(:,1),lidarododata(:,2),'b')
plot(lidarododata(:,2)-lidarododata(1,2),lidarododata(:,3)-lidarododata(1,3),'b')
xlabel('x/m');
ylabel('y/m');
grid on;
legend('gpsdata','lidarodometry');

hold off;
figure(2);
plot(gpsdata(:,1),gpsdata(:,4) - gpsdata(1,4),'r')
hold on
plot(lidarododata(:,1),lidarododata(:,4) - lidarododata(1,4),'b')
xlabel('time/s');
ylabel('z/^o');
legend('gpsdata','lidarodometry');

hold off;
figure(3);
plot(gpsdata(:,1),gpsdata(:,5) - gpsdata(1,5),'r')
hold on
plot(lidarododata(:,1),lidarododata(:,5) - lidarododata(1,5),'b')
xlabel('time/s');
ylabel('heading/^o');
legend('gpsdata','lidarodometry');

hold off;
figure(4);
plot(gpsdata(:,1),gpsdata(:,8) ,'r')
hold on
plot(lidarododata(:,1),lidarododata(:,9) ,'b')
xlabel('time/s');
ylabel('pitch/^o');
legend('gpsdata','lidarodometry');


hold off;
figure(5);
plot(gpsdata(:,1),gpsdata(:,9),'r')
hold on
plot(lidarododata(:,1),lidarododata(:,10) ,'b')
xlabel('time/s');
ylabel('roll/^o');
legend('gpsdata','lidarodometry');