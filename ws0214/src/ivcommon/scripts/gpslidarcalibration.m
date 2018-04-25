clc,clear,close all;
gpsdata = load('/home/jkj/catkin_ws/logdata/2017-12-05_22-26-11/gpsmetricdata.txt');
lidarododata = load('/home/jkj/catkin_ws/logdata/2017-12-05_22-26-11/trajectory.txt');

%% fitting gpstheta
x = gpsdata(:,2);
y = gpsdata(:,3);
plot(x,y,'-.k','markersize',3);
hold on;box off;
xlabel('x','fontsize',16); ylabel('y','fontsize',16);
axis([min(x(:)) max(x(:)) min(y(:))-0.01 max(y(:))+0.01]);
P = polyfit(x,y,1);
t = linspace(min(x(:))-0.1,max(x(:))+0.1,100);
S3=polyval(P,t);
plot(t,S3,'-r');
thetagps = atan(P(1))*180/pi;
legend('位置点','拟合直线');
title('gpsdata')
%% fitting lidartheta
hold off;
figure(2)
lidarx = lidarododata(:,2);
lidary = lidarododata(:,3);
plot(lidarx,lidary,'-.k','markersize',3);
hold on;box off;
xlabel('x','fontsize',16); ylabel('y','fontsize',16);
axis([min(lidarx(:)) max(lidarx(:)) min(lidary(:))-0.01 max(lidary(:))+0.01]);
P = polyfit(lidarx,lidary,1);
t = linspace(min(lidarx(:))-0.1,max(lidarx(:))+0.1,100);
S3=polyval(P,t);
plot(t,S3,'-r');
thetalidar = atan(P(1))*180/pi;
thetadiff = thetalidar - thetagps
legend('位置点','拟合直线');
title('lidarodometry')
%% calibration
[lidarnum lidardim] = size(lidarododata)
[gpsnum gpsdim] = size(gpsdata)
klidar = 1;
kgps = 1;

t0 = 5
 for j=klidar:lidarnum
    if lidarododata(j,1) > t0
        klidar = j
        break;
    end
end

roll = lidarododata(klidar,10)*pi/180;
pitch = lidarododata(klidar,9)*pi/180;
yaw = lidarododata(klidar,5)*pi/180;
lastlidarmat = angle2dcm(yaw,pitch,roll,'ZYX');
for j=kgps:gpsnum
    if gpsdata(j,1) > t0
        kgps = j
        break;
    end
end

roll = gpsdata(kgps,9)*pi/180;
pitch = gpsdata(kgps,8)*pi/180;
yaw = gpsdata(kgps,5)*pi/180;
lastgpsmat = angle2dcm(yaw,pitch,roll,'ZYX');

    
for i=10:10:60


    for j=klidar:lidarnum
        if lidarododata(j,1) > i
            klidar = j
            break;
        end
    end

    roll = lidarododata(klidar,10)*pi/180;
    pitch = lidarododata(klidar,9)*pi/180;
    yaw = lidarododata(klidar,5)*pi/180;
    lidarmat = angle2dcm(yaw,pitch,roll,'ZYX')
    inv(lastlidarmat)*lidarmat
    [V,D]=eig(inv(lastlidarmat)*lidarmat)
    for j=kgps:gpsnum
        if gpsdata(j,1) > i
            kgps = j
            break;
        end
    end

    roll = gpsdata(kgps,9)*pi/180;
    pitch = gpsdata(kgps,8)*pi/180;
    yaw = gpsdata(kgps,5)*pi/180;
    gpsmat = angle2dcm(yaw,pitch,roll,'ZYX')
    inv(lastgpsmat)*gpsmat
    [V,D]=eig(inv(lastgpsmat)*gpsmat)
end
cx= cos(roll);
sx= sin(roll);
rollmat = [1 0 0;
            0 cx sx;
            0 -sx cx];
cy= cos(pitch);
sy= sin(pitch);
pitchmat = [cy 0 -sy;
            0 1 0;
            sy 0 cy];
cz= cos(yaw);
sz= sin(yaw);
yawmat = [cz sz 0;
            -sz cz 0;
            0 0 1];
yawmat * pitchmat *rollmat;