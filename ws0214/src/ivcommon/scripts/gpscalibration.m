clc,clear,close all;
gpsdata = load('/home/jkj/catkin_ws/logdata/2017-12-04_11-20-00/gpsmetricdata.txt');

lat = gpsdata(:,2);
lon = gpsdata(:,3);
plot(lon,lat,'-.k','markersize',3);
hold on;box off;
xlabel('x','fontsize',16); ylabel('y','fontsize',16);
axis([min(lon(:)) max(lon(:)) min(lat(:))-0.01 max(lat(:))+0.01]);
P = polyfit(lon,lat,1);
t = linspace(min(lon(:))-0.1,max(lon(:))+0.1,100);
S3=polyval(P,t);
plot(t,S3,'-r');
theta = atan(P(1))*180/pi;
%% fitting heading 
heading = gpsdata(:,5);
% mu_heading = mean(heading);
count = length(heading);
time = [1:count]';
figure();
plot(1:count,heading,'-.k','markersize',3);
hold on;box off;
xlabel('t','fontsize',16); ylabel('heading','fontsize',16);
axis([min(time(:)) max(time(:)) min(heading(:))-10 max(heading(:))+10]);
P = polyfit(time,heading,1);
t = linspace(min(time(:))-100,max(time(:))+100,100);
S3=polyval(P,t);
mu_heading = mean(S3);
plot(t,S3,'-r');