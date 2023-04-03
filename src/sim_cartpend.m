clear all, close all, clc

m = 1;
M = 5;
L = 2;
g = -10;
d = 1;

tspan = 0:.1:30;
y0 = [0; 0; pi; .5];
[t,y] = ode45(@(t,y)cartpend(y,m,M,L,g,d,0),tspan,y0);

myVideo = VideoWriter('cartpend'); % open video file
myVideo.FrameRate = 35;  
open(myVideo)

for k=1:100:length(t)
    drawcartpend_bw(y(k,:),m,M,L);
    pause(0.01)  % Pause and grab frame
    frame = getframe(gcf); % get frame
    writeVideo(myVideo, frame);
end