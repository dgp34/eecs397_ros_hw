function ur10_reach
clear
g = importdata('Ground');
bt = importdata('Bin Top');
agv = importdata('AGV');
ct = importdata('Conveyor Top');
tt = importdata('Tray Top');
figure(1)
plot3(g(:,1),g(:,2),g(:,3),'k*',bt(:,1),bt(:,2),bt(:,3),'b*',agv(:,1),agv(:,2),agv(:,3),'g*',ct(:,1),ct(:,2),ct(:,3),'r*',tt(:,1),tt(:,2),tt(:,3),'y*')
title('UR10 Reachability Around Specified Magic Numbers')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
legend('Ground','Bin Top','AGV','Conveyor Top','Tray Top')
end