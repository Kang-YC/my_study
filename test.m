load lidar.txt
load odom.txt
load coupled.txt
plot((odom(:,2)-odom(1,2)), (odom(:,3)-odom(1,3)),'g')
angle = -odom(1,5)
a = 0.1
hold on;plot((lidar(:,2))*sin(angle-a)+(lidar(:,4))*cos(angle-a), (lidar(:,2))*cos(angle-a)-(lidar(:,4))*sin(angle-a),'r')
hold on;plot((coupled(:,2))*sin(angle-a)+(coupled(:,4))*cos(angle-a), (coupled(:,2))*cos(angle-a)-(coupled(:,4))*sin(angle-a),'k')
print('test.png', '-dpng')
