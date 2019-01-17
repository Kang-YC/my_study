load lidar.txt
load odom.txt
load coupled.txt
angle = -odom(1,5)
a = 0.1
plot(-(coupled(:,2)), (coupled(:,4)),'k','LineWidth',1)
hold on;plot(-(lidar(:,2)), (lidar(:,4)),'g', 'LineWidth',1)
hold on;plot(-((odom(:,2)-odom(1,2))*sin(angle-a)+(odom(:,3)-odom(1,3))*cos(angle-a)), (odom(:,2)-odom(1,2))*cos(angle-a)-(odom(:,3)-odom(1,3))*sin(angle-a),'r','LineWidth',1)




%title('慢速'); % 标题
%xlabel('p'); % 横轴坐标
%ylabel('H(p)'); % 纵轴坐标

print('0115材料慢lidar.png', '-dpng')
