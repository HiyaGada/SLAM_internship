load('variables_SE(2).mat')

figure;

% X position 
subplot(3,1,1);
plot((0:iter-1)*dt , x_true(1,:), 'Color','red','LineStyle','-', 'DisplayName','True')
hold on
plot((0:iter-1)*dt , x_hat(1,:), 'Color','blue','LineStyle','--','DisplayName','Estimate')
hold off
ylabel('x (m)')
xlabel('time (s)')
legend
grid on

% Y position 
subplot(3,1,2);
plot((0:iter-1)*dt , x_true(2,:), 'Color','red','LineStyle','-', 'DisplayName','True')
hold on
plot((0:iter-1)*dt , x_hat(2,:), 'Color','blue','LineStyle','--','DisplayName','Estimate')
hold off
ylabel('y (m)')
xlabel('time (s)')

grid on

% theta value
subplot(3,1,3);
plot((0:iter-1)*dt , angles_true(:, 1)*180/pi, 'Color','red','LineStyle','-','DisplayName','True' )
hold on
plot((0:iter-1)*dt , angles(:, 1)*180/pi, 'Color','blue','LineStyle','--','DisplayName','Estimate' )
hold off
ylabel('\theta (deg)')
xlabel('time (s)')

grid on

sgtitle("Estimated and True Robot Pose")

