load('variables_SE(2)_twoeqf.mat')

figure;

% X position 
subplot(3,1,1);
plot((0:iter-1)*dt , x_true(1,:), 'Color','red','LineStyle','-', 'DisplayName','True')
hold on
plot((0:iter-1)*dt , x_hat_eqf1(1,:), 'Color','blue','LineStyle','--','DisplayName','EQF 1')
plot((0:iter-1)*dt , x_hat_eqf2(1,:), 'Color','black','LineStyle',':','DisplayName','EQF 2')
plot((0:iter-1)*dt , x_hat_eqf3(1,:), 'Color','magenta','LineStyle','-.','DisplayName','EQF 3')
hold off
ylabel('x (m)')
xlabel('time (s)')
legend
grid on

% Y position 
subplot(3,1,2);
plot((0:iter-1)*dt , x_true(2,:), 'Color','red','LineStyle','-', 'DisplayName','True')
hold on
plot((0:iter-1)*dt , x_hat_eqf1(2,:), 'Color','blue','LineStyle','--','DisplayName','EQF 1')
plot((0:iter-1)*dt , x_hat_eqf2(2,:), 'Color','black','LineStyle',':','DisplayName','EQF 2')
plot((0:iter-1)*dt , x_hat_eqf3(2,:), 'Color','magenta','LineStyle','-.','DisplayName','EQF 3')
hold off
ylabel('y (m)')
xlabel('time (s)')
grid on


% theta value
subplot(3,1,3);
plot((0:iter-1)*dt , angles_true(:, 1)*180/pi, 'Color','red','LineStyle','-','DisplayName','True' )
hold on
plot((0:iter-1)*dt , angles_eqf1(:, 1)*180/pi, 'Color','blue','LineStyle','--','DisplayName','EQF 1' )
plot((0:iter-1)*dt , angles_eqf2(:, 1)*180/pi, 'Color','black','LineStyle',':','DisplayName','EQF 2' )
plot((0:iter-1)*dt , angles_eqf3(:, 1)*180/pi, 'Color','magenta','LineStyle','-.','DisplayName','EQF 3' )
hold off
ylabel('\theta (deg)')
xlabel('time (s)')
grid on

% %Angle error norm
% angle_norm_eqf1 = zeros(1,iter);
% for i=1:iter
%     angle_norm_eqf1(1, i) =norm(angles_eqf1(i, 1) - angles_true(i, 1))*180/pi;
% end
% angle_norm_eqf2 = zeros(1,iter);
% for i=1:iter
%     angle_norm_eqf2(1, i) =norm(angles_eqf2(i, 1) - angles_true(i, 1))*180/pi;
% end
% subplot(4,1,4);
% plot((0:iter-1)*dt, angle_norm_eqf1(1,:), 'Color','blue','LineStyle','--');
% hold on
% plot((0:iter-1)*dt, angle_norm_eqf1(1,:), 'Color','black','LineStyle',':');
% hold off
% ylabel('|\theta - \theta_{true}|(deg)')
% xlabel('time (s)')

sgtitle("Estimated and True Robot Pose")
