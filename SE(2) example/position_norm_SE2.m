load('variables_SE(2).mat')

%True position of robot
x_ttrue = zeros(2, iter);
for i=1:iter
    x_ttrue(:, i)=Pose(1:2,3*i);
end

%Find norm with real position of robot
norm_iter = zeros(iter);
for i = 1:iter
    norm_iter(i)= norm(x_hat(:, i) - x_ttrue(:, i));
end

figure;


% Plot position norm
subplot(2,1,1);
lg= plot(dt*(0:iter-1), norm_iter(1:iter));
set(gca, 'YScale', 'log')
%lg(1).LineWidth = 1;
grid on
xlabel('time (s)')
ylabel('|x - x_{true}|(m)')


%Angle error norm
angle_norm = zeros(1,iter);
for i=1:iter
    angle_norm(1, i) =norm(angles(i, 1) - angles_true(i, 1))*180/pi;
end
subplot(2,1,2);
plot((0:iter-1)*dt, angle_norm(1,:),'LineStyle','-');
ylabel('|\theta - \theta_{true}|(deg)')
xlabel('time (s)')
grid on

sgtitle("Position and Angle Error")
