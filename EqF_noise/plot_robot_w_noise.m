load('EqF_variables')

hold on
axis([0 1 -0.1 0.9])
%Plot iterated position of robot
for i= 1:iter-1
    scatter(x_hat(1,i),x_hat(2,i),'filled','green','HandleVisibility','off')
    pause(.01)
end

scatter(x_hat(1,iter),x_hat(2,iter),'filled','green','DisplayName','Iterated position')

%Plot true position of robot
for i=1:iter-1
    x_ttrue=Pose(1:3,4*i);
    scatter(x_ttrue(1),x_ttrue(2), 'filled','red','HandleVisibility','off')
    pause(.01)
end

x_ttrue=Pose(1:3,4*iter);
scatter(x_ttrue(1),x_ttrue(2),'filled','red','DisplayName','True position')

title('Robot position')
hold off
legend