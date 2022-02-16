load('EqF_variables')

hold on
axis([0 1 -0.1 0.9])

%Plot true position of robot
for i=1:iter-1
    x_ttrue=Pose(1:3,4*i);
    scatter(x_ttrue(1),x_ttrue(2), 'filled','red','HandleVisibility','off')
    pause(.001)
end

x_ttrue=Pose(1:3,4*iter);
scatter(x_ttrue(1),x_ttrue(2),'filled', 'red','DisplayName','True position')

%%% EQF1 %%%
%Plot iterated position of robot
for i= 1:iter-1
    scatter(x_hat_eqf1(1,i),x_hat_eqf1(2,i),'blue','HandleVisibility','off')
    pause(.001)
end

scatter(x_hat_eqf1(1,iter),x_hat_eqf1(2,iter),'blue','DisplayName','Iterated position EQF1')

%%% EQF2 %%%
%Plot iterated position of robot
for i= 1:iter-1
    scatter(x_hat_eqf2(1,i),x_hat_eqf2(2,i),'green','HandleVisibility','off')
    pause(.001)
end

scatter(x_hat_eqf2(1,iter),x_hat_eqf2(2,iter),'green','DisplayName','Iterated position EQF2')


title('Robot position')
hold off
legend