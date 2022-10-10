load('variables')

%True position of robot
x_ttrue = zeros(3, iter);
for i=1:iter
    x_ttrue(:, i)=Pose(1:3,4*i);
end

%Find norm with real position of robot
norm_iter_eqf1 = zeros(iter);
norm_iter_eqf2 = zeros(iter);
for i = 1:iter
    norm_iter_eqf1(i)= norm(x_hat_eqf1(:, i) - x_ttrue(:, i));
    norm_iter_eqf2(i)= norm(x_hat_eqf2(:, i)- x_ttrue(:, i));
end

%Plot
% eqf1 = 100
% eqf2 = 50
% eqf3 = 1000
% eqf4 = 10000
% eqf5 = 700
% eqf6 = 7000

lg= plot(1:iter-1, norm_iter_eqf1(1:iter-1), ...
         1:iter-1, norm_iter_eqf2(1:iter-1),'--');
set(gca, 'YScale', 'log')
lg(1).LineWidth = 1;
lg(2).LineWidth = 1;
title('Position norm')
legend('EQF1','EQF2','Location','southwest')
grid on
