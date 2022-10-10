load('variables')

%Plot Euler angles according to ZYX
angles_eqf1 =  zeros(iter, 3);
angles_eqf2 =  zeros(iter, 3);
for i=1:iter
    angles_eqf1(i, :) = rotm2eul(P_hat_eqf1(1:3,4*i-3:4*i-1));
    angles_eqf2(i, :) = rotm2eul(P_hat_eqf2(1:3,4*i-3:4*i-1));
end

figure;
% alpha  
subplot(3,1,1);
plot(1:iter , angles_eqf1(:, 1), 'Color','blue','LineStyle','-', 'LineWidth',2, 'DisplayName','EQF1')
hold on
plot(1:iter , angles_eqf2(:, 1), 'Color','green','LineStyle','--', 'LineWidth',2, 'DisplayName','EQF2')
hold off
title('\alpha angle')
legend

% beta
subplot(3,1,2);
plot(1:iter , angles_eqf1(:, 2), 'Color','blue','LineStyle','-', 'LineWidth',2, 'DisplayName','EQF1')
hold on
plot(1:iter , angles_eqf2(:, 2), 'Color','green','LineStyle','--', 'LineWidth',2, 'DisplayName','EQF2')
hold off
title('\beta angle')
legend

% gamma 
subplot(3,1,3);
plot(1:iter , angles_eqf1(:, 3), 'Color','blue','LineStyle','-', 'LineWidth',2, 'DisplayName','EQF1')
hold on
plot(1:iter , angles_eqf2(:, 3), 'Color','green','LineStyle','--', 'LineWidth',2, 'DisplayName','EQF2')
hold off
title('\gamma angle')
legend



