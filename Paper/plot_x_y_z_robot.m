load('variables')

figure;
% X position 
subplot(3,1,1);
plot(1:iter , x_hat_eqf1(1,:), 'Color','blue','LineStyle','-', 'LineWidth',2, 'DisplayName','EQF1')
hold on
plot(1:iter , x_hat_eqf2(1,:), 'Color','green','LineStyle','--', 'LineWidth',2, 'DisplayName','EQF2')
hold off
title('X position')
legend

% Y position 
subplot(3,1,2);
plot(1:iter , x_hat_eqf1(2,:), 'Color','blue','LineStyle','-', 'LineWidth',2, 'DisplayName','EQF1')
hold on
plot(1:iter , x_hat_eqf2(2,:), 'Color','green','LineStyle','--', 'LineWidth',2, 'DisplayName','EQF2')
hold off
title('Y position')
legend

% Z position 
subplot(3,1,3);
plot(1:iter , x_hat_eqf1(3,:), 'Color','blue','LineStyle','-', 'LineWidth',2, 'DisplayName','EQF1')
hold on
plot(1:iter , x_hat_eqf2(3,:), 'Color','green','LineStyle','--', 'LineWidth',2, 'DisplayName','EQF2')
hold off
title('Z position')
legend


