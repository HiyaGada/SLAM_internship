load('error1')
load('error2')
load('error3')

%Number of iterations
iter=1000;

%Plot
plot(1:iter, norm_epsilon1 , 'DisplayName','Type1')
title('Error Evolution')
ylabel('||eps||**2')
xlabel('Iterations')

hold on
plot(1:iter, norm_epsilon2 , 'DisplayName','Type2')
plot(1:iter, norm_epsilon3 , 'DisplayName','Type3')

legend

hold off