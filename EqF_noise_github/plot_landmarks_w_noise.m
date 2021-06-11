load('EqF_variables')

%Plot true landmark posititons
scatter(px, py, 'filled', 'red', 'DisplayName','True Landmark positions')
title('Landmarks')

%axis([0 1 0 1])
hold on

%Plot iterated landmark positions
for i=1:iter-1
    for k= 1:n
    scatter(p_hat(3*k-2,i),p_hat(3*k-1,i),'filled','blue','HandleVisibility','off')
    end
    pause(.001)
end

for k= 1:n-1
    scatter(p_hat(3*k-2,iter),p_hat(3*k-1,iter),'filled','green','HandleVisibility','off')
end

scatter(p_hat(3*n-2,iter),p_hat(3*n-1,iter),'filled','green','DisplayName','Iterated Landmark position')

hold off
legend