Eqf_eqn

%Plot true landmark posititons
scatter(px, py, 'filled', 'red', 'DisplayName','True Landmark positions')
title('Landmarks')

hold on

%Plot iterated landmark positions
for i=i:iter
    for k= 1:n
    p_temp(:,k)=p_0(:,k)+(R_P0*X(1:3,(i)*(4+n)-n+k));
    end
    scatter(p_temp(1,:),p_temp(2,:), 'filled','blue', 'DisplayName','Iterated landmark positions') 
end

hold off
legend