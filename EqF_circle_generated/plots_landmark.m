%Eqf_eqn

%Plot true landmark posititons
scatter(px, py, 'filled', 'red', 'DisplayName','True Landmark positions')
title('Landmarks')

hold on

%Plot iterated landmark positions
for i=1:iter-1
    for k= 1:n
    p_temp(3*i-2:3*i,k)=p_0(:,k)+(R_P0*X(1:3,(i)*(4+n)-n+k));
    end
    scatter(p_temp(3*i-2,:),p_temp(3*i-1,:),'filled','blue','HandleVisibility','off') 
    pause(.1)
end

for k= 1:n
    p_temp(3*iter-2:3*iter,k)=p_0(:,k)+(R_P0*X(1:3,(iter)*(4+n)-n+k));
end
scatter(p_temp(3*iter-2,:),p_temp(3*iter-1,:),'filled','green','DisplayName','Iterated Landmark position')
    

hold off
legend