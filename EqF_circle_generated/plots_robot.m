Eqf_eqn

hold on

%Plot iterated position of robot
for i= 1:iter-1
    P_temp=P_0*X(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n);
    x_temp=P_temp(1:3,4);
    scatter(x_temp(1),x_temp(2),'filled','green','HandleVisibility','off')
    pause(.1)
end

P_temp=P_0*X(1:4,(iter)*(4+n)-3-n: (iter)*(4+n)-n);
x_temp=P_temp(1:3,4);
scatter(x_temp(1),x_temp(2),'filled','green','DisplayName','Iterated position')

%Plot true position of robot
P_true=P_0;

for i=1:iter-1
    P_true=P_true*expm(V_t(i-1)*dt);
    x_true=P_true(1:3,4);
    scatter(x_true(1),x_true(2),'filled','red','HandleVisibility','off')
    pause(.1)
end

P_true=P_true*expm(V_t(iter-1)*dt);
x_true=P_true(1:3,4);
scatter(x_true(1),x_true(2),'filled','red','DisplayName','True position')

title('Robot position')
hold off
legend

function V=V_t(i)
    vy=4;
    vz=0;
    ox=0;
    oy=0;
    oz=10;
    dt=0.001;
    V= [0,-oz,oy,-vy*sin(oz*i*dt); oz,0,-ox,vy*cos(oz*i*dt); -oy,-ox,0,vz; 0,0,0,0];
end