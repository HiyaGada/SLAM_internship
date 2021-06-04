initialise

%Sigma Ricatti eqn
iter=1000;
dt=0.001;
Jag(1:(N+6),1:(N+6))=Jag_0;

for i=1:iter
    Jag(1:(N+6),1+i*(N+6):(i+1)*(N+6))=Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6)) + dt*(P_- (Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*C*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))));
end

%Delta equation
H= zeros(N+6);

H(1:6,1:6)= eye(6);

for i=1:n
    H(4+(i*3):6+(i*3),4+(i*3):6+(i*3))=transpose(R_P0);
end

%Loop for true Pose
Pose=P_0;
xp=xp_0;

for i =1:iter   
    Pose(1:4,4*(i+1)-3:4*(i+1))= Pose(1:4,4*i-3:4*i)*expm(V_t(i-1)*dt);
end

%Loop for X
X(1:4+n, 1:4+n)=eye(4+n);

for i=1:iter
    %Find y_k
    %y_k=transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)));
    %Find delta
    for k=1:n
        delta(k*3-2:k*3,1)=X(1:3,(i)*(4+n)-3-n :(i)*(4+n)-1-n)*(transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)))) - h(:,k)+ (X(1:3,(i)*(4+n)-n)- X(1:3,(i)*(4+n)-n+k));
    end
    %Find Dlta
    Dlta=H*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*(delta);
    %Find DLTA
    DLTA=zeros(4+n);
    DLTA(1:3,1:3)=skew(Dlta(1:3));
    DLTA(1:3,4)=Dlta(4:6);
    for j=1:n
        DLTA(1:3,4+j)=Dlta(3*j+4:3*j+6);
    end
    %Find LIFT
    LIFT=zeros(4+n);
    LIFT(1:4,1:4)=V_t(i);
    %The differential equation
    X(1:4+n,(i+1)*(4+n)-3-n :(i+1)*(4+n))= expm(dt*DLTA)*X(1:4+n,(i)*(4+n)-3-n :(i)*(4+n))*expm(dt*LIFT);
end

function V=V_t(i)
    vy=4;
    vz=0;
    ox=0;
    oy=0;
    oz=10;
    dt=0.001;
    V= [0,-oz,oy,-vy*sin(oz*i*dt); oz,0,-ox,vy*cos(oz*i*dt); -oy,-ox,0,vz; 0,0,0,0];
end
function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end
