%Number of iterations
iter=100;

%Set seed
rng('default');

%Number of landmarks and position
n=5;
px = randn(1,n);
py = randn(1,n);

%Making known p vector
p = zeros(2, n);
for k= 1:n
    p(:,k)=[px(k);py(k)];
end

%Real Initial pose of robot
P_in_r=[1,0,0.7; 0,1,0.5; 0,0,1];
R_Pin_r= P_in_r(1:2,1:2);
xp_in_r= P_in_r(1:2,3);


%State and Output Matrix dimensions
M=3;
N=2*n;

%Choose Sigma_0
Jag_0 = 0.5*eye(M);
eta = mvnrnd(zeros(1, M), Jag_0, 1);
eta_P= alg(eta);  

%Estimated Initial pose of robot
P_in_e= P_in_r*expm(eta_P);    

%Choose origin (\xi^{not})
P_0= eye(3);
R_P0= P_0(1:2,1:2);
xp_0= P_0(1:2,3);


%Make V_t
vy=4;
oz=10;
dt=0.1;
V_t=zeros(3,3*iter);
for i=1:iter
    V_t(1:3,3*i-2:3*i)=[0,-oz,-vy*sin(oz*(i-1)*dt); oz,0,vy*cos(oz*(i-1)*dt); 0,0,0];
end

%Make true Pose
Pose=zeros(3,3*iter);
Pose(1:3,1:3)=P_in_r;
for i =1:iter-1   
    Pose(1:3,3*(i+1)-2:3*(i+1))= Pose(1:3,3*i-2:3*i)*expm(V_t(1:3,3*i-2:3*i)*dt);
end

%Making simulated measurement y_k
y=zeros(2*n,iter);
for i=1:iter
   for k=1:n
       y(2*k-1:2*k,i)=transpose(Pose(1:2, 3*i-2:3*i-1))*(p(:,k)-(Pose(1:2,3*i)));
   end
end






%Measure the origin output
h = zeros(2, n);
for k= 1:n
    h(:,k)=transpose(R_P0)*(p(:,k)-xp_0);
end


%Find theta of origin
R_P0_3 = eye(3);
R_P0_3(1:2, 1:2)=R_P0; 
angles = rotm2eul(R_P0_3);
theta_P0 = angles(1);

%Initialising the C matrix
C =zeros(N, M*iter);

%%% EQF %%%

%Loop for X
X = zeros(3, 3*(iter+1)); %Last term not considered
X(1:3,1:3)= inv(P_0)*P_in_e;

%State and Output matrix
P= 0.1*eye(M);
Q= 0.1*eye(N);

%T matrix for transforming Jag_0
T= eye(M);
T(2:3, 1) = -[0 -1; 1 0]*R_P0*X(1:2, 3); %confirm again

%Initialise Sigma Ricatti eqn
Jag=zeros(M,(iter+1)*M); %Last element not used 
Jag(1:M,1:M)=T*Jag_0*transpose(T); 

%Defining for convenience 
R_90_theta0 = zeros(2,2);
R_90_theta0(1,1) = cos((pi/2) + theta_P0);
R_90_theta0(2,2) = cos((pi/2) + theta_P0);
R_90_theta0(1,2) = -sin((pi/2) + theta_P0);
R_90_theta0(2,1) = sin((pi/2) + theta_P0);

%%% !!!
% Differential eqn

for i=1:iter
    %Calculate C matrix
    for k=1:n
        C(2*k-1:2*k, 3*i-2)= transpose(X(1:2, 3*i-2:3*i-1))*[0 -1; 1 0]*h(:,k); %confirm again
        C(2*k-1:2*k, 3*i-1:3*i) = transpose(X(1:2, 3*i-2:3*i-1))*transpose(R_P0); %confirm again
    end
    
    %Eqn #2
    %Find delta
    delta_1 = zeros(2*n, 1);
    for k=1:n
        delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X(1:2, 3*i-2:3*i-1))*transpose(R_P0)*(p(:, k) - X(1:2, 3*i));
    end
    delta_2 = Jag(1:(M),1+(i-1)*(M):i*(M))*transpose(C(:, 3*i-2:3*i))*inv(Q)*delta_1(:,1);
    delta_3 = [R_90_theta0(1,1)*delta_2(1), R_90_theta0(1,2)*delta_2(1), delta_2(2);...
             R_90_theta0(2,1)*delta_2(1), R_90_theta0(2,2)*delta_2(1), delta_2(3);...
             0, 0, 0];
    delta = -inv(P_0)*delta_3;

    %Find LIFT
    LIFT=V_t(1:3,3*i-2:3*i);
    
    %Eqn #1
    %The differential equation with X
    X(1:3,3*(i+1)-2 :3*(i+1))= expm(dt*delta)*X(1:3,3*i-2 :3*i)*expm(dt*LIFT);  %??
    
    %Eqn #3
    %Sigma Ricatti equation
    Jag(1:M,1+i*M:(i+1)*M)= dt*(P) + inv(dt*transpose(C(:, 3*i-2:3*i))*inv(Q)*C(1:2*n, 3*i-2:3*i) + inv(Jag(1:(M),1+(i-1)*(M):i*(M)))) ;
end


%Iterated robot pose
P_hat=zeros(3,3*iter);
x_hat=zeros(2,iter);
for i= 1:iter
    P_hat(1:3,3*i-2:3*i)=P_0*X(1:3, 3*i-2 :3*i);      %??
    x_hat(1:2,i)=P_hat(1:2,3*i);
end

%Save variables
name = strcat('variables_SE(2).mat');
save(name)


function X = alg(x)
    X=[0 -x(1) x(2) ; x(1) 0 x(3) ; 0 0 0 ]; 
end