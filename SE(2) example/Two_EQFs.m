%Number of iterations
iter=201;

%Set seed
rng('default');

%Number of landmarks and position
n=5;
p = randn(2,n);

%Real Initial pose of robot
P_in_r=[1,0,0.7;
        0,1,0.5;
        0,0, 1];


%State and Output Matrix dimensions
M=3;
N=2*n;

%Choose Sigma_0
Jag_0 = 0.05*eye(M);
eta = mvnrnd(zeros(1, M), Jag_0, 1);
eta_P= alg(eta);  

%Estimated Initial pose of robot
P_in_e= P_in_r*expm(eta_P);    

%%% EQF1 %%%
%Choose origin (\xi^{not})
P_0_eqf1= eye(3);
P_0_eqf1(1:2, 3)= 1e3*[1;1];
R_P0_eqf1= P_0_eqf1(1:2,1:2);
xp_0_eqf1= P_0_eqf1(1:2,3);

%%% EQF2 %%%
%Choose origin (\xi^{not})
P_0_eqf2= eye(3);
P_0_eqf2(1:2, 3)= 1e4*[1;1];
R_P0_eqf2= P_0_eqf2(1:2,1:2);
xp_0_eqf2= P_0_eqf2(1:2,3);

%%% EQF3 %%%
%Choose origin (\xi^{not})
P_0_eqf3= eye(3);
P_0_eqf3(1:2, 3)= 1e5*[1;1];
R_P0_eqf3= P_0_eqf3(1:2,1:2);
xp_0_eqf3= P_0_eqf3(1:2,3);


%Make V_t
vy=0.5;
oz=0.2;
dt=0.1;
V_t=zeros(3,3*iter);
for i=1:iter
    V_t(1:3,3*i-2:3*i)=[0,-oz,-vy*sin(oz*(i-1)*dt); oz,0,vy*cos(oz*(i-1)*dt); 0,0,0];
end

%Make true Pose
Pose=zeros(3,3*iter);
Pose(1:3,1:3)=P_in_r;
for i = 1:iter-1   
    Pose(1:3,3*(i+1)-2:3*(i+1))= Pose(1:3,3*i-2:3*i)*expm(V_t(1:3,3*i-2:3*i)*dt);
end

%Making simulated measurement y_k
y=zeros(2*n,iter);
for i=1:iter
   for k=1:n
       y(2*k-1:2*k,i)=transpose(Pose(1:2, 3*i-2:3*i-1))*(p(:,k)-(Pose(1:2,3*i)));
   end
end


%%% EQF1 %%%
%Measure the origin output
h_eqf1 = zeros(2, n);
for k= 1:n
    h_eqf1(:,k)=transpose(R_P0_eqf1)*(p(:,k)-xp_0_eqf1);
end

%Find theta of origin
R_P0_3_eqf1 = eye(3);
R_P0_3_eqf1(1:2, 1:2)=R_P0_eqf1;                     
theta_P0_eqf1 = atan2(R_P0_3_eqf1(2,1), R_P0_3_eqf1(1,1)); % Gives beyond standard range

%Initialising the C matrix
C_eqf1 =zeros(N, M*iter);


%%% EQF2 %%%
h_eqf2 = zeros(2, n);
for k= 1:n
    h_eqf2(:,k)=transpose(R_P0_eqf2)*(p(:,k)-xp_0_eqf2);
end

%Find theta of origin
R_P0_3_eqf2 = eye(3);
R_P0_3_eqf2(1:2, 1:2)=R_P0_eqf2;                     
theta_P0_eqf2 = atan2(R_P0_3_eqf2(2,1), R_P0_3_eqf2(1,1)); % Gives beyond standard range

%Initialising the C matrix
C_eqf2 =zeros(N, M*iter);

%%% EQF3 %%%
%Measure the origin output
h_eqf3 = zeros(2, n);
for k= 1:n
    h_eqf3(:,k)=transpose(R_P0_eqf3)*(p(:,k)-xp_0_eqf3);
end

%Find theta of origin
R_P0_3_eqf3 = eye(3);
R_P0_3_eqf3(1:2, 1:2)=R_P0_eqf3;                     
theta_P0_eqf3 = atan2(R_P0_3_eqf3(2,1), R_P0_3_eqf3(1,1)); % Gives beyond standard range

%Initialising the C matrix
C_eqf3 =zeros(N, M*iter);

% EQF

%%% EQF1 %%%

%Loop for X
X_eqf1 = zeros(3, 3*(iter+1)); %Last term not considered
X_eqf1(1:3,1:3)= inv(P_0_eqf1)*P_in_e;

%T matrix for transforming Jag_0
T_eqf1= eye(M);
T_eqf1(2:3, 1) = -[0 -1; 1 0]*R_P0_eqf1*X_eqf1(1:2, 3); %confirm again

%Initialise Sigma Ricatti eqn
Jag_eqf1=zeros(M,(iter+1)*M); %Last element not used 
Jag_eqf1(1:M,1:M)=T_eqf1*Jag_0*transpose(T_eqf1);

%Defining for convenience 
R_90_theta0_eqf1 = zeros(2,2);
R_90_theta0_eqf1(1,1) = cos((pi/2) + theta_P0_eqf1);
R_90_theta0_eqf1(2,2) = cos((pi/2) + theta_P0_eqf1);
R_90_theta0_eqf1(1,2) = -sin((pi/2) + theta_P0_eqf1);
R_90_theta0_eqf1(2,1) = sin((pi/2) + theta_P0_eqf1);

%%% EQF2 %%%

%Loop for X
X_eqf2 = zeros(3, 3*(iter+1)); %Last term not considered
X_eqf2(1:3,1:3)= inv(P_0_eqf2)*P_in_e;

%T matrix for transforming Jag_0
T_eqf2= eye(M);
T_eqf2(2:3, 1) = -[0 -1; 1 0]*R_P0_eqf2*X_eqf2(1:2, 3); %confirm again

%Initialise Sigma Ricatti eqn
Jag_eqf2=zeros(M,(iter+1)*M); %Last element not used 
Jag_eqf2(1:M,1:M)=T_eqf2*Jag_0*transpose(T_eqf2);


%Defining for convenience 
R_90_theta0_eqf2 = zeros(2,2);
R_90_theta0_eqf2(1,1) = cos((pi/2) + theta_P0_eqf2);
R_90_theta0_eqf2(2,2) = cos((pi/2) + theta_P0_eqf2);
R_90_theta0_eqf2(1,2) = -sin((pi/2) + theta_P0_eqf2);
R_90_theta0_eqf2(2,1) = sin((pi/2) + theta_P0_eqf2);

%%% EQF3 %%%

%Loop for X
X_eqf3 = zeros(3, 3*(iter+1)); %Last term not considered
X_eqf3(1:3,1:3)= inv(P_0_eqf3)*P_in_e;

%T matrix for transforming Jag_0
T_eqf3= eye(M);
T_eqf3(2:3, 1) = -[0 -1; 1 0]*R_P0_eqf3*X_eqf3(1:2, 3); %confirm again

%Initialise Sigma Ricatti eqn
Jag_eqf3=zeros(M,(iter+1)*M); %Last element not used 
Jag_eqf3(1:M,1:M)=T_eqf3*Jag_0*transpose(T_eqf3);

%Defining for convenience 
R_90_theta0_eqf3 = zeros(2,2);
R_90_theta0_eqf3(1,1) = cos((pi/2) + theta_P0_eqf3);
R_90_theta0_eqf3(2,2) = cos((pi/2) + theta_P0_eqf3);
R_90_theta0_eqf3(1,2) = -sin((pi/2) + theta_P0_eqf3);
R_90_theta0_eqf3(2,1) = sin((pi/2) + theta_P0_eqf3);

%--------------------------------------------------------------
% Z matrix
Z12 = X_eqf1(1:3, 1:3)*inv(X_eqf2(1:3, 1:3)); %at t=0

Z13 = X_eqf1(1:3, 1:3)*inv(X_eqf3(1:3, 1:3)); %at t=0

% M matrix (M_trans) 
M_trans12 = eye(M); 
M_trans12(2:3, 1) = [0 -1; 1 0]*R_P0_eqf1*Z12(1:2, 3);

M_trans13 = eye(M); 
M_trans13(2:3, 1) = [0 -1; 1 0]*R_P0_eqf1*Z13(1:2, 3);

%Jag_eqf2(1:M,1:M)= M_trans*Jag_eqf1(1:3,1:3)*transpose(M_trans);

%%% EQF1 %%%

%State and Output matrix
P_eqf0 = 0.1 * eye(M);

% P_eqf1= 0.1*eye(M);
P_eqf1 = T_eqf1 * P_eqf0 * T_eqf1';
Q_eqf1= 0.1*eye(N);

%%% EQF2 %%%

%State and Output matrix
% P_eqf2= M_trans*P_eqf1*transpose(M_trans);
P_eqf2 = T_eqf2 * P_eqf0 * T_eqf2';
Q_eqf2= Q_eqf1;

%%% EQF3 %%%

%State and Output matrix
% P_eqf2= M_trans*P_eqf1*transpose(M_trans);
P_eqf3 = T_eqf3 * P_eqf0 * T_eqf3';
Q_eqf3= Q_eqf1;

%--------------------------------------------------------------------------
% Differential eqn

for i=1:iter
    %Calculate C matrix
    for k=1:n
        C_eqf1(2*k-1:2*k, 3*i-2)= transpose(X_eqf1(1:2, 3*i-2:3*i-1))*[0 -1; 1 0]*h_eqf1(:,k); %confirm again
        C_eqf1(2*k-1:2*k, 3*i-1:3*i) = transpose(X_eqf1(1:2, 3*i-2:3*i-1))*transpose(R_P0_eqf1); %confirm again
    end
    
    %Eqn #2
    %Find delta
    delta_1 = zeros(2*n, 1);
    for k=1:n
        delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X_eqf1(1:2, 3*i-2:3*i-1))*(R_P0_eqf1'*(p(:, k) - xp_0_eqf1) - X_eqf1(1:2, 3*i));
        %delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X_eqf1(1:2, 3*i-2:3*i-1))*transpose(R_P0_eqf1)*(p(:, k) - X_eqf1(1:2, 3*i));
    end
    delta_2 = Jag_eqf1(1:(M),1+(i-1)*(M):i*(M))*transpose(C_eqf1(:, 3*i-2:3*i))*inv(Q_eqf1)*delta_1(:,1);
    delta_3 = [R_90_theta0_eqf1(1,1)*delta_2(1), R_90_theta0_eqf1(1,2)*delta_2(1), delta_2(2);...
               R_90_theta0_eqf1(2,1)*delta_2(1), R_90_theta0_eqf1(2,2)*delta_2(1), delta_2(3);...
               0, 0, 0];
    delta_eqf1 = -inv(P_0_eqf1)*delta_3;

    %Find LIFT
    LIFT_eqf1=V_t(1:3,3*i-2:3*i);
    
    %Eqn #1
    %The differential equation with X
    X_eqf1(1:3,3*(i+1)-2 :3*(i+1))= expm(dt*delta_eqf1)*X_eqf1(1:3,3*i-2 :3*i)*expm(dt*LIFT_eqf1);  %??
    
    %Eqn #3
    %Sigma Ricatti equation
    Jag_eqf1(1:M,1+i*M:(i+1)*M)= dt*(P_eqf1) + inv(dt*transpose(C_eqf1(:, 3*i-2:3*i))*inv(Q_eqf1)*C_eqf1(1:2*n, 3*i-2:3*i) + inv(Jag_eqf1(1:(M),1+(i-1)*(M):i*(M)))) ;
end

% Differential eqn

for i=1:iter
    %Calculate C matrix
    for k=1:n
        C_eqf2(2*k-1:2*k, 3*i-2)= transpose(X_eqf2(1:2, 3*i-2:3*i-1))*[0 -1; 1 0]*h_eqf2(:,k); %confirm again
        C_eqf2(2*k-1:2*k, 3*i-1:3*i) = transpose(X_eqf2(1:2, 3*i-2:3*i-1))*transpose(R_P0_eqf2); %confirm again
    end
    
    %Eqn #2
    %Find delta
    delta_1 = zeros(2*n, 1);
    for k=1:n
        delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X_eqf2(1:2, 3*i-2:3*i-1))*(R_P0_eqf2'*(p(:, k) - xp_0_eqf2) - X_eqf2(1:2, 3*i));
        %delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X_eqf2(1:2, 3*i-2:3*i-1))*transpose(R_P0_eqf2)*(p(:, k) - X_eqf2(1:2, 3*i));
    end
    delta_2 = Jag_eqf2(1:(M),1+(i-1)*(M):i*(M))*transpose(C_eqf2(:, 3*i-2:3*i))*inv(Q_eqf2)*delta_1(:,1);
    delta_3 = [R_90_theta0_eqf2(1,1)*delta_2(1), R_90_theta0_eqf2(1,2)*delta_2(1), delta_2(2);...
             R_90_theta0_eqf2(2,1)*delta_2(1), R_90_theta0_eqf2(2,2)*delta_2(1), delta_2(3);...
             0, 0, 0];
    delta_eqf2 = -inv(P_0_eqf2)*delta_3;

    %Find LIFT
    LIFT_eqf2=V_t(1:3,3*i-2:3*i);
    
    %Eqn #1
    %The differential equation with X
    X_eqf2(1:3,3*(i+1)-2 :3*(i+1))= expm(dt*delta_eqf2)*X_eqf2(1:3,3*i-2 :3*i)*expm(dt*LIFT_eqf2);  %??
    
    %Eqn #3
    %Sigma Ricatti equation
    Jag_eqf2(1:M,1+i*M:(i+1)*M)= dt*(P_eqf2) + inv(dt*transpose(C_eqf2(:, 3*i-2:3*i))*inv(Q_eqf2)*C_eqf2(1:2*n, 3*i-2:3*i) + inv(Jag_eqf2(1:(M),1+(i-1)*(M):i*(M)))) ;
end

% Differential eqn

for i=1:iter
    %Calculate C matrix
    for k=1:n
        C_eqf3(2*k-1:2*k, 3*i-2)= transpose(X_eqf3(1:2, 3*i-2:3*i-1))*[0 -1; 1 0]*h_eqf3(:,k); %confirm again
        C_eqf3(2*k-1:2*k, 3*i-1:3*i) = transpose(X_eqf3(1:2, 3*i-2:3*i-1))*transpose(R_P0_eqf3); %confirm again
    end
    
    %Eqn #2
    %Find delta
    delta_1 = zeros(2*n, 1);
    for k=1:n
        delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X_eqf3(1:2, 3*i-2:3*i-1))*(R_P0_eqf3'*(p(:, k) - xp_0_eqf3) - X_eqf3(1:2, 3*i));
        %delta_1(2*k-1:2*k, 1)= y(2*k-1:2*k, i) - transpose(X_eqf3(1:2, 3*i-2:3*i-1))*transpose(R_P0_eqf3)*(p(:, k) - X_eqf3(1:2, 3*i));
    end
    delta_2 = Jag_eqf3(1:(M),1+(i-1)*(M):i*(M))*transpose(C_eqf3(:, 3*i-2:3*i))*inv(Q_eqf3)*delta_1(:,1);
    delta_3 = [R_90_theta0_eqf3(1,1)*delta_2(1), R_90_theta0_eqf3(1,2)*delta_2(1), delta_2(2);...
               R_90_theta0_eqf3(2,1)*delta_2(1), R_90_theta0_eqf3(2,2)*delta_2(1), delta_2(3);...
               0, 0, 0];
    delta_eqf3 = -inv(P_0_eqf3)*delta_3;

    %Find LIFT
    LIFT_eqf3=V_t(1:3,3*i-2:3*i);
    
    %Eqn #1
    %The differential equation with X
    X_eqf3(1:3,3*(i+1)-2 :3*(i+1))= expm(dt*delta_eqf3)*X_eqf3(1:3,3*i-2 :3*i)*expm(dt*LIFT_eqf3);  %??
    
    %Eqn #3
    %Sigma Ricatti equation
    Jag_eqf3(1:M,1+i*M:(i+1)*M)= dt*(P_eqf3) + inv(dt*transpose(C_eqf3(:, 3*i-2:3*i))*inv(Q_eqf3)*C_eqf3(1:2*n, 3*i-2:3*i) + inv(Jag_eqf3(1:(M),1+(i-1)*(M):i*(M)))) ;
end

%Iterated robot pose
P_hat_eqf1=zeros(3,3*iter);
x_hat_eqf1=zeros(2,iter);
for i= 1:iter
    P_hat_eqf1(1:3,3*i-2:3*i)=P_0_eqf1*X_eqf1(1:3, 3*i-2 :3*i);      %??
    x_hat_eqf1(1:2,i)=P_hat_eqf1(1:2,3*i);
end

P_hat_eqf2=zeros(3,3*iter);
x_hat_eqf2=zeros(2,iter);
for i= 1:iter
    P_hat_eqf2(1:3,3*i-2:3*i)=P_0_eqf2*X_eqf2(1:3, 3*i-2 :3*i);      %??
    x_hat_eqf2(1:2,i)=P_hat_eqf2(1:2,3*i);
end

P_hat_eqf3=zeros(3,3*iter);
x_hat_eqf3=zeros(2,iter);
for i= 1:iter
    P_hat_eqf3(1:3,3*i-2:3*i)=P_0_eqf3*X_eqf3(1:3, 3*i-2 :3*i);      %??
    x_hat_eqf3(1:2,i)=P_hat_eqf3(1:2,3*i);
end

%Real positions
x_true = zeros(2, iter);
for i= 1:iter
    x_true(:, i)=  Pose(1:2, 3*i);
end

%Angles
angles_eqf1 =  zeros(iter, 3);
for i=1:iter
    R = eye(3);
    R(1:2, 1:2) = P_hat_eqf1(1:2,3*i-2:3*i-1);
    angles_eqf1(i, :) = rotm2eul(R);
end
angles_eqf2 =  zeros(iter, 3);
for i=1:iter
    R = eye(3);
    R(1:2, 1:2) = P_hat_eqf2(1:2,3*i-2:3*i-1);
    angles_eqf2(i, :) = rotm2eul(R);
end
angles_eqf3 =  zeros(iter, 3);
for i=1:iter
    R = eye(3);
    R(1:2, 1:2) = P_hat_eqf3(1:2,3*i-2:3*i-1);
    angles_eqf3(i, :) = rotm2eul(R);
end
angles_true = zeros(iter, 3);
for i=1:iter
    Rt = eye(3);
    Rt(1:2, 1:2) = Pose(1:2,3*i-2:3*i-1);
    angles_true(i, :) = rotm2eul(Rt);
end

%Save variables
name = strcat('variables_SE(2)_twoeqf.mat');
save(name)

function X = alg(x)
    X=[0 -x(1) x(2) ; x(1) 0 x(3) ; 0 0 0 ]; 
end

