%Number of iterations
iter=500;

%Set seed
rng('default');

%Number of landmarks and position
n=5;
px = randn(1,n);
py = randn(1,n);
pz = zeros(1,n);

%Making true p vector
for k= 1:n
    p(:,k)=[px(k);py(k);pz(k)];
end

%Real Initial pose of robot
P_in_r=[1,0,0,0.7; 0,1,0,0.5; 0,0,1,0; 0,0,0,1];
R_Pin_r= P_in_r(1:3,1:3);
xp_in_r= P_in_r(1:3,4);

%State and Output Matrix dimensions
M=6+ (3*n);
N=3*n;

%%% COMMON %%%
%Sigma 
Jag_0 = 10*eye(M);
eta = mvnrnd(zeros(1, M), Jag_0, 1); 
eta_P= skew4(eta(1:6));

%Estimated Initial pose of robot
P_in_e= P_in_r*expm(eta_P);    
for k=1:n
    p_e(:,k)= p(:,k) + transpose(eta(4 + 3*k : 6 + 3*k));
end

%%% EQF1 %%%
% Origin (\xi^{not})
P_0_eqf1= eye(4);
R_P0_eqf1= P_0_eqf1(1:3,1:3);
xp_0_eqf1= P_0_eqf1(1:3,4);

%Making origin p_0 vector
for k= 1:n
    p_0_eqf1(:,k)= [mean(p_e(1,:)) + 100; mean(p_e(2,:)) + 100; mean(p_e(3,:))];
end

%%% EQF2 %%%
% Origin (\xi^{not})
P_0_eqf2= eye(4);
R_P0_eqf2= P_0_eqf2(1:3,1:3);
xp_0_eqf2= P_0_eqf2(1:3,4);

%Making origin p_0 vector
for k= 1:n
    p_0_eqf2(:,k)= [mean(p_e(1,:)) + 50; mean(p_e(2,:)) + 50; mean(p_e(3,:))];
end

%Make V_t
vy=4;
vz=0;
ox=0;
oy=0;
oz=10;
dt=0.001;
V_t=zeros(4,4*iter);
for i=1:iter
    V_t(1:4,4*i-3:4*i)=[0,-oz,oy,-vy*sin(oz*(i-1)*dt); oz,0,-ox,vy*cos(oz*(i-1)*dt); -oy,-ox,0,vz; 0,0,0,0];
end

%Make true Pose
Pose=zeros(4,4*iter);
Pose(1:4,1:4)=P_in_r;
for i =1:iter   
    Pose(1:4,4*(i+1)-3:4*(i+1))= Pose(1:4,4*i-3:4*i)*expm(V_t(1:4,4*i-3:4*i)*dt);
end

%Making simulated measurement y_k
y=zeros(3*n,iter);
for i=1:iter
   for k=1:n
       y(3*k-2:3*k,i)=transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)));
   end
end

%Adding noise

%Std dev
Rvar=0*eye(6);
Qvar=0*eye(3*n);
R= zeros(3*n + 6);
R(1:6,1:6)= Rvar;

%Adding noise in V_t to get measured V_t_m
V_t_m=zeros(4,4*iter);
for i=1:iter
   V_t_m(1:4,4*i-3:4*i) = V_t(1:4,4*i-3:4*i) + skew4(Rvar*randn(6,1)); 
end

%Adding noise in y_k to get measured y_k_m 
y_m=zeros(3*n,iter);
for i=1:iter
    y_m(:,i)=y(:,i) + Qvar*randn(3*n,1);
end

filename = strcat('save_variables.mat');
save(filename)

% Main EQF code 

naamo = strcat('save_variables.mat');
load(naamo)

%EqF eqn



%%% EQF1 %%%
%Measure the origin output
for i= 1:n
    h_eqf1(:,i)=transpose(R_P0_eqf1)*(p_0_eqf1(:,i)-xp_0_eqf1);
end
%Making the C matrix
C_eqf1 =zeros(N, N+6);
for i= 1:n
    C_eqf1(3*i-2:3*i, 1:3)= skew3(h_eqf1(:,i));
end
for i= 1:n
    C_eqf1(3*i-2:3*i, 4:6)= -eye(3);
end
temp_eqf1= zeros(N);
for i=1:n
    temp_eqf1(3*i-2:3*i,3*i-2:3*i)=transpose(R_P0_eqf1);
end
C_eqf1(:,7:N+6)=temp_eqf1;


%%% EQF2 %%%
%Measure the origin output
for i= 1:n
    h_eqf2(:,i)=transpose(R_P0_eqf2)*(p_0_eqf2(:,i)-xp_0_eqf2);
end
%Making the C matrix
C_eqf2 =zeros(N, N+6);
for i= 1:n
    C_eqf2(3*i-2:3*i, 1:3)= skew3(h_eqf2(:,i));
end
for i= 1:n
    C_eqf2(3*i-2:3*i, 4:6)= -eye(3);
end
temp_eqf2= zeros(N);
for i=1:n
    temp_eqf2(3*i-2:3*i,3*i-2:3*i)=transpose(R_P0_eqf2);
end
C_eqf2(:,7:N+6)=temp_eqf2;



%%% EQF1 %%%
%Loop for X
X_eqf1(1:4+n, 1:4+n)=eye(4+n); 
X_eqf1(1:4,1:4)= inv(P_0_eqf1)*P_in_e;
for k=1:n
   X_eqf1(1:3, 4+k)= inv(R_P0_eqf1)*(p_e(:,k)- p_0_eqf1(:,k)); 
end

%%% EQF2 %%%
%Loop for X
X_eqf2(1:4+n, 1:4+n)=eye(4+n); 
X_eqf2(1:4,1:4)= inv(P_0_eqf2)*P_in_e;
for k=1:n
   X_eqf2(1:3, 4+k)= inv(R_P0_eqf2)*(p_e(:,k)- p_0_eqf2(:,k)); 
end

% Z matrix 
Z = X_eqf1(1:4+n, 1:4+n)*inv(X_eqf2(1:4+n, 1:4+n));

% M matrix (M_trans)
M_trans = eye(M); 
M_trans(1:3, 1:3) = transpose(Z(1:3,1:3));
M_trans(4:6, 4:6) = transpose(Z(1:3,1:3));
M_trans(4:6, 1:3) = -skew3(transpose(Z(1:3,1:3))*Z(1:3, 4))*transpose(Z(1:3,1:3));
% Loop for column 1
for k= 1:n
    M_trans(3*k + 4: 3*k + 6, 1:3)= -R_P0_eqf1*skew3(M_trans(1:3, 4+k)) ;
end

%%% EQF1 %%%
%State and Output matrix
P_eqf1= 0.1*eye(M);
Q_eqf1= 0.1*eye(N);
A= zeros(M);

%%% EQF2 %%%
%State and Output matrix
P_eqf2= M_trans*P_eqf1*transpose(M_trans);
Q_eqf2= Q_eqf1;
%A= zeros(M);


%%% EQF1 %%%
%T matrix for transforming Jag_0
T_eqf1= eye(M);
T_eqf1(1:3, 1:3)= X_eqf1(1:3,1:3);
T_eqf1(4:6, 4:6)= X_eqf1(1:3,1:3);
T_eqf1(4:6, 1:3)= skew3(X_eqf1(1:3,4))*X_eqf1(1:3,1:3);
for k= 1:n
    T_eqf1(3*k + 4: 3*k + 6, 1:3)= R_P0_eqf1*skew3(X_eqf1(1:3, 4+k))*X_eqf1(1:3,1:3) ;
end

%%% EQF2 %%%
%T matrix for transforming Jag_0
T_eqf2= eye(M);
T_eqf2(1:3, 1:3)= X_eqf2(1:3,1:3);
T_eqf2(4:6, 4:6)= X_eqf2(1:3,1:3);
T_eqf2(4:6, 1:3)= skew3(X_eqf2(1:3,4))*X_eqf2(1:3,1:3);
for k= 1:n
    T_eqf2(3*k + 4: 3*k + 6, 1:3)= R_P0_eqf2*skew3(X_eqf2(1:3, 4+k))*X_eqf2(1:3,1:3) ;
end


%Initialise Sigma Ricatti eqn

%%% USE M HERE CONFIRM %%%

%%% EQF1 %%%
Jag_eqf1=zeros(N+6,(iter+1)*(N+6));
Jag_eqf1(1:(N+6),1:(N+6))=T_eqf1*Jag_0*transpose(T_eqf1); 

%%% EQF2 %%%
Jag_eqf2=zeros(N+6,(iter+1)*(N+6));
Jag_eqf2(1:(N+6),1:(N+6))=M_trans*T_eqf2*Jag_0*transpose(T_eqf2)*transpose(M_trans); %%% DOUBT!!! %%%
%Jag_eqf2(1:(N+6),1:(N+6))=M_trans*Jag_eqf1(1:(N+6),1:(N+6))*transpose(M_trans);


%%% EQF1 %%%
%Delta equation part
H_eqf1= zeros(N+6);
H_eqf1(1:6,1:6)= eye(6);
for i=1:n
    H_eqf1(4+(i*3):6+(i*3),4+(i*3):6+(i*3))=transpose(R_P0_eqf1);
end

for i=1:iter
    %Find y_k
    %y_k=transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)));
    %Find delta
    for k=1:n
        delta_eqf1(k*3-2:k*3,1)=X_eqf1(1:3,(i)*(4+n)-3-n :(i)*(4+n)-1-n)*(y_m(3*k-2:3*k,i)) - h_eqf1(:,k)+ (X_eqf1(1:3,(i)*(4+n)-n)- X_eqf1(1:3,(i)*(4+n)-n+k));
    end
    %Find Dlta
    Dlta_eqf1=H_eqf1*Jag_eqf1(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C_eqf1)*inv(Q_eqf1)*(delta_eqf1);
    %Find DLTA
    DLTA_eqf1=zeros(4+n);
    DLTA_eqf1(1:3,1:3)=skew3(Dlta_eqf1(1:3));
    DLTA_eqf1(1:3,4)=Dlta_eqf1(4:6);
    for j=1:n
        DLTA_eqf1(1:3,4+j)=Dlta_eqf1(3*j+4:3*j+6);
    end
    %Find LIFT
    LIFT_eqf1=zeros(4+n);
    LIFT_eqf1(1:4,1:4)=V_t_m(1:4,4*i-3:4*i);
    %The differential equation
    X_eqf1(1:4+n,(i+1)*(4+n)-3-n :(i+1)*(4+n))= expm(dt*DLTA_eqf1)*X_eqf1(1:4+n,(i)*(4+n)-3-n :(i)*(4+n))*expm(dt*LIFT_eqf1);
    %B matrix
    B_eqf1 = zeros(M);
    for l=1:n
        B_eqf1(4+3*l:6+3*l,4+3*l:6+3*l)= R_P0_eqf1*X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
        B_eqf1(4+3*l:6+3*l,1:3)=R_P0_eqf1*X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n)*skew3(transpose(X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n)*X_eqf1(1:3,(i)*(4+n)-n + l)));
    end
    B_eqf1(1:3,1:3)= X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    B_eqf1(4:6,4:6)= X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    B_eqf1(4:6,1:3)= skew3(X_eqf1(1:3,(i)*(4+n)-n))*X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    %Sigma Ricatti equation
    for i=1:iter
        Jag_eqf1(1:(N+6),1+i*(N+6):(i+1)*(N+6))= dt*(B_eqf1*R*transpose(B_eqf1) + P_eqf1) + inv(dt*transpose(C_eqf1)*inv(Q_eqf1)*C_eqf1 + inv(Jag_eqf1(1:(N+6),1+(i-1)*(N+6):i*(N+6)))) ;
    end
    %for i=1:iter
    %    Jag(1:(N+6),1+i*(N+6):(i+1)*(N+6))=Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6)) + dt*(B*R*transpose(B) + P_- (Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*C*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))));
    %end 
end


%%% EQF2 %%%
%Delta equation part
H_eqf2= zeros(N+6);
H_eqf2(1:6,1:6)= eye(6);
for i=1:n
    H_eqf2(4+(i*3):6+(i*3),4+(i*3):6+(i*3))=transpose(R_P0_eqf2);
end

for i=1:iter
    %Find y_k
    %y_k=transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)));
    %Find delta
    for k=1:n
        delta_eqf2(k*3-2:k*3,1)=X_eqf2(1:3,(i)*(4+n)-3-n :(i)*(4+n)-1-n)*(y_m(3*k-2:3*k,i)) - h_eqf2(:,k)+ (X_eqf2(1:3,(i)*(4+n)-n)- X_eqf2(1:3,(i)*(4+n)-n+k));
    end
    %Find Dlta
    Dlta_eqf2=H_eqf2*Jag_eqf2(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C_eqf2)*inv(Q_eqf2)*(delta_eqf2);
    %Find DLTA
    DLTA_eqf2=zeros(4+n);
    DLTA_eqf2(1:3,1:3)=skew3(Dlta_eqf2(1:3));
    DLTA_eqf2(1:3,4)=Dlta_eqf2(4:6);
    for j=1:n
        DLTA_eqf2(1:3,4+j)=Dlta_eqf2(3*j+4:3*j+6);
    end
    %Find LIFT
    LIFT_eqf2=zeros(4+n);
    LIFT_eqf2(1:4,1:4)=V_t_m(1:4,4*i-3:4*i);
    %The differential equation
    X_eqf2(1:4+n,(i+1)*(4+n)-3-n :(i+1)*(4+n))= expm(dt*DLTA_eqf2)*X_eqf2(1:4+n,(i)*(4+n)-3-n :(i)*(4+n))*expm(dt*LIFT_eqf2);
    %B matrix
    B_eqf2 = zeros(M);
    for l=1:n
        B_eqf2(4+3*l:6+3*l,4+3*l:6+3*l)= R_P0_eqf2*X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
        B_eqf2(4+3*l:6+3*l,1:3)=R_P0_eqf2*X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n)*skew3(transpose(X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n)*X_eqf2(1:3,(i)*(4+n)-n + l)));
    end
    B_eqf2(1:3,1:3)= X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    B_eqf2(4:6,4:6)= X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    B_eqf2(4:6,1:3)= skew3(X_eqf2(1:3,(i)*(4+n)-n))*X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    %Sigma Ricatti equation
    for i=1:iter
        Jag_eqf2(1:(N+6),1+i*(N+6):(i+1)*(N+6))= dt*(B_eqf2*R*transpose(B_eqf2) + P_eqf2) + inv(dt*transpose(C_eqf2)*inv(Q_eqf2)*C_eqf2 + inv(Jag_eqf2(1:(N+6),1+(i-1)*(N+6):i*(N+6)))) ;
    end
    %for i=1:iter
    %    Jag(1:(N+6),1+i*(N+6):(i+1)*(N+6))=Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6)) + dt*(B*R*transpose(B) + P_- (Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*C*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))));
    %end 
end


%%% EQF1 %%%
%Correction/Allignment factor
S_corr_eqf1=P_0_eqf1*X_eqf1(1:4,(iter)*(4+n)-3-n :(iter)*(4+n)-n)*inv(Pose(1:4,4*iter-3:4*iter));

%Iterated robot pose
P_hat_eqf1=zeros(4,4*iter);
x_hat_eqf1=zeros(3,iter);
for i= 1:iter
    P_hat_eqf1(1:4,4*i-3:4*i)=inv(S_corr_eqf1)*P_0_eqf1*X_eqf1(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n);
    x_hat_eqf1(1:3,i)=P_hat_eqf1(1:3,4*i);
end

%Iterated landmarks
p_hat_eqf1=zeros(3*n,iter);
for i=1:iter
    for k=1:n
        p_hat_eqf1(3*k-2:3*k,i)=p_0_eqf1(:,k)+(R_P0_eqf1*X_eqf1(1:3,(i)*(4+n)-n+k));
        %p_hat(3*k-2:3*k,i)=inv(S_corr)*p_hat(3*k-2:3*k,i);
        p_hat_eqf1(3*k-2:3*k,i)=transpose(S_corr_eqf1(1:3,1:3))*(p_hat_eqf1(3*k-2:3*k,i)- S_corr_eqf1(1:3,4));
    end
end

%%% EQF1 %%%
%Correction/Allignment factor
S_corr_eqf1=P_0_eqf1*X_eqf1(1:4,(iter)*(4+n)-3-n :(iter)*(4+n)-n)*inv(Pose(1:4,4*iter-3:4*iter));

%Iterated robot pose
P_hat_eqf1=zeros(4,4*iter);
x_hat_eqf1=zeros(3,iter);
for i= 1:iter
    P_hat_eqf1(1:4,4*i-3:4*i)=inv(S_corr_eqf1)*P_0_eqf1*X_eqf1(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n);
    x_hat_eqf1(1:3,i)=P_hat_eqf1(1:3,4*i);
end

%Iterated landmarks
p_hat_eqf1=zeros(3*n,iter);
for i=1:iter
    for k=1:n
        p_hat_eqf1(3*k-2:3*k,i)=p_0_eqf1(:,k)+(R_P0_eqf1*X_eqf1(1:3,(i)*(4+n)-n+k));
        %p_hat(3*k-2:3*k,i)=inv(S_corr)*p_hat(3*k-2:3*k,i);
        p_hat_eqf1(3*k-2:3*k,i)=transpose(S_corr_eqf1(1:3,1:3))*(p_hat_eqf1(3*k-2:3*k,i)- S_corr_eqf1(1:3,4));
    end
end


%%% EQF2 %%%
%Correction/Allignment factor
S_corr_eqf2=P_0_eqf2*X_eqf2(1:4,(iter)*(4+n)-3-n :(iter)*(4+n)-n)*inv(Pose(1:4,4*iter-3:4*iter));

%Iterated robot pose
P_hat_eqf2=zeros(4,4*iter);
x_hat_eqf2=zeros(3,iter);
for i= 1:iter
    P_hat_eqf2(1:4,4*i-3:4*i)=inv(S_corr_eqf2)*P_0_eqf2*X_eqf2(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n);
    x_hat_eqf2(1:3,i)=P_hat_eqf2(1:3,4*i);
end

%Iterated landmarks
p_hat_eqf2=zeros(3*n,iter);
for i=1:iter
    for k=1:n
        p_hat_eqf2(3*k-2:3*k,i)=p_0_eqf2(:,k)+(R_P0_eqf2*X_eqf2(1:3,(i)*(4+n)-n+k));
        %p_hat(3*k-2:3*k,i)=inv(S_corr)*p_hat(3*k-2:3*k,i);
        p_hat_eqf2(3*k-2:3*k,i)=transpose(S_corr_eqf2(1:3,1:3))*(p_hat_eqf2(3*k-2:3*k,i)- S_corr_eqf2(1:3,4));
    end
end

name = strcat('EqF_variables.mat');
save(name)






function X = skew4(x)
    X=[0 -x(3) x(2) x(4); x(3) 0 -x(1) x(5); -x(2) x(1) 0 x(6);0 0 0 0]; 
end
function X = skew3(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end
function V=unskew4(P)
    V=[P(3,2); P(1,3); P(2,1); P(1,4); P(2,4); P(3,4)];
end


