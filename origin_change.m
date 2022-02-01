%Number of iterations
iter=500;

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

%M matrix (M_trans)
M_trans = zeros(M); %%% CHANGE after CONFIRMING Math derivation for M_trans %%%


%%% EQF1 %%%
%Sigma 
Jag_0_eqf1 = 10*eye(M);
eta_eqf1 = mvnrnd(zeros(1, M), Jag_0_eqf1, 1); 
eta_P_eqf1= skew4(eta_eqf1(1:6));

%%% EQF2 %%%
%Sigma 
Jag_0_eqf2 = M_trans*Jag_0_eqf1*transpose(M_trans); 
eta_eqf2 = mvnrnd(zeros(1, M), Jag_0_eqf2, 1); 
eta_P_eqf2= skew4(eta_eqf2(1:6));

%%% EQF1 %%%
%Estimated Initial pose of robot
P_in_e_eqf1= P_in_r*expm(eta_P_eqf1);    
for k=1:n
    p_e_eqf1(:,k)= p(:,k) + transpose(eta_eqf1(4 + 3*k : 6 + 3*k));
end

%%% EQF2 %%%
%Estimated Initial pose of robot
P_in_e_eqf2= P_in_r*expm(eta_P_eqf2);    
for k=1:n
    p_e_eqf2(:,k)= p(:,k) + transpose(eta_eqf2(4 + 3*k : 6 + 3*k));
end

%%% EQF1 %%%
% Origin (\xi^{not})
P_0_eqf1= eye(4);
R_P0_eqf1= P_0_eqf1(1:3,1:3);
xp_0_eqf1= P_0_eqf1(1:3,4);

%Making origin p_0 vector
for k= 1:n
    p_0_eqf1(:,k)= [mean(p_e_eqf1(1,:)) + 100; mean(p_e_eqf1(2,:)) + 100; mean(p_e_eqf1(3,:))];
end

%%% EQF2 %%%
% Origin (\xi^{not})
P_0_eqf2= eye(4);
R_P0_eqf2= P_0_eqf2(1:3,1:3);
xp_0_eqf2= P_0_eqf2(1:3,4);

%Making origin p_0 vector
for k= 1:n
    p_0_eqf2(:,k)= [mean(p_e_eqf2(1,:)) + 100; mean(p_e_eqf2(2,:)) + 100; mean(p_e_eqf2(3,:))];
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Adding noise

%Std dev
Rvar=0*sqrt(0.01 + ((stup-1)*dn))*eye(6);
Qvar=0*sqrt(0.01 + ((stup-1)*dn))*eye(3*n);
R= zeros(3*n + 6);
R(1:6,1:6)= Rvar;

%Adding noise in V_t to get measured V_t_m
V_t_m=zeros(4,4*iter);
for i=1:iter
   V_t_m(1:4,4*i-3:4*i) = V_t(1:4,4*i-3:4*i) + skew4(Rvar*randn(6,1)); 
end

%Adding noise in y_k to get measured y_k_m %%% DOUBT %%%
y_m=zeros(3*n,iter);
for i=1:iter
    y_m(:,i)=y(:,i) + Qvar*randn(3*n,1);
end

filename = strcat('save_variables.mat');
save(filename)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Main EQF code CHANGE FROM HERE

naamo = strcat('save_variables.mat');
load(naamo)

%EqF eqn

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
C_eqf2(:,7:N+6)=temp_eqf;


%Loop for X
X(1:4+n, 1:4+n)=eye(4+n); 
X(1:4,1:4)= inv(P_0)*P_in_e;
for k=1:n
   X(1:3, 4+k)= inv(R_P0)*(p_e(:,k)- p_0(:,k)); 
end

%T matrix for transforming Jag_0
T= eye(M);
T(1:3, 1:3)= X(1:3,1:3);
T(4:6, 4:6)= X(1:3,1:3);
T(4:6, 1:3)= skew3(X(1:3,4))*X(1:3,1:3);
for k= 1:n
    T(3*k + 4: 3*k + 6, 1:3)= R_P0*skew3(X(1:3, 4+k))*X(1:3,1:3) ;
end



%Initialise Sigma Ricatti eqn
Jag=zeros(N+6,(iter+1)*(N+6));
Jag(1:(N+6),1:(N+6))=T*Jag_0*transpose(T); 

%Delta equation part
H= zeros(N+6);
H(1:6,1:6)= eye(6);
for i=1:n
    H(4+(i*3):6+(i*3),4+(i*3):6+(i*3))=transpose(R_P0);
end

for i=1:iter
    %Find y_k
    %y_k=transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)));
    %Find delta
    for k=1:n
        delta(k*3-2:k*3,1)=X(1:3,(i)*(4+n)-3-n :(i)*(4+n)-1-n)*(y_m(3*k-2:3*k,i)) - h(:,k)+ (X(1:3,(i)*(4+n)-n)- X(1:3,(i)*(4+n)-n+k));
    end
    %Find Dlta
    Dlta=H*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C_eqf1)*inv(Q_)*(delta);
    %Find DLTA
    DLTA=zeros(4+n);
    DLTA(1:3,1:3)=skew3(Dlta(1:3));
    DLTA(1:3,4)=Dlta(4:6);
    for j=1:n
        DLTA(1:3,4+j)=Dlta(3*j+4:3*j+6);
    end
    %Find LIFT
    LIFT=zeros(4+n);
    LIFT(1:4,1:4)=V_t_m(1:4,4*i-3:4*i);
    %The differential equation
    X(1:4+n,(i+1)*(4+n)-3-n :(i+1)*(4+n))= expm(dt*DLTA)*X(1:4+n,(i)*(4+n)-3-n :(i)*(4+n))*expm(dt*LIFT);
    %B matrix
    B = zeros(M);
    for l=1:n
        B(4+3*l:6+3*l,4+3*l:6+3*l)= R_P0*X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
        B(4+3*l:6+3*l,1:3)=R_P0*X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n)*skew3(transpose(X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n)*X(1:3,(i)*(4+n)-n + l)));
    end
    B(1:3,1:3)= X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    B(4:6,4:6)= X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    B(4:6,1:3)= skew3(X(1:3,(i)*(4+n)-n))*X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-1-n);
    %Sigma Ricatti equation
    for i=1:iter
        Jag(1:(N+6),1+i*(N+6):(i+1)*(N+6))= dt*(B*R*transpose(B) + P_) + inv(dt*transpose(C_eqf1)*inv(Q_)*C_eqf1 + inv(Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6)))) ;
    end
    %for i=1:iter
    %    Jag(1:(N+6),1+i*(N+6):(i+1)*(N+6))=Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6)) + dt*(B*R*transpose(B) + P_- (Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*C*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))));
    %end 
end

%Correction/Allignment factor
S_corr=P_0*X(1:4,(iter)*(4+n)-3-n :(iter)*(4+n)-n)*inv(Pose(1:4,4*iter-3:4*iter));

%Iterated robot pose
P_hat=zeros(4,4*iter);
x_hat=zeros(3,iter);
for i= 1:iter
    P_hat(1:4,4*i-3:4*i)=inv(S_corr)*P_0*X(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n);
    x_hat(1:3,i)=P_hat(1:3,4*i);
end

%Iterated landmarks
p_hat=zeros(3*n,iter);
for i=1:iter
    for k=1:n
        p_hat(3*k-2:3*k,i)=p_0(:,k)+(R_P0*X(1:3,(i)*(4+n)-n+k));
        %p_hat(3*k-2:3*k,i)=inv(S_corr)*p_hat(3*k-2:3*k,i);
        p_hat(3*k-2:3*k,i)=transpose(S_corr(1:3,1:3))*(p_hat(3*k-2:3*k,i)- S_corr(1:3,4));
    end
end

name = strcat('EqF_variables_', string(b),'_', string(stup), '.mat');
save(name)

%disp(stup)


function X = skew4(x)
    X=[0 -x(3) x(2) x(4); x(3) 0 -x(1) x(5); -x(2) x(1) 0 x(6);0 0 0 0]; 
end
function X = skew3(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end