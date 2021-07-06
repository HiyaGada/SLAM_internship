load('simulated_variables1')

%Setting seed as 0 for random number generator
rng(0)

%Std dev
Rvar=sqrt(0.01)*eye(6);
Qvar=sqrt(0.01)*eye(3*n);
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

%save('noise_variables')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%EqF eqn

%State and Output Matrix dimensions
M=6+ (3*n);
N=3*n;

%Riccati, state and output matrix
Jag_0= eye(M);
P_= 0.1*eye(M);
Q_= 0.1*eye(N);
A= zeros(M);


%Making origin p_0 vector
for k= 1:n
    p_0(:,k)=[0.5;0.5;0];
end

%Measure the origin output
for i= 1:n
    h(:,i)=transpose(R_P0)*(p_0(:,i)-xp_0);
end

%Making the C matrix
C=zeros(N, N+6);
for i= 1:n
    C(3*i-2:3*i, 1:3)= skew3(h(:,i));
end
for i= 1:n
    C(3*i-2:3*i, 4:6)= -eye(3);
end
temp= zeros(N);
for i=1:n
    temp(3*i-2:3*i,3*i-2:3*i)=transpose(R_P0);
end
C(:,7:N+6)=temp;

%Initialise Sigma Ricatti eqn
Jag=zeros(N+6,(iter+1)*(N+6));
Jag(1:(N+6),1:(N+6))=Jag_0;


%Delta equation part
H= zeros(N+6);
H(1:6,1:6)= eye(6);
for i=1:n
    H(4+(i*3):6+(i*3),4+(i*3):6+(i*3))=transpose(R_P0);
end

%Loop for X
X(1:4+n, 1:4+n)=eye(4+n);
for i=1:iter
    %Find y_k
    %y_k=transpose(Pose(1:3, 4*i-3:4*i-1))*(p(:,k)-(Pose(1:3,4*i)));
    %Find delta
    for k=1:n
        delta(k*3-2:k*3,1)=X(1:3,(i)*(4+n)-3-n :(i)*(4+n)-1-n)*(y_m(3*k-2:3*k,i)) - h(:,k)+ (X(1:3,(i)*(4+n)-n)- X(1:3,(i)*(4+n)-n+k));
    end
    %Find Dlta
    Dlta=H*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*(delta);
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
        Jag(1:(N+6),1+i*(N+6):(i+1)*(N+6))=Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6)) + dt*(B*R*transpose(B) + P_- (Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))*transpose(C)*inv(Q_)*C*Jag(1:(N+6),1+(i-1)*(N+6):i*(N+6))));
    end 
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


save('EqF_variables1')

function X = skew4(x)
    X=[0 -x(3) x(2) x(4); x(3) 0 -x(1) x(5); -x(2) x(1) 0 x(6);0 0 0 0]; 
end
function X = skew3(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end