%Number of landmarks
n=5;
px = rand(1,n);
py = rand(1,n);
pz = zeros(1,n);

%State and Output Matrix dimensions
M=6+ (3*n);
N=3*n;

%Riccati, state and output matrix
Jag_0= eye(M);
P_= 0.1*eye(M);
Q_= 0.1*eye(N);
A= zeros(M);

%Initialise pose of robot
P_0= [1,0,0,0.7; 0,1,0,0.5; 0,0,1,0; 0,0,0,1];
R_P0= P_0(1:3,1:3);
xp_0= [0.7;0.5;0];

%Making origin p_0 vector
for i= 1:n
    p_0(:,i)=[0.5;0.5;0];
end

%Making true p vector
for i= 1:n
    p(:,i)=[px(i);py(i);pz(i)];
end

%Measure the output
for i= 1:n
    h(:,i)=transpose(R_P0)*(p_0(:,i)-xp_0);
end


%Making the C matrix
C= zeros(N, N+6);

for i= 1:n
    C(3*i-2:3*i, 1:3)= skew(h(:,i));
end

for i= 1:n
    C(3*i-2:3*i, 4:6)= -eye(3);
end

temp= zeros(N);
for i=1:n
    temp(3*i-2:3*i,3*i-2:3*i)=transpose(R_P0);
end
C(:,7:N+6)=temp;


function X = skew(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end
