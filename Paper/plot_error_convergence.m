% Plot the error evolution

load('Eqf_variables')

%%% EQF1 %%%
%Define error
ePose_eqf1 = zeros(4, 4*iter);
for i=1:iter
    ePose_eqf1(1:4,4*i-3:4*i) = Pose(1:4,4*i-3:4*i)*inv(X_eqf1(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n))*inv(P_0_eqf1)*S_corr_eqf1*P_0_eqf1;
end

ep_eqf1 = zeros(3*n, iter);
for i= 1:iter
    for k=1:n
        ep_eqf1(3*k-2:3*k,i)= p(:,k) + Pose(1:3, 4*i-3:4*i-1)*transpose(X_eqf1(1:3,(i)*(4+n)-3-n: (i)*(4+n)-n-1))*transpose(R_P0_eqf1)*S_corr_eqf1(1:3,1:3)*R_P0_eqf1*X_eqf1(1:3,(i)*(4+n)-n + k);
    end
end

%Define epsilon
epsilon_eqf1 = zeros(M,iter);
for i = 1:iter
    epsilon_eqf1(1:6, i)= unskew4(logm(inv(P_0_eqf1)*inv(S_corr_eqf1)*ePose_eqf1(1:4,4*i-3:4*i)));
    for k=1:n
        epsilon_eqf1(3*k-2:3*k,i)= ep_eqf1(3*k-2:3*k,i) - transpose(S_corr_eqf1(1:3,1:3))*(p_0_eqf1(:,k)- S_corr_eqf1(1:3,4));
    end
end

%Find norm
norm_epsilon_eqf1 = zeros(1,iter);
for i= 1:iter
   norm_epsilon_eqf1(1,i) = transpose(epsilon_eqf1(:,i))*epsilon_eqf1(:,i);
end

%Final value of the norm
fnorm_eqf1 = norm_epsilon_eqf1(iter)*ones(1,iter);


%%% EQF2 %%%
%Define error
ePose_eqf2 = zeros(4, 4*iter);
for i=1:iter
    ePose_eqf2(1:4,4*i-3:4*i) = Pose(1:4,4*i-3:4*i)*inv(X_eqf2(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n))*inv(P_0_eqf2)*S_corr_eqf2*P_0_eqf2;
end

ep_eqf2 = zeros(3*n, iter);
for i= 1:iter
    for k=1:n
        ep_eqf2(3*k-2:3*k,i)= p(:,k) + Pose(1:3, 4*i-3:4*i-1)*transpose(X_eqf2(1:3,(i)*(4+n)-3-n: (i)*(4+n)-n-1))*transpose(R_P0_eqf2)*S_corr_eqf2(1:3,1:3)*R_P0_eqf2*X_eqf2(1:3,(i)*(4+n)-n + k);
    end
end

%Define epsilon
epsilon_eqf2 = zeros(M,iter);
for i = 1:iter
    epsilon_eqf2(1:6, i)= unskew4(logm(inv(P_0_eqf2)*inv(S_corr_eqf2)*ePose_eqf2(1:4,4*i-3:4*i)));
    for k=1:n
        epsilon_eqf2(3*k-2:3*k,i)= ep_eqf2(3*k-2:3*k,i) - transpose(S_corr_eqf2(1:3,1:3))*(p_0_eqf2(:,k)- S_corr_eqf2(1:3,4));
    end
end

%Find norm
norm_epsilon_eqf2 = zeros(1,iter);
for i= 1:iter
   norm_epsilon_eqf2(1,i) = transpose(epsilon_eqf2(:,i))*epsilon_eqf2(:,i);
end

%Final value of the norm
fnorm_eqf2 = norm_epsilon_eqf2(iter)*ones(1,iter);



%Plot
plot(1:iter, norm_epsilon_eqf1 , 'DisplayName','Error EQF1')
hold on
plot(1:iter, norm_epsilon_eqf2 , 'DisplayName','Error EQF2')
title('Error Evolution')
ylabel('||eps||**2')
xlabel('Iterations')
hold off

% hold on 
% plot(1:iter, (norm_eps), 'DisplayName','Error1')

legend

function X = skew4(x)
    X=[0 -x(3) x(2) x(4); x(3) 0 -x(1) x(5); -x(2) x(1) 0 x(6);0 0 0 0]; 
end
function X = skew3(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end
function V=unskew4(P)
    V=[P(3,2); P(1,3); P(2,1); P(1,4); P(2,4); P(3,4)];
end