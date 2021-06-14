load('Eqf_variables')

%load('error_variables')

%Define error
ePose = zeros(4, 4*iter);
for i=1:iter
    ePose(1:4,4*i-3:4*i) = Pose(1:4,4*i-3:4*i)*inv(X(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n))*inv(P_0)*S_corr*P_0;
end

ep = zeros(3*n, iter);
for i= 1:iter
    for k=1:n
        ep(3*k-2:3*k,i)= p(:,k) + Pose(1:3, 4*i-3:4*i-1)*transpose(X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-n-1))*transpose(R_P0)*S_corr(1:3,1:3)*R_P0*X(1:3,(i)*(4+n)-n + k);
    end
end

%Define epsilon
epsilon = zeros(M,iter);
for i = 1:iter
    epsilon(1:6, i)= unskew4(logm(inv(P_0)*inv(S_corr)*ePose(1:4,4*i-3:4*i)));
    for k=1:n
        epsilon(3*k-2:3*k,i)= ep(3*k-2:3*k,i) - transpose(S_corr(1:3,1:3))*(p_0(:,k)- S_corr(1:3,4));
    end
end

%Find norm
norm_epsilon =zeros (1,iter);
for i= 1:iter
   norm_epsilon(1,i) = transpose(epsilon(:,i))*epsilon(:,i);
end

%Final value of the norm
fnorm = norm_epsilon(iter)*ones(1,iter);

save('error2')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot

plot(1:iter, norm_epsilon , 'DisplayName','Error2')
title('Error Evolution')
ylabel('||eps||**2')
xlabel('Iterations')

% hold on 
% plot(1:iter, (norm_eps), 'DisplayName','Error1')

legend

function V=unskew4(P)
    V=[P(3,2); P(1,3); P(2,1); P(1,4); P(2,4); P(3,4)];
end


