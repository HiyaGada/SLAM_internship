load('Eqf_variables3')

%Define error
ePose = zeros(4, 4*iter);
for i=1:iter
    ePose(1:4,4*i-3:4*i) = Pose(1:4,4*i-3:4*i)*inv(inv(P_0)*inv(S_corr)*P_0*X(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n));
end

ep = zeros(3*n, iter);
for i= 1:iter
    Anew = inv(P_0)*inv(S_corr)*P_0*X(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n);
    for k=1:n
        anewk= transpose(R_P0)*transpose(S_corr(1:3,1:3))*( p_0(:,k) + R_P0*X(1:3,(i)*(4+n)-n + k) - S_corr(1:3,4) ) - transpose(R_P0)*p_0(:,k);
        ep(3*k-2:3*k,i)= p(:,k) - Pose(1:3, 4*i-3:4*i-1)*transpose(Anew(1:3,1:3))*anewk;
    end
end

%Define epsilon
epsilon = zeros(M,iter);
for i = 1:iter
    epsilon(1:6, i)= unskew4(logm(inv(P_0)*ePose(1:4,4*i-3:4*i)));
    for k=1:n
        epsilon(3*k-2:3*k,i)= ep(3*k-2:3*k,i) - p_0(:,k);
    end
end

%Find norm
norm_epsilon3 =zeros (1,iter);
for i= 1:iter
   norm_epsilon3(1,i) = transpose(epsilon(:,i))*epsilon(:,i);
end

save('error3', 'norm_epsilon3')

% %Plot
% plot(1:iter, norm_epsilon3 , 'DisplayName','Error4')
% title('Error Evolution')
% ylabel('||eps||**2')
% xlabel('Iterations')
% legend

function V=unskew4(P)
    V=[P(3,2); P(1,3); P(2,1); P(1,4); P(2,4); P(3,4)];
end
