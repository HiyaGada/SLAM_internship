load('Eqf_variables')

%Define error terms
errPose = zeros(4, 4*iter);
for i=1:iter
    errPose(1:4,4*i-3:4*i) = Pose(1:4,4*i-3:4*i)*inv(X(1:4,(i)*(4+n)-3-n: (i)*(4+n)-n));
end

errp = zeros(3*n, iter);
for i= 1:iter
    for k=1:n
        errp(3*k-2:3*k,i)= p(:,k) + Pose(1:3, 4*i-3:4*i-1)*transpose(X(1:3,(i)*(4+n)-3-n: (i)*(4+n)-n-1))*X(1:3,(i)*(4+n)-n + k);
    end
end

%Define epsilon
eps = zeros(M,iter);
for i = 1:iter
    eps(1:6, i)= unskew4(logm(inv(P_0)*errPose(1:4,4*i-3:4*i)));
    for k=1:n
        eps(3*k-2:3*k,i)= errp(3*k-2:3*k,i) - p_0(:,k);
    end
end

%Find norm
norm_eps =zeros (1,iter);
for i= 1:iter
   norm_eps(1,i) = transpose(eps(:,i))*eps(:,i);
end

%Find error after considering S_corr
eS_corrPose = inv(S_corr)*P_0;
eS_corrp = zeros(3*n, 1);
for k=1:n
   eS_corrp(3*k-2:3*k) = p(:,k) + transpose(S_corr(1:3,1:3))*R_P0*X(1:3,(iter)*(4+n)-n + k); 
end

%Find epsilon after considering S_corr
epS_corr= zeros(M,1);
epS_corr(1:6)=unskew4(logm(inv(P_0)*eS_corrPose));

for k=1:n
    epS_corr(3*k-2:3*k) = eS_corrp(3*k-2:3*k) - p_0(:,k);
end

%Find norm of epsilon after considering S_corr
norm_epS_corr= (transpose(epS_corr)*epS_corr)*ones(1,iter);


save('error_variables')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot

plot(1:iter, abs(norm_eps - norm_epS_corr), 'DisplayName','Error')
title('Error Evolution')
ylabel('||eps||**2')
xlabel('Iterations')


function V=unskew4(P)
    V=[P(3,2); P(1,3); P(2,1); P(1,4); P(2,4); P(3,4)];
end
