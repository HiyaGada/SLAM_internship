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

save('error_variables')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot

plot(1:iter, norm_eps)

function V=unskew4(P)
    V=[P(3,2); P(1,3); P(2,1); P(1,4); P(2,4); P(3,4)];
end
