%Number of iterations
iter=500;
dn= 0.4;
bun= (10/dn); %bun=25;

for stup=1:bun
    for b=1:20
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

        %Sigma 
        Jag_0 = 10*eye(M);
        eta = mvnrnd(zeros(1, M), Jag_0, 1); 
        eta_P= skew4(eta(1:6));

        %Estimated Initial pose of robot
        P_in_e= P_in_r*expm(eta_P);    
        for k=1:n
            p_e(:,k)= p(:,k) + transpose(eta(4 + 3*k : 6 + 3*k));
        end


        % Origin (\xi^{not})
        P_0= eye(4);
        R_P0= P_0(1:3,1:3);
        xp_0= P_0(1:3,4);

        %Making origin p_0 vector
        for k= 1:n
            p_0(:,k)= [mean(p_e(1,:)) + 100; mean(p_e(2,:)) + 100; mean(p_e(3,:))];
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
        Rvar=sqrt(0.01 + ((stup-1)*dn))*eye(6);
        Qvar=sqrt(0.01 + ((stup-1)*dn))*eye(3*n);
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

        filename = strcat('save_variables_',string(b),'_', string(stup),'.mat');
        save(filename)  

        disp(stup);
    end
end

function X = skew4(x)
    X=[0 -x(3) x(2) x(4); x(3) 0 -x(1) x(5); -x(2) x(1) 0 x(6);0 0 0 0]; 
end
function X = skew3(x)
    X=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ]; 
end