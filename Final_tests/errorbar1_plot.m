%Number of iterations
iter=500;
dn= 0.4;
bun= (10/dn); %bun=25;

x_ax= zeros(1, bun);
for stup=1:bun
    x_ax(1,stup)= sqrt(0.01 + (stup-1)*dn);
end

y_ax1= zeros(1, bun);
y_std1=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type1_', string(stup), '.mat');
    load(naamo)
    
    y_ax1(1,stup)= average;
    y_std1(1,stup)=deviation;
end

y_ax2= zeros(1, bun);
y_std2=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type2_', string(stup), '.mat');
    load(naamo)
    
    y_ax2(1,stup)= average;
    y_std2(1,stup)=deviation;
end

y_ax3= zeros(1, bun);
y_std3=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type3_', string(stup), '.mat');
    load(naamo)
    
    y_ax3(1,stup)= average;
    y_std3(1,stup)=deviation;
end

y_ax4= zeros(1, bun);
y_std4=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type4_', string(stup), '.mat');
    load(naamo)
    
    y_ax4(1,stup)= average;
    y_std4(1,stup)=deviation;
end

y_ax5= zeros(1, bun);
y_std5=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type5_', string(stup), '.mat');
    load(naamo)
    
    y_ax5(1,stup)= average;
    y_std5(1,stup)=deviation;
end

y_ax6= zeros(1, bun);
y_std6=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type6_', string(stup), '.mat');
    load(naamo)
    
    y_ax6(1,stup)= average;
    y_std6(1,stup)=deviation;
end

y_ax7= zeros(1, bun);
y_std7=zeros(1, bun);
for stup=1:bun
    naamo= strcat('avg_type7_', string(stup), '.mat');
    load(naamo)
    
    y_ax7(1,stup)= average;
    y_std7(1,stup)=deviation;
end


% %Errorbar
% errorbar(x_ax, y_ax1, y_std1,'LineWidth',1)
% hold on
% errorbar(x_ax, y_ax2, y_std2, 'LineWidth',1)
% errorbar(x_ax, y_ax3, y_std3, 'LineWidth',1)
% errorbar(x_ax, y_ax4, y_std4, 'LineWidth',1)
% errorbar(x_ax, y_ax5, y_std5, 'LineWidth',1)
% errorbar(x_ax, y_ax6, y_std6, 'LineWidth',1)
% errorbar(x_ax, y_ax7, y_std7, 'LineWidth',1)

%2_3_6_7 on
plot(x_ax, y_ax1,'LineWidth',1)
hold on
errorbar(x_ax, y_ax2, y_std2, 'LineWidth',1)
errorbar(x_ax, y_ax3, y_std3, 'LineWidth',1)
plot(x_ax, y_ax4, 'LineWidth',1)
plot(x_ax, y_ax5, 'LineWidth',1)
errorbar(x_ax, y_ax6, y_std6, 'LineWidth',1)
 errorbar(x_ax, y_ax7, y_std7, 'LineWidth',1)

% %1_4_5 on
% errorbar(x_ax, y_ax1, y_std1,'LineWidth',1)
% hold on
% plot(x_ax, y_ax2, 'LineWidth',1)
% plot(x_ax, y_ax3, 'LineWidth',1)
% errorbar(x_ax, y_ax4, y_std4, 'LineWidth',1)
% errorbar(x_ax, y_ax5, y_std5, 'LineWidth',1)
% plot(x_ax, y_ax6, 'LineWidth',1)
% errorbar(x_ax, y_ax7, y_std7, 'LineWidth',1)

title('Error')
ylabel('average($||\epsilon_f||$)', 'Interpreter','latex')
xlabel('Measure of noise')

leg=legend('$(I_4,0)$', '$(\hat{P}(0), \hat{p}_i(0))$', '$x_P$ avg of $p_i^\circ$', '$(I_4, mean(\hat{p}_i(0)))$', '$(\hat{P}(0), mean(\hat{p}_i(0))$', '$(I_4, \hat{p}_i(0))$', '$(I_4, mean(\hat{p}_i(0))+ 10)$');
set(leg,'Interpreter','latex');
set(leg,'FontSize',13);

hold off
legend