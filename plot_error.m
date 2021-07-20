temp = zeros(1,100);

for b=1:100
    name= strcat('error_', string(b), '.mat');
    load(name)
    
    temp(1,b)= last_error;
end

%Take average
average= mean(temp);
av= average*ones(1,100);

%save('[5;5;0]_lownoise_type3_var');

%Plot 
plot(1:100, temp , 'DisplayName','Error')
hold on
plot(1:100, av,'DisplayName','Average Error' )
title('Error')
ylabel('||eps||**2')
xlabel('Cases')
legend
