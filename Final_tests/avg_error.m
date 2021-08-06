%Number of iterations
iter=500;
dn= 0.4;
bun= (10/dn); %bun=25;

for stup=1:bun    
    tmp = zeros(1,20);

    for b=1:20
        name= strcat('error_', string(b),'_',string(stup), '.mat');
        load(name)

        tmp(1,b)= last_error;
    end

    %Take average
    average= mean(tmp);
    %av= average*ones(1,20);
    deviation= std(tmp);
    %dev= deviation*ones(1,20);
    
    naamo= strcat('avg_type8_', string(stup), '.mat');
    save(naamo, 'average', 'deviation')
end