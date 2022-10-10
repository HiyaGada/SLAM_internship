load('variables_SE(2)_twoeqf.mat')

%True position of robot
x_ttrue = zeros(2, iter);
for i=1:iter
    x_ttrue(:, i)=Pose(1:2,3*i);
end

%Find norm with real position of robot
norm_iter_eqf1 = zeros(iter);
for i = 1:iter
    norm_iter_eqf1(i)= norm(x_hat_eqf1(:, i) - x_ttrue(:, i));
end
norm_iter_eqf2 = zeros(iter);
for i = 1:iter
    norm_iter_eqf2(i)= norm(x_hat_eqf2(:, i) - x_ttrue(:, i));
end
norm_iter_eqf3 = zeros(iter);
for i = 1:iter
    norm_iter_eqf3(i)= norm(x_hat_eqf3(:, i) - x_ttrue(:, i));
end

figure;


% Plot position norm
subplot(2,1,1);
lg1 = plot(dt*(0:iter-1), norm_iter_eqf1(1:iter), 'Color','blue','LineStyle','--','DisplayName','EQF 1');
hold on
lg2 = plot(dt*(0:iter-1), norm_iter_eqf2(1:iter), 'Color','black','LineStyle',':','DisplayName','EQF 2');
lg2 = plot(dt*(0:iter-1), norm_iter_eqf3(1:iter), 'Color','magenta','LineStyle','-','DisplayName','EQF 3');
hold off
set(gca, 'YScale', 'log')
%lg(1).LineWidth = 1;
grid on
xlabel('time (s)')
ylabel('|x - x_{true}|(m)')
%legend


%Angle error norm
angle_norm_eqf1 = zeros(1,iter);
for i=1:iter
    angle_norm_eqf1(1, i) =norm(angles_eqf1(i, 1) - angles_true(i, 1))*180/pi;
end
angle_norm_eqf2 = zeros(1,iter);
for i=1:iter
    angle_norm_eqf2(1, i) =norm(angles_eqf2(i, 1) - angles_true(i, 1))*180/pi;
end
angle_norm_eqf3 = zeros(1,iter);
for i=1:iter
    angle_norm_eqf3(1, i) =norm(angles_eqf3(i, 1) - angles_true(i, 1))*180/pi;
end
subplot(2,1,2);
plot((0:iter-1)*dt, angle_norm_eqf1(1,:),'Color','blue','LineStyle','--','DisplayName','EQF 1');
hold on
plot((0:iter-1)*dt, angle_norm_eqf2(1,:),'Color','black','LineStyle',':','DisplayName','EQF 2');
plot((0:iter-1)*dt, angle_norm_eqf3(1,:),'Color','magenta','LineStyle','-','DisplayName','EQF 3');
hold off
ylabel('|\theta - \theta_{true}|(deg)')
xlabel('time (s)')
set(gca, 'YScale', 'log')
grid on
legend

sgtitle("Position and Angle Error")

function fix_dottedline(filename)
% Fix the way that the default dotted line and dash-dot lines look terrible
% when exported as eps
fid = fopen(filename,'r');
tempfile = tempname;
outfid = fopen(tempfile,'w');
repeat = 1;
while repeat==1
    thisLine = fgetl(fid);
    iStart = strfind(thisLine,'/DO { [.5');
    if iStart
        thisLine(iStart+7:iStart+8) = '03';
    end
    iStart = strfind(thisLine,'/DD { [.5');
    if iStart
        thisLine(iStart+7:iStart+9) = '1.5';
        thisLine(iStart+10:end+1) = [' ' thisLine(iStart+10:end)];
    end
    if ~ischar(thisLine)
        repeat = 0; 
    else
        fprintf(outfid,'%s\n',thisLine);
    end
end

fclose(fid);
fclose(outfid);
copyfile(tempfile, filename);
delete(tempfile);
end
