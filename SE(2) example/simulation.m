load('variables_SE(2)_twoeqf.mat')

hold on

% %Plot true position of robot
% for i=1:iter-1
%     x_ttrue=Pose(1:2,3*i);
%     plot(x_ttrue(1),x_ttrue(2), 'Color','red','LineStyle','-.', 'HandleVisibility','off')
%     pause(.001)
% end


% %Plot iterated position of robot
% for i= 1:iter-1
%     plot(x_hat_eqf1(1,i),x_hat_eqf1(2,i),'Color','blue','LineStyle','--','HandleVisibility','off')
%     pause(.001)
% end
plot(x_hat_eqf1(1,1:iter-1), x_hat_eqf1(2,1:iter-1),'Color','blue','LineStyle','--', 'HandleVisibility','off')

scatter(x_hat_eqf1(1, 1),x_hat_eqf1(2, 1),'d', 'blue','HandleVisibility','off')
scatter(x_hat_eqf1(1,iter),x_hat_eqf1(2,iter),'d','blue','DisplayName','EQF1 start and end position')

% %Plot iterated position of robot
% for i= 1:iter-1
%     scatter(x_hat_eqf2(1,i),x_hat_eqf2(2,i),'Color','black','LineStyle',':','HandleVisibility','off')
%     pause(.001)
% end

plot(x_hat_eqf2(1,1:iter-1), x_hat_eqf2(2,1:iter-1),'Color','black','LineStyle',':', 'HandleVisibility','off')

scatter(x_hat_eqf2(1, 1),x_hat_eqf2(2, 1),'o', 'black','HandleVisibility','off')
scatter(x_hat_eqf2(1,iter),x_hat_eqf2(2,iter),'o', 'black','DisplayName','EQF2 start and end position')

% %Plot iterated position of robot
% for i= 1:iter-1
%     plot(x_hat_eqf3(1,i),x_hat_eqf3(2,i),'Color','magenta','LineStyle','-','HandleVisibility','off')
%     pause(.001)
% end
plot(x_hat_eqf3(1,1:iter-1), x_hat_eqf3(2,1:iter-1),'Color','magenta','LineStyle','-', 'HandleVisibility','off')

scatter(x_hat_eqf3(1, 1),x_hat_eqf3(2, 1),'v', 'magenta','HandleVisibility','off')
scatter(x_hat_eqf3(1,iter),x_hat_eqf3(2,iter),'v', 'magenta','DisplayName','EQF3 start and end position')

plot(x_true(1,1:iter-1), x_true(2,1:iter-1),'Color','red','LineStyle','-.', 'HandleVisibility','off')

%x_ttrue=Pose(1:2,3*iter);
scatter(x_true(1, 1),x_true(2, 1),'s', 'red','HandleVisibility','off')
scatter(x_true(1, iter),x_true(2, iter),'s', 'red','DisplayName','True start and end position')


title('Robot position')
hold off
legend
grid on

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