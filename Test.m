% Write all the code necessary to run and test your controller(s) in this file
% Code should be in ready-to-execute format

% All the code related to generating plots and verifying your requirement
% should also be included here

nTrials = 100;

distance = 0;
distance = [];
fdist = [];
times = [];
didFail = 1;
count = 0;
iterator = 0;
% uncomment below line to run code only once and comment
% everything below it
% DesignProblem04('Controller','datafile','data.mat','display', false)
% for i=1:nTrials
% MakeRoad('road.mat','display',false);
DesignProblem04('Controller','datafile','data.mat','display', true)
% % Uncomment to use for controller 2
% % DesignProblem04('Controller2','datafile','data.mat','display', false)
% load('data.mat');
% 
% 
% didFail = processdata.result(end);
% if didFail == 0
%     count = count+1;
%     didFail = 1;
% end
% 
% iterator = 1+(processdata.t(end)/.02);
% for n = 1: iterator
%     distance = [distance (processdata.v(n)*.02)];
% end
% 
% fdist = [fdist sum(distance)];
% distance = [];
% times = [times processdata.t(end)];
% i
% end

figure(1)
scatter(fdist,times,'filled')
xlim([0,140]);
ylim([0,55]);
counter = uint8(count);
countlabel=sprintf('Number of Crashes -- %.0f', counter);
h=annotation('textbox',[0.2 0.75 0.1 0.1]);
set(h,'String',{countlabel});
    xlabel('Distance');
    ylabel('Time');
    title('Distance vs. Time for 100 runs')