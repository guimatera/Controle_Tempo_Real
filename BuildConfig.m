


%% STEP: Run Configs
step=hc;%1e-2;

tfinal=NT*T; 
%% Wind

for ind=1:length(step)

    TIMEvector{ind}=[0:step(ind):tfinal];

    %% Wind Velocity - Measured with an anemometer
    load anemometer.mat
    twind=anemometer(:,1)-anemometer(1,1);

    
    windXcase1{ind}=interp1(twind,anemometer(:,2),TIMEvector{ind});
    windYcase1{ind}=interp1(twind,anemometer(:,3),TIMEvector{ind});
    windZcase1{ind}=interp1(twind,anemometer(:,4),TIMEvector{ind});
    % 
%     figure(10)
%     subplot(3,1,1)
%     hold on
%     plot(TIMEvector{ind},windXcase1{ind},'r','LineWidth',2)
%     plot(twind,anemometer(:,2),'b','LineWidth',1)
%     hold off
%     subplot(3,1,2)
%     hold on
%     plot(TIMEvector{ind},windYcase1{ind},'r','LineWidth',2)
%     plot(twind,anemometer(:,3),'b','LineWidth',1)
%     hold off
%     subplot(3,1,3)
%     hold on
%     plot(TIMEvector{ind},windZcase1{ind},'r','LineWidth',2)
%     plot(twind,anemometer(:,4),'b','LineWidth',1)
%     hold off

    %% Wind Velocity - Artificial 
    windXcase2{ind}=interp1(linspace(0,tfinal,10),4*[0 0 1 1 2 2 3 3 2 2],TIMEvector{ind});

    windYcase2{ind}=interp1(linspace(0,tfinal,10),2*[0 0 -2 -2 -2 -2 -2 3 3 3],TIMEvector{ind});
    %windYcase2=windXcase2*0;
    windZcase2{ind}=interp1(linspace(0,tfinal,10),1*[0 0 2 2 3 3 4 4 0 0],TIMEvector{ind});
    %windZcase2=windXcase2*0;

    windXcase3{ind}=interp1(linspace(0,180/36,10),4*[0 0 1 1 2 2 3 3 2 2],TIMEvector{ind});

    windYcase3{ind}=interp1(linspace(0,180/36,10),2*[0 0 2 2 2 2 2 2 2 2],TIMEvector{ind});
    %windYcase2=windXcase2*0;
    windZcase3{ind}=interp1(linspace(0,180/36,10),1*[0 0 2 2 3 3 4 4 2 2],TIMEvector{ind});

    %ruido=IMU.wnmean + 1e3*I%MU.wnstd.*randn(1,length(TIMEvector));
    %windXcase2=windXcase2+ruido;
    % 
    

    %% Wind
    windX{ind}=(windXcase1{ind}+windXcase2{ind})/30;
    windY{ind}=(windYcase1{ind}+windYcase2{ind})/30;
    windZ{ind}=(windZcase1{ind}+windZcase2{ind})/30;

    figure(11)
    subplot(3,1,1)
    hold on
    plot(TIMEvector{ind},windX{ind},'r','LineWidth',2)
    hold off
    subplot(3,1,2)
    hold on
    plot(TIMEvector{ind},windY{ind},'r','LineWidth',2)
    hold off
    subplot(3,1,3)
    hold on
    plot(TIMEvector{ind},windZ{ind},'r','LineWidth',2)
    hold off
    
end


%%
INITIALIZE;

%--------------------------------------------------
% Plant
xs0=[UAV.p0;UAV.v0;UAV.q0;UAV.Omega0;InPID.IC];
% Equivalent Input Disturbance
xs0=[xs0;dFilter.IC];

%--------------------------------------------------
Config=INIT;
Config.xs0=xs0;
%--------------------------------------------------
%Config.FLAG.PLANTtopology='UAVlinear2nd'; % 'UAVlinear2nd', 'UAVfull'
Config.FLAG.PLANTtopology='UAVfull'; % 'UAVlinear2nd', 'UAVfull'
Config.FLAG.DRAGtype=1;
Config.DATA=[windX{1};windY{1};windZ{1}];
%--------------------------------------------------
% 
%load('PreviousData.mat');
%Config{2}.DATA=data4; %vecd

% vectprevious=aux5;
% % TIME=[0:step(2):tfinal];
% % ideald{2}(1,:)=interp1(vectprevious,vecdprevious(1,:),TIME);
% % ideald{2}(2,:)=interp1(vectprevious,vecdprevious(2,:),TIME);
% % ideald{2}(3,:)=interp1(vectprevious,vecdprevious(3,:),TIME);
% %     

   
