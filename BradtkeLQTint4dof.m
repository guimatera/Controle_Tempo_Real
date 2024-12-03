clear all

FLAG_PLANT='DISCRETE';
%FLAG_PLANT='CONTINUOUSlinear';
%FLAG_PLANT='CONTINUOUSdrone';


FLAG_UPDATE=0;
% Sampling time (h) and integration step (hc)
h=1/1;
T=30*h;
NT=1*8/h;

hc=h/100;
substeps=floor(h/hc);

%% Reference Generator
syms ts real
% ----------------------------------------------------------------
c1=5;c2=3;c3=3*0.1571/3;c4=10;
ymtilde=((c1-c2)*cos(c3*ts)+c4*cos(((c1-c2)*c3/c2)*ts));
yss=4;

%% Integrator
xi0=0;
% ----------------------------------------------------------------

%% 1DOF Linear Plant
% ----------------------------------------------------------------
%Continuous
%den=conv(conv([1 1],[1 2]),[1 3]);
%den=conv(conv(den,[1 4]),[1 5]);
%[Ac,Bc,Cc,Dc]=tf2ss(den(end),den);
Ac=[0 1;0 -1.5];Bc=[0 1.3]';Cc=[1 0];Dc=0;

% Condições iniciais dadas pelo Professor:
x0 = [5;0];
y0 = x0;
z0 = x0;
yaw0 = x0;

% Condições Iniciais calculadas/escolhidas por mim e Guilherme com base no slide 67
% x0 = [12; 0];
% y0 = [0; 0];
% z0 = [11; 0];
% yaw0 = [0.7854; 0];

% ----------------------------------------------------------------
%% LQR Design
% Seguir com os parâmetros que o Professor disse na 
% última aula (foto na pasta Documentação)
% ----------------------------------------------------------------
R=1;
g=0.5;%0.999919;
Qe=1/10; % For CONTINUOUSdrone flag
%Qe=1;

%% Initial Gain, Noise and Disturbance
K0factor=1/2; % For CONTINUOUSdrone flag
%K0factor=1;
noise=0;
dc0=0;d0=0;

%% Initial RLS
THETA0factor=0.9;
PRLS0factor=1e-6;%*1e-5/2;
% grande permite explorar


%% Others
CHgT=[];
GainNorm=[];
Gain=[];
    
%% Initialization for S,PAR,BUFFER
%{
for i=1:2
    [s{i},par{i},buffer{i}]=F.InitializeLQTint(h,xi0,Ac,Bc,Cc,Dc,x0,ymtilde,yss,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0);
end
%}

c1=5;c2=3;c3=3*0.1571/3;c4=10;
yss=4;
for i=1:4
    switch i
        case 1
            ymtilde=((c1-c2)*cos(c3*ts)+c4*cos(((c1-c2)*c3/c2)*ts));
            [s{i},par{i},buffer{i}]=F.InitializeLQTint(h,xi0,Ac,Bc,Cc,Dc,x0,ymtilde,yss,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0);
            
        case 2
            ymtilde=((c1-c2)*sin(c3*ts)-c4*sin(((c1-c2)*c3/c2)*ts));
            [s{i},par{i},buffer{i}]=F.InitializeLQTint(h,xi0,Ac,Bc,Cc,Dc,y0,ymtilde,yss,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0);
            
        case 3
            c3=3*0.1571/10;
            ymtilde=cos(c3*ts);
            yss=10;
            [s{i},par{i},buffer{i}]=F.InitializeLQTint(h,xi0,Ac,Bc,Cc,Dc,z0,ymtilde,yss,R,Qe*2,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0);
           
        case 4
            c3=3*0.1571/20;
            ymtilde=(45*pi/180)*cos(c3*ts);
            yss=0;
            [s{i},par{i},buffer{i}]=F.InitializeLQTint(h,xi0,Ac,Bc,Cc,Dc,yaw0,ymtilde,yss,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0);
            
    end 
end
%[S.K;PAR.Kstar]

%{
for i=1:4
    [s{i}.K;par{i}.Kstar]
end
%}

%% Drone Initialization
BuildConfig;

DATA=Config.DATA;
FLAG=Config.FLAG;
DIST=Config.DIST;
UAV=Config.UAV;
PARuav=Config.PARuav;
InPID=Config.InPID;
dFilter=Config.dFilter;
xs0=Config.xs0;     

%% Buffer DRONE
bufferDRONE.vect=[];
bufferDRONE.vecCmdVel=[];
bufferDRONE.vecdx=[];
bufferDRONE.vecdy=[];
bufferDRONE.vecdz=[];
bufferDRONE.vecdyaw=[];
bufferDRONE.vecx=[];
bufferDRONE.vecy=[];
bufferDRONE.vecz=[];
bufferDRONE.vecyaw=[];

bufferDRONE.vecOmega=[];
bufferDRONE.vecq=[];
bufferDRONE.vecroll=[];
bufferDRONE.vecpitch=[];
% Forces and Wind
bufferDRONE.vecvw=[];
bufferDRONE.vecFdrag=[];
bufferDRONE.vecFweight=[];
bufferDRONE.vecFcontrol=[];
bufferDRONE.vecFresult=[];
bufferDRONE.vecFi=[];
bufferDRONE.vecsumFdi=[];
bufferDRONE.vecFd=[];  
bufferDRONE.vecd=[];



%% Simulation

switch FLAG_PLANT
    case {'DISCRETE','CONTINUOUSlinear'}
        for i=1:4
            s{i}.xc=s{i}.x;
        end
    case 'CONTINUOUSdrone'
        s{1}.xc=[UAV.p0(1);UAV.v0(1)];
        s{2}.xc=[UAV.p0(2);UAV.v0(2)];
        s{3}.xc=[UAV.p0(3);UAV.v0(3)];
        s{4}.xc=[UAV.yaw0;UAV.dyaw0];
       
end

k=0;tc=0;xs=xs0;
countk=1;counttc=1;
while (countk<=NT)
    % Initialize RLS: 
    for i=1:4
        s{i}.prls=par{i}.prls0;
    end
    for counti=1:floor(T/h)
        % READ PLANT STATES at k
        %% Compute the Control uk 
        for i=1:4
            [s{i},buffer{i}]=F.SignalsLQTint(k,par{i},s{i},buffer{i},h);
        end
        
        %% Apply uk to the PLANT
        switch FLAG_PLANT
          case 'DISCRETE'
                %% LINEAR
                for i=1:4
                    s{i}.xnew=F.VectorField(s{i}.x,s{i}.u,par{i}.Ad,par{i}.Bd,s{i}.d);
                end
          case 'CONTINUOUSlinear'
                %% LINEAR
                for j=1:substeps
                    %% Signals
                    for i=1:4
                        [s{i},buffer{i}]=F.ContinuousSignalsLQT(par{i},s{i},buffer{i},tc);
                    end
                    %% Integration
                    for i=1:4
                        s{i}.xc = s{i}.xc + hc*F.VectorField(s{i}.xc,s{i}.uc,par{i}.Ac,par{i}.Bc,s{i}.dc);
                    end
                    tc=tc+hc;
                end
                for i=1:4
                    s{i}.xnew=s{i}.xc;
                end
          case 'CONTINUOUSdrone'
                %% DRONE
                for j=1:substeps
                    %% Signals
                    for i=1:4
                        [s{i},buffer{i}]=F.ContinuousSignalsLQT(par{i},s{i},buffer{i},tc);
                    end
                    UC=[s{1}.u;s{2}.u;s{3}.u;s{4}.u]; %ZOH
                    
                    %% Integration
                    [dxs,bufferDRONE]=DroneFullModel(xs,tc,counttc,UC,Config,bufferDRONE,windX,windY,windZ);
                    counttc=counttc+1;
                    xs=xs+hc*dxs;
                    s{1}.xc=[xs(1);xs(4)];% px
                    s{2}.xc=[xs(2);xs(5)];% py
                    s{3}.xc=[xs(3);xs(6)];% pz
                    % Obtaining psi
                    q=xs(7:10);
                    % Angular Velocity
                    Omega=xs(11:13);

                    % Orientation (Rotation Matrix and Euler)
                    R=quat2rotm(q');
                    eul=rotm2eul(R,'ZYX')'; 
                    yaw=eul(1);pitch=eul(2);roll=eul(3);
                    Jacob1c=[cos(pitch);0;0];
                    Jacob2c=[sin(roll).*sin(pitch);cos(roll).*cos(pitch);sin(roll)];
                    Jacob3c=[cos(roll).*sin(pitch);-sin(roll).*cos(pitch);cos(roll)];
                    aux=Jacob1c.*Omega(1)+Jacob2c.*Omega(2)+Jacob3c.*Omega(3);

                    % Angular Velocity (Euler)
                    deul=aux./cos(pitch);
                    dyaw=deul(3);
                    s{4}.xc=[yaw;dyaw];

                    tc=tc+hc;
                end
                for i=1:4
                    s{i}.xnew=s{i}.xc;
                end
        end
        for i=1:4
            s{i}.xmnew=par{i}.Am*s{i}.xm;   
        end
        
        
        %% Update RLS
        for i=1:4
            [s{i},buffer{i}]=F.LeastSquaresLQTint(k,par{i},s{i},buffer{i},h);
        end
        

        %% Update Time and Control/State
        % Discrete time update
        k=k+1;
        % Control/State Update
        for i=1:4
            s{i}.xold=s{i}.x;s{i}.uold=s{i}.u;s{i}.x=s{i}.xnew;s{i}.xm=s{i}.xmnew;
        end
        
        %% End Loop 1
    end
    %% For visualization
    VTHETAhat{countk}=buffer{1}.thetahat;
    Vprls{countk}=buffer{1}.prls;
    VE{countk}=buffer{1}.erls;
    Vti{countk}=buffer{1}.ki;
    buffer{1}.ki=[];buffer{1}.erls=[];buffer{1}.thetahat=[];buffer{1}.prls=[];
    
    CHgT=[CHgT k*h];
    GainNorm=[GainNorm norm(s{1}.K)];
    Gain=[Gain s{1}.K'];
    
    
        
    for i=1:4
        Hhat=F.FromTHETAtoP(s{i}.thetahat,par{i}.na+1);
        % Policy Inmprovement
        if (FLAG_UPDATE)
            s{i}.K=inv(Hhat(par{i}.na+1,par{i}.na+1))*Hhat(par{i}.na+1,1:end-1);
        end
    end
    
    % Update counter: change gain
    countk=countk+1;
end
plotBradtkeLQTint4dof;
%[S.K;PAR.Kstar]
for i=1:4
    [s{i}.K;par{i}.Kstar]
end