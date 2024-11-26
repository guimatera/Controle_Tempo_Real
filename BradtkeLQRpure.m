clear all

FLAG_PLANT='DISCRETE';
%FLAG_PLANT='CONTINUOUS';



FLAG_UPDATE=0;
% Sampling time (h) and integration step (hc)
h=1/1;
T=30*h;
NT=8/h;

hc=h/10;
substeps=floor(h/hc);


%% 1DOF Linear Plant
% ----------------------------------------------------------------
%Continuous
%den=conv(conv([1 1],[1 2]),[1 3]);
%den=conv(conv(den,[1 4]),[1 5]);
%[Ac,Bc,Cc,Dc]=tf2ss(den(end),den);
Ac=[0 1;0 -1.5];Bc=[0 1.3]';Cc=[1 0];Dc=0;
x0=[5;0];

% ----------------------------------------------------------------
%% LQR Design
% ----------------------------------------------------------------
R=1;
g=0.01;%0.999919; %forgetting factor
Qe=1;

%% Initial Gain, Noise and Disturbance
K0factor=1/1;
noise=0;
dc0=0;d0=0;

%% Initial RLS
THETA0factor=0.8;
PRLS0factor=1e-6;%*1e-5/2;
% grande permite explorar

%% Others
CHgT=[];
GainNorm=[];
Gain=[];
    
%% Initialization for S,PAR,BUFFER
[S,PAR,BUFFER]=F.InitializeLQRpure(h,Ac,Bc,Cc,Dc,x0,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0);




%[S.K;PAR.Kstar]
%% Simulation
k=0;S.xc=S.x;tc=0;
countk=1;countj=1;
while (countk<=NT)
    % Initialize RLS: 
    S.prls=PAR.prls0;
    for counti=1:floor(T/h)
        % READ PLANT STATES at k
        %% Compute the Control uk 
        [S,BUFFER]=F.SignalsLQRpure(k,PAR,S,BUFFER,h);
        
        %% Apply uk to the PLANT
        switch FLAG_PLANT
          case 'DISCRETE'
                S.xnew=F.VectorField(S.x,S.u,PAR.Ad,PAR.Bd,S.d);
          case 'CONTINUOUS'
                for i=1:substeps
                    %% Signals
                    [S,BUFFER]=F.ContinuousSignalsLQR(PAR,S,BUFFER,tc);
                    %% LINEAR
                    S.xc = S.xc + hc*F.VectorField(S.xc,S.uc,PAR.Ac,PAR.Bc,S.dc);
                    
                    tc=tc+hc;
                end
                S.xnew=S.xc;
        end
       
        %% Update RLS
        [S,BUFFER]=F.LeastSquaresLQRpure(k,PAR,S,BUFFER,h);

        %% Update Time and Control/State
        % Discrete time update
        k=k+1;
        % Control/State Update
        S.x=S.xnew;
        S.xm=S.xmnew;
        
        %% End Loop 1
    end
    VTHETAhat{countk}=BUFFER.thetahat;
    Vprls{countk}=BUFFER.prls;
    VE{countk}=BUFFER.erls;
    Vti{countk}=BUFFER.ki;
    BUFFER.ki=[];BUFFER.erls=[];BUFFER.thetahat=[];BUFFER.prls=[];
    
    CHgT=[CHgT k*h];
    GainNorm=[GainNorm norm(S.K)];
    Gain=[Gain S.K'];
    
    
    %[S.K;PAR.Kstar]
    Hhat=F.FromTHETAtoP(S.thetahat,PAR.na+1);
    % Policy Inmprovement
    if (FLAG_UPDATE)
        S.K=inv(Hhat(PAR.na+1,PAR.na+1))*Hhat(PAR.na+1,1:end-1);
    end
    % Update counter: change gain
    countk=countk+1;
end
plotBradtkeLQRpure;
[S.K;PAR.Kstar]
