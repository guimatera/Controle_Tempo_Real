
%% General
PARuav.g=10;
PARuav.e3=[0 0 1]';
%% UAVfull
% --------------------------------------------
% Plant-> Full UAV MODEL: Based on the M600 with 4 propellers
% --------------------------------------------
% Number of Propellers 
% Rotation Direction
UAV.vsi=[1 -1 1 -1];
UAV.nr=length(UAV.vsi);
% Propeller half-length
UAV.BladeR=27/100;%m
% Motor Center ArmAltitude (ArmAlt -> h)
% Motor Center Horizontal Displacement (d)
UAV.h=0*4.5/100;%m
UAV.d=57/100;%m
UAV.ArmAlt=UAV.h; 
UAV.ArmLength=sqrt(UAV.d^2+UAV.h^2);
% UAV's mass with payload
UAV.m=10.5;%kg
% Maximum Thrust
UAV.fimax=30*10/6;
% Nominal Spin Rotation
UAV.dtinom=1000*2*pi/60; %rd/s
% Nominal Thrust Magnitude
UAV.finom=1.5*UAV.m*PARuav.g/6; %N
% Nominal Armature Current and Voltage Magnitude
UAV.ianom=4; %A
UAV.MotorDriveVnom=24;%V
% Structure Inertia Matrix 
% Assumed to be a sphere in the center with radius rsphere and mass msphere
% plus 4 (or 6) point mass mrotor ArmLength = sqrt(d^2+h^2) appart
UAV.rsphere=UAV.ArmLength/4;
UAV.msphere=UAV.m*70/100;
UAV.mrotor=(UAV.m*30/100)/6;
% Propeller Hub Inertia Matrix and Mass
% assume a cylinder with radius rcyl and length 2*BladeR and mass mi
UAV.rcyl=1/100;
% Propeller Hub (Motor)
%Bi
UAV.Bi=1e-6;
%kwi
UAV.kwi=UAV.MotorDriveVnom/UAV.dtinom;
%ki
UAV.kii=UAV.kwi;
%rai
UAV.rai=7;
%Lai
UAV.Lai=1e-1;
%=================================================
% Parameters Calculated
%=================================================
% ---------------------------------------------------------
% Geometry and Rotation Matrices
% ---------------------------------------------------------
% Rotation Matrix Rbi, Position Vector pbi
UAV.pbi{1}=[UAV.d*cos(0);UAV.d*sin(0);UAV.ArmAlt]; 
ArmElevAngle=atan(UAV.pbi{1}(3)/UAV.pbi{1}(1));
ROTYArmEle=[cos(-ArmElevAngle) 0 sin(-ArmElevAngle);0 1 0;-sin(-ArmElevAngle) 0  cos(-ArmElevAngle)];

Rbibar{1}=ROTYArmEle;
ROTZarm=[cos(0) -sin(0) 0;sin(0) cos(0) 0;0 0 1];
   
theta=0*[pi/3;pi/4;pi/6;pi/2];
ROTZTI=[cos(theta(1)) -sin(theta(1)) 0;sin(theta(1)) cos(theta(1)) 0;0 0 1];
UAV.Rbi{1}=Rbibar{1}*ROTZTI;
  
for i=2:UAV.nr
    PIsym=pi;
    ArmAngle=(i-1)*(2*PIsym/UAV.nr);
    UAV.pbi{i}=[UAV.d*cos(ArmAngle);UAV.d*sin(ArmAngle);UAV.ArmAlt]; 
    
    ROTZarm=[cos((2*PIsym/UAV.nr)) -sin((2*PIsym/UAV.nr)) 0;sin((2*PIsym/UAV.nr)) cos((2*PIsym/UAV.nr)) 0;0 0 1];
    Rbibar{i}=ROTZarm*Rbibar{i-1};
    
    ROTZTI=[cos(theta(i)) -sin(theta(i)) 0;sin(theta(i)) cos(theta(i)) 0;0 0 1];
    UAV.Rbi{i}=Rbibar{i}*ROTZTI;
end
% ---------------------------------------------------------
% Inertial Matrices
% ---------------------------------------------------------
% Structure Inertia Matrix 
% Assumed to be a sphere in the center with radius rsphere and mass msphere
% plus 4 (or 6) point mass mrotor ArmLength appart
UAV.Ibx=(2/5)*UAV.msphere*UAV.rsphere^2+2*UAV.mrotor*UAV.ArmLength^2;
UAV.Iby=(2/5)*UAV.msphere*UAV.rsphere^2+2*UAV.mrotor*UAV.ArmLength^2;
UAV.Ibz=(2/5)*UAV.msphere*UAV.rsphere^2+4*UAV.mrotor*UAV.ArmLength^2;
UAV.Ib=diag([UAV.Ibx UAV.Iby UAV.Ibz]);%kg m2
%UAV.Ib=diag([0.034 0.034 0.097]);%kg m2
% Propeller Hub Inertia Matrix and Mass
% Propeller-Hub mass 
UAV.mi=UAV.mrotor*20/100;%kg
UAV.Ix=UAV.mi*UAV.rcyl^2/4+UAV.mi*(2*UAV.BladeR)^2/3;
UAV.Iy=UAV.mi*UAV.rcyl^2/4+UAV.mi*(2*UAV.BladeR)^2/3;
UAV.Iz=UAV.mi*UAV.rcyl^2/2;
UAV.Ii=[UAV.Ix 0 0;0 UAV.Iy 0;0 0 UAV.Iz];
% Inertial Matrix Iibar
Iibar{1}=UAV.Rbi{1}*UAV.Ii*UAV.Rbi{1}';

UAV.IibarSum=Iibar{1};
for i=2:UAV.nr 
    Iibar{i}=UAV.Rbi{i}*UAV.Ii*UAV.Rbi{i}';
    UAV.IibarSum=UAV.IibarSum+Iibar{i};
end
% ---------------------------------------------------------
% Propeller Aerodynamics Coeff
% ---------------------------------------------------------
UAV.kTi=UAV.finom/UAV.dtinom^2;
%ctaui
% Mznom=Mxynom/4=finom*ArmLength 
% recalling that
% |taudinom|=ctauinom*finom, one has Mznom = finom*ArmLength = ctauinom*finom
% and ctauinom=ArmLength
UAV.ctaui=UAV.ArmLength;
%taudinom=(ctaui)*finom;
Dragfactor=1/3*60;
%KFdi
UAV.KFdi=diag([1 1 1])*8e-5*Dragfactor;
%kFdi
UAV.kFdi=8e-5*Dragfactor;
% ---------------------------------------------------------
% Structure Aerodynamics Coeff
% ---------------------------------------------------------
UAV.KFd=diag([0.3 0.3/2 0.15/3])*Dragfactor;
% ---------------------------------------------------------
% Propeller Aerodynamics Coeff
% ---------------------------------------------------------
M=[];
for i=1:UAV.nr
    UAV.mui{i}=UAV.Rbi{i}*PARuav.e3;
    M=[M cross(UAV.pbi{i},UAV.mui{i})+UAV.vsi(i)*UAV.ctaui*UAV.mui{i}];
end
UAV.CtrAllocation=[ones(1,UAV.nr);M];
UAV.invCtrAllocation=inv(UAV.CtrAllocation);
% ---------------------------------------------------------
% Propeller Hub plus Structure
% ---------------------------------------------------------
UAV.M=UAV.m+UAV.nr*UAV.mi;
UAV.J=UAV.IibarSum+UAV.Ib;
%================================================
% Drag and Disturbance FLAGS
UAV.DISTterm=0;

%% Disturbances 
% --------------------------------------------
% Input Disturbance
% --------------------------------------------
DIST.twind=20*0;
DIST.tD=20;

%% Controllers
% --------------------------------------------
% Inner PID Controllers
% --------------------------------------------
%x y z yaw roll pitch
InPID.kp=[0 0 0 0 15 15];
%x y z yaw roll pitch
InPID.kd=[1 1 1 1 3.75 3.75];
%x y z yaw roll pitch
InPID.ki=[0.1389 0.1389 0.1389 0.1389 0 0];

%% IC
% --------------------------------------------
% ICs
% --------------------------------------------
% Full UAV model
% --------------------------------------------


%UAV.p0=[15 10 10]';
UAV.p0=[par{1}.Cm*par{1}.xm0+par{1}.Yss par{2}.Cm*par{2}.xm0+par{2}.Yss par{3}.Cm*par{3}.xm0+par{3}.Yss]';
UAV.v0=[0 0 0]';
%UAV.yaw0=0*45*pi/180;
UAV.yaw0=par{4}.Cm*par{4}.xm0+par{4}.Yss;

UAV.Omega0=[0 0 0]';
UAV.pitch0=0;UAV.roll0=0;
% Finding the IC of the time derivatives of roll,pitch,yaw 
eul0=[UAV.yaw0 UAV.pitch0 UAV.roll0];
R0=eul2rotm(eul0);
UAV.q0=rotm2quat(R0)';
JacobInv0=[cos(UAV.pitch0) sin(UAV.roll0)*sin(UAV.pitch0) cos(UAV.roll0)*sin(UAV.pitch0);
            0 cos(UAV.roll0)*cos(UAV.pitch0) -sin(UAV.roll0)*cos(UAV.pitch0);
            0 sin(UAV.roll0) cos(UAV.roll0)]/cos(UAV.pitch0);
deul0=JacobInv0*R0*UAV.Omega0;
UAV.dyaw0=deul0(3);UAV.dpitch0=deul0(2);UAV.droll0=deul0(1);
% --------------------------------------------
% Inner PID integrators: x y z yaw 
% --------------------------------------------
InPID.IC=[0 0 0 0]';
% --------------------------------------------
% Outer PI Integrators: x y z yaw
% --------------------------------------------

dFilter.IC=[0 0 0]';



%% Initial Struct
INIT.PARuav=PARuav;
INIT.UAV=UAV;
INIT.DIST=DIST;
INIT.InPID=InPID;
INIT.dFilter=dFilter;
