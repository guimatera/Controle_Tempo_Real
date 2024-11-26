classdef F
    methods(Static)
        function out = VectorField(x,u,A,B,d)
            out=(A*x+B*(u+d));
        end
        
        %% LQR pure
        function [S,PAR,BUFFER]=InitializeLQRpure(h,Ac,Bc,Cc,Dc,x0,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0)
                
                %% 1DOF Linear Plant
                Op=obsv(Ac,Cc);
                sysc=ss(Ac,Bc,Cc,Dc);
                np=max(size(Ac));
                % Discrete 
                %TypeDiscretization='Euler';
                TypeDiscretization='ZOH';
                switch TypeDiscretization
                    case 'Euler'
                        % Discrete Euler
                        Ad=eye(2,2)+h*Ac;Bd=h*Bc;Cd=Cc;
                    case 'ZOH'
                        % Discrete ZOH
                        sysZOH=c2d(sysc,h,'zoh');
                        Ad=sysZOH.a;Bd=sysZOH.b;Cd=sysZOH.c;
                end
                PAR.Ac=Ac;PAR.Bc=Bc;PAR.Cc=Cc;PAR.Dc=Dc;
                PAR.Ad=Ad;PAR.Bd=Bd;PAR.Cd=Cd;PAR.np=np;
                
                PAR.x0=x0;
                % ----------------------------------------------------------------
                                
                %% Augumented System: Just Plant and Generator
                Aa=[PAR.Ad];
                Ba=[PAR.Bd];
                Ca=[PAR.Cd];
                na=max(size(Aa));

                PAR.rankdesiredQ=(na+1)*(na+2)/2; 

                PAR.xa0=[PAR.x0];
                
                %% LQR Design
                gh=g^h;
                Qa=Ca'*Qe*Ca;

                % Optimum Gain via ARE
                [Kstar,Pstar,CLP] = dlqr(sqrt(gh)*Aa,sqrt(gh)*Ba,Qa,R);
                % ----------------------------------------------------------------
                % Optimum Gain via ARE (Checking)
                Kstar1=inv(R+sqrt(gh)*Ba'*Pstar*sqrt(gh)*Ba)*sqrt(gh)*Ba'*Pstar*sqrt(gh)*Aa;
                % ----------------------------------------------------------------
                % Optimum Gain via Lyapunov (Checking)
                Pstar1=dlyap((Aa-Ba*Kstar)'*sqrt(gh),Qa+Kstar'*R*Kstar);
                % ----------------------------------------------------------------
                %disp('Consistence ARE (Expected 0)')
                %[norm(Kstar-Kstar1) norm(Pstar-Pstar1)]
                %disp('Ideal Eigenvalues (Aabar-Babar*Kstar)')
                %Eigstar=eig((Aa*sqrt(gh)-Ba*sqrt(gh)*Kstar));
                %abs(Eigstar)
                % ----------------------------------------------------------------
                Lstar=Kstar;
                Jstar=[Aa Ba;
                  -Lstar*Aa -Lstar*Ba];
                Gstar=[Qa zeros(na,1);
                   zeros(1,na) R];
                Hstar=dlyap(Jstar'*sqrt(gh),Gstar);
                THETAstar=F.FromPtoTHETA(Hstar);
                % ----------------------------------------------------------------

                PAR.Qa=Qa;PAR.R=R;PAR.g=g;PAR.gh=gh;
                PAR.Aa=Aa;PAR.Ba=Ba;PAR.na=na;
                PAR.Kstar=Kstar;PAR.Pstar=Pstar;PAR.THETAstar=THETAstar;
                % ----------------------------------------------------------------

 
                %% Buffers (Discrete)
                BUFFER.k=[];BUFFER.d=[];
                BUFFER.u=[];BUFFER.x=[];BUFFER.y=[];BUFFER.e=[];
                BUFFER.ei=[];BUFFER.du=[];
                BUFFER.tc=[];BUFFER.uc=[];
                BUFFER.xc=[];BUFFER.yc=[];
                BUFFER.ec=[];
                BUFFER.K=[];BUFFER.v0=[];BUFFER.rg=[];BUFFER.r=[];
                BUFFER.ki=[];
                BUFFER.erls=[];
                BUFFER.thetahat=[];
                BUFFER.prls=[];
                %% Buffers (Continuous)
                BUFFER.tc=[];
                BUFFER.uc=[];
                BUFFER.xc=[];
                BUFFER.yc=[];
                BUFFER.ec=[];
                
                %% Initial Gain, Noise and Disturbance
                S.K=PAR.Kstar*K0factor;
                %Eig=eig((Aa(1:2,1:2)*sqrt(gh)-Ba(1:2,1)*sqrt(gh)*Kstar(1,1:2)));
                %S.K(1,1:2)=place(Aa(1:2,1:2)*sqrt(gh),Ba(1:2,1)*sqrt(gh),Eig/20);

                PAR.Anoise=noise;
                S.dc=dc0;% Continuous Disturbance
                S.d=d0;% Discrete Disturbance

                %% Initial RLS state (THETA,Prls)
                L=S.K;
                J=[PAR.Aa PAR.Ba;
                  -L*PAR.Aa -L*PAR.Ba];
                G=[PAR.Qa zeros(PAR.na,1);
                   zeros(1,PAR.na) PAR.R];
                H0=dlyap(J'*sqrt(PAR.gh),G);
                THETAhat0=F.FromPtoTHETA(H0);

                S.thetahat=THETAhat0*THETA0factor;
                %S.thetahat=PAR.THETAstar*0.4;
                %S.thetahat=THETAhat0*1.2;
                PAR.prls0=eye(PAR.rankdesiredQ)*PRLS0factor;

                %% Initial Conditions
                S.x=PAR.x0;
                S.xa=PAR.xa0;

                S.xnew=[];
                S.xmnew=[];


                S.sumv0=0;
                S.v0=-S.xa'*PAR.Pstar*S.xa;
                
                S.prls=PAR.prls0;

                
        end
 
        function [S,BUFFER]=SignalsLQRpure(k,PAR,S,BUFFER,h)
                %% Input Disturbance
                if (k*h>100)
                    S.d=-4*0;
                end
                %% Auxiliary Signals at k
                % Discrete Plant Output at k
                S.y=PAR.Cd*S.x;
                % Discrete Tracking Error at k
                S.e=S.y-0;



                %% Read Plant-Augumented States at k
                S.xa=[S.x];
                %% Compute the Control uk
                U=-S.K*S.xa;
                % Noise for persistence excitation
                noise=PAR.Anoise*rand(1,1);
                %noise=PAR.Anoise*chirp(k*h,1,200,10);
                U=U+noise;
                S.u=U;

                %% Get Reward rk
                S.r=-S.xa'*PAR.Qa*S.xa-S.u'*PAR.R*S.u;
                S.sumv0=S.sumv0+S.r*PAR.gh^k;

                %% BUFFER
                BUFFER.k=[BUFFER.k k];

                BUFFER.r=[BUFFER.r S.r];
                BUFFER.rg=[BUFFER.rg S.r*PAR.gh^k];
                BUFFER.v0=[BUFFER.v0 S.sumv0];

                BUFFER.K=[BUFFER.K S.K];
                BUFFER.x=[BUFFER.x S.x];
                BUFFER.y=[BUFFER.y S.y];
                BUFFER.e=[BUFFER.e S.e];
                BUFFER.u=[BUFFER.u S.u];
                BUFFER.d=[BUFFER.d S.d];
                
        end

        function [S,BUFFER]=ContinuousSignalsLQR(PAR,S,BUFFER,tc)
                %% Input Disturbance
                if (tc>100)
                    S.dc=-4*0;
                end
                
                % Continuous Plant Output
                S.yc=PAR.Cc*S.xc;
                
                % Continuous Tracking Error
                S.ec=S.yc-0;

                % ZOH --> Continuous Plant Control Signal
                S.uc=S.u; 
                
                %% BUFFER
                BUFFER.tc=[BUFFER.tc tc];
                BUFFER.uc=[BUFFER.uc S.uc];
                BUFFER.xc=[BUFFER.xc S.xc];
                BUFFER.yc=[BUFFER.yc S.yc];
                BUFFER.ec=[BUFFER.ec S.ec];
      
        end
     
        function [S,BUFFER]=LeastSquaresLQRpure(k,PAR,S,BUFFER,h)
                % RLS-Augumented State at k -- Include U for RLS estimation

                Z=[S.xa;S.u];

                Zbar=F.Fromx2xbar(Z);


                % Plant-Augumented State at k+1 
                Xanew=[S.xnew;S.xmnew];
                % RLS-Augumented State at k+1 -- Include DU for RLS estimation
                Znew=[Xanew;-S.K*Xanew];
                Zbarnew=F.Fromx2xbar(Znew);
                phiz=Zbar-PAR.gh*Zbarnew;



                %% Update THETAk(i) in the RLS 
                Erls=S.r-phiz'*S.thetahat;
                mu=1; 
                S.thetahatnew=S.thetahat+S.prls*phiz*Erls/(mu+phiz'*S.prls*phiz);
                S.prlsnew=(1/mu)*S.prls-(1/mu)*(S.prls*phiz*phiz'*S.prls)/(mu+phiz'*S.prls*phiz);

                BUFFER.ki=[BUFFER.ki k];
                BUFFER.erls=[BUFFER.erls norm(Erls)];
                BUFFER.thetahat=[BUFFER.thetahat norm(S.thetahat)/norm(PAR.THETAstar)];
                BUFFER.prls=[BUFFER.prls norm(S.prls)];

                S.thetahat=S.thetahatnew;
                S.prls=S.prlsnew;

        end

        %% LQT Continuos Part
        %---------------------------------------------------
        function [S,BUFFER]=ContinuousSignalsLQT(PAR,S,BUFFER,tc)
                %% Input Disturbance
                if (tc>100)
                    S.dc=-4*0;
                end
                
                % Continuous Plant Output
                S.yc=PAR.Cc*S.xc;
                % ZOH --> Desired Trajectory 
                S.xmc=S.xm; 
                S.ymc=PAR.Cm*S.xmc+PAR.Yss;

                % Continuous Tracking Error
                S.ec=S.yc-S.ymc;

                % ZOH --> Continuous Plant Control Signal
                S.uc=S.u; 
                
                %% BUFFER
                BUFFER.tc=[BUFFER.tc tc];
                BUFFER.uc=[BUFFER.uc S.uc];
                BUFFER.xc=[BUFFER.xc S.xc];
                BUFFER.yc=[BUFFER.yc S.yc];
                BUFFER.ec=[BUFFER.ec S.ec];
                BUFFER.ymc=[BUFFER.ymc S.ymc];

        end
        %---------------------------------------------------
        %% LQT pure
        function [S,BUFFER]=SignalsLQTpure(k,PAR,S,BUFFER,h)
                %% Input Disturbance
                if (k*h>100)
                    S.d=-4*0;
                end
                %% Auxiliary Signals at k
                % Desired Trajectory at k
                S.ym=PAR.Cm*S.xm;
                % Discrete Plant Output at k
                S.y=PAR.Cd*S.x;
                % Discrete Tracking Error at k
                S.e=S.y-S.ym;



                %% Read Plant-Augumented States at k
                S.xa=[S.x;S.xm];
                %% Compute the Control uk
                U=-S.K*S.xa;
                % Noise for persistence excitation
                noise=PAR.Anoise*rand(1,1);
                %noise=PAR.Anoise*chirp(k*h,1,200,10);
                U=U+noise;
                S.u=U;

                %% Get Reward rk
                S.r=-S.xa'*PAR.Qa*S.xa-S.u'*PAR.R*S.u;
                S.sumv0=S.sumv0+S.r*PAR.gh^k;

                %% BUFFER
                BUFFER.k=[BUFFER.k k];

                BUFFER.r=[BUFFER.r S.r];
                BUFFER.rg=[BUFFER.rg S.r*PAR.gh^k];
                BUFFER.v0=[BUFFER.v0 S.sumv0];

                BUFFER.K=[BUFFER.K S.K];
                BUFFER.x=[BUFFER.x S.x];
                BUFFER.y=[BUFFER.y S.y];
                BUFFER.e=[BUFFER.e S.e];
                BUFFER.u=[BUFFER.u S.u];
                BUFFER.d=[BUFFER.d S.d];
                BUFFER.ym=[BUFFER.ym S.ym];

        end
        
        function [S,PAR,BUFFER]=InitializeLQTpure(h,Ac,Bc,Cc,Dc,x0,ymtilde,yss,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0)
                %% Reference Generator
                [Am,Cm,nm,xm0]=F.ParRef(ymtilde,h);

                PAR.Yss=yss;
                PAR.Cm=Cm;
                PAR.Am=Am;PAR.nm=nm;
                PAR.xm0=xm0;
                % ----------------------------------------------------------------

                %% 1DOF Linear Plant
                Op=obsv(Ac,Cc);
                sysc=ss(Ac,Bc,Cc,Dc);
                np=max(size(Ac));
                % Discrete 
                %TypeDiscretization='Euler';
                TypeDiscretization='ZOH';
                switch TypeDiscretization
                    case 'Euler'
                        % Discrete Euler
                        Ad=eye(2,2)+h*Ac;Bd=h*Bc;Cd=Cc;
                    case 'ZOH'
                        % Discrete ZOH
                        sysZOH=c2d(sysc,h,'zoh');
                        Ad=sysZOH.a;Bd=sysZOH.b;Cd=sysZOH.c;
                end
                PAR.Ac=Ac;PAR.Bc=Bc;PAR.Cc=Cc;PAR.Dc=Dc;
                PAR.Ad=Ad;PAR.Bd=Bd;PAR.Cd=Cd;PAR.np=np;
                
                PAR.x0=x0;
                % ----------------------------------------------------------------
                
                
                %% Augumented System: Just Plant and Generator
                Aa=[PAR.Ad zeros(PAR.np,PAR.nm);
                    zeros(PAR.nm,PAR.np) PAR.Am];
                Ba=[PAR.Bd;zeros(PAR.nm,1)];
                Ca=[PAR.Cd -PAR.Cm];
                na=max(size(Aa));

                PAR.rankdesiredQ=(na+1)*(na+2)/2;

                PAR.xa0=[PAR.x0;PAR.xm0];
                
                %% LQR Design
                gh=g^h;
                Qa=Ca'*Qe*Ca;

                % Optimum Gain via ARE
                [Kstar,Pstar,CLP] = dlqr(sqrt(gh)*Aa,sqrt(gh)*Ba,Qa,R);
                % ----------------------------------------------------------------
                % Optimum Gain via ARE (Checking)
                Kstar1=inv(R+sqrt(gh)*Ba'*Pstar*sqrt(gh)*Ba)*sqrt(gh)*Ba'*Pstar*sqrt(gh)*Aa;
                % ----------------------------------------------------------------
                % Optimum Gain via Lyapunov (Checking)
                Pstar1=dlyap((Aa-Ba*Kstar)'*sqrt(gh),Qa+Kstar'*R*Kstar);
                % ----------------------------------------------------------------
                %disp('Consistence ARE (Expected 0)')
                %[norm(Kstar-Kstar1) norm(Pstar-Pstar1)]
                %disp('Ideal Eigenvalues (Aabar-Babar*Kstar)')
                %Eigstar=eig((Aa*sqrt(gh)-Ba*sqrt(gh)*Kstar));
                %abs(Eigstar)
                % ----------------------------------------------------------------
                Lstar=Kstar;
                Jstar=[Aa Ba;
                  -Lstar*Aa -Lstar*Ba];
                Gstar=[Qa zeros(na,1);
                   zeros(1,na) R];
                Hstar=dlyap(Jstar'*sqrt(gh),Gstar);
                THETAstar=F.FromPtoTHETA(Hstar);
                % ----------------------------------------------------------------

                PAR.Qa=Qa;PAR.R=R;PAR.g=g;PAR.gh=gh;
                PAR.Aa=Aa;PAR.Ba=Ba;PAR.na=na;
                PAR.Kstar=Kstar;PAR.Pstar=Pstar;PAR.THETAstar=THETAstar;
                % ----------------------------------------------------------------

 
                %% Buffers (Discrete)
                BUFFER.k=[];BUFFER.d=[];
                BUFFER.u=[];BUFFER.x=[];BUFFER.y=[];BUFFER.e=[];BUFFER.ym=[];
                BUFFER.ei=[];BUFFER.du=[];
                BUFFER.tc=[];BUFFER.uc=[];
                BUFFER.xc=[];BUFFER.yc=[];
                BUFFER.ec=[];BUFFER.ymc=[];
                BUFFER.K=[];BUFFER.v0=[];BUFFER.rg=[];BUFFER.r=[];
                BUFFER.ki=[];
                BUFFER.erls=[];
                BUFFER.thetahat=[];
                BUFFER.prls=[];
                %% Buffers (Continuous)
                BUFFER.tc=[];
                BUFFER.uc=[];
                BUFFER.xc=[];
                BUFFER.yc=[];
                BUFFER.ec=[];
                BUFFER.ymc=[];
                
                %% Initial Gain, Noise and Disturbance
                S.K=PAR.Kstar*K0factor;
                %Eig=eig((Aa(1:2,1:2)*sqrt(gh)-Ba(1:2,1)*sqrt(gh)*Kstar(1,1:2)));
                %S.K(1,1:2)=place(Aa(1:2,1:2)*sqrt(gh),Ba(1:2,1)*sqrt(gh),Eig/20);

                PAR.Anoise=noise;
                S.dc=dc0;% Continuous Disturbance
                S.d=d0;% Discrete Disturbance

                %% Initial RLS state (THETA,Prls)
                L=S.K;
                J=[PAR.Aa PAR.Ba;
                  -L*PAR.Aa -L*PAR.Ba];
                G=[PAR.Qa zeros(PAR.na,1);
                   zeros(1,PAR.na) PAR.R];
                H0=dlyap(J'*sqrt(PAR.gh),G);
                THETAhat0=F.FromPtoTHETA(H0);

                S.thetahat=THETAhat0*THETA0factor;
                %S.thetahat=PAR.THETAstar*0.4;
                %S.thetahat=THETAhat0*1.2;
                PAR.prls0=eye(PAR.rankdesiredQ)*PRLS0factor;

                %% Initial Conditions
                S.x=PAR.x0;
                S.xm=PAR.xm0;
                S.xa=PAR.xa0;

                %S.xold=[];
                %S.uold=[];
                S.xnew=[];
                S.xmnew=[];


                S.sumv0=0;
                S.v0=-S.xa'*PAR.Pstar*S.xa;
                
                S.prls=PAR.prls0;

                
        end
        
        function [S,BUFFER]=LeastSquaresLQTpure(k,PAR,S,BUFFER,h)
                % RLS-Augumented State at k -- Include U for RLS estimation

                Z=[S.xa;S.u];

                Zbar=F.Fromx2xbar(Z);


                % Plant-Augumented State at k+1 
                Xanew=[S.xnew;S.xmnew];
                % RLS-Augumented State at k+1 -- Include DU for RLS estimation
                Znew=[Xanew;-S.K*Xanew];
                Zbarnew=F.Fromx2xbar(Znew);
                phiz=Zbar-PAR.gh*Zbarnew;



                %% Update THETAk(i) in the RLS 
                Erls=S.r-phiz'*S.thetahat;
                mu=1; 
                S.thetahatnew=S.thetahat+S.prls*phiz*Erls/(mu+phiz'*S.prls*phiz);
                S.prlsnew=(1/mu)*S.prls-(1/mu)*(S.prls*phiz*phiz'*S.prls)/(mu+phiz'*S.prls*phiz);

                BUFFER.ki=[BUFFER.ki k];
                BUFFER.erls=[BUFFER.erls norm(Erls)];
                BUFFER.thetahat=[BUFFER.thetahat norm(S.thetahat)/norm(PAR.THETAstar)];
                BUFFER.prls=[BUFFER.prls norm(S.prls)];

                S.thetahat=S.thetahatnew;
                S.prls=S.prlsnew;

        end
        %---------------------------------------------------
        %% LQT with Integrator
        function [S,PAR,BUFFER]=InitializeLQTint(h,xi0,Ac,Bc,Cc,Dc,x0,ymtilde,yss,R,Qe,g,K0factor,THETA0factor,PRLS0factor,noise,d0,dc0)
                %% Reference Generator
                [Am,Cm,nm,xm0]=F.ParRef(ymtilde,h);

                PAR.Yss=yss;
                PAR.Cm=Cm;
                PAR.Am=Am;PAR.nm=nm;
                PAR.xm0=xm0;
                % ----------------------------------------------------------------

                %% Integrator
                PAR.xi0=xi0;
                %% 1DOF Linear Plant
                Op=obsv(Ac,Cc);
                sysc=ss(Ac,Bc,Cc,Dc);
                np=max(size(Ac));
                % Discrete 
                %TypeDiscretization='Euler';
                TypeDiscretization='ZOH';
                switch TypeDiscretization
                    case 'Euler'
                        % Discrete Euler
                        Ad=eye(2,2)+h*Ac;Bd=h*Bc;Cd=Cc;
                    case 'ZOH'
                        % Discrete ZOH
                        sysZOH=c2d(sysc,h,'zoh');
                        Ad=sysZOH.a;Bd=sysZOH.b;Cd=sysZOH.c;
                end
                PAR.Ac=Ac;PAR.Bc=Bc;PAR.Cc=Cc;PAR.Dc=Dc;
                PAR.Ad=Ad;PAR.Bd=Bd;PAR.Cd=Cd;PAR.np=np;
                
                PAR.x0=x0;
                % ----------------------------------------------------------------
                
                
                %% Augumented System: Just Plant and Generator
                Aa=[PAR.Ad zeros(PAR.np,1) zeros(PAR.np,PAR.nm);
                   -h*PAR.Cd 1 zeros(1,PAR.nm);
                   zeros(PAR.nm,PAR.np) zeros(PAR.nm,1) PAR.Am];
                Ba=[PAR.Bd;0;zeros(PAR.nm,1)];
                Ca=[PAR.Cd -1/h -PAR.Cm];
                na=max(size(Aa));
                PAR.rankdesiredQ=(na+1)*(na+2)/2;
                
                PAR.xa0=[PAR.x0;PAR.xi0;PAR.xm0];

                
                %% LQR Design
                gh=g^h;
                Qa=Ca'*Qe*Ca;

                % Optimum Gain via ARE
                [Kstar,Pstar,CLP] = dlqr(sqrt(gh)*Aa,sqrt(gh)*Ba,Qa,R);
                % ----------------------------------------------------------------
                % Optimum Gain via ARE (Checking)
                Kstar1=inv(R+sqrt(gh)*Ba'*Pstar*sqrt(gh)*Ba)*sqrt(gh)*Ba'*Pstar*sqrt(gh)*Aa;
                % ----------------------------------------------------------------
                % Optimum Gain via Lyapunov (Checking)
                Pstar1=dlyap((Aa-Ba*Kstar)'*sqrt(gh),Qa+Kstar'*R*Kstar);
                % ----------------------------------------------------------------
                %disp('Consistence ARE (Expected 0)')
                %[norm(Kstar-Kstar1) norm(Pstar-Pstar1)]
                %disp('Ideal Eigenvalues (Aabar-Babar*Kstar)')
                %Eigstar=eig((Aa*sqrt(gh)-Ba*sqrt(gh)*Kstar));
                %abs(Eigstar)
                % ----------------------------------------------------------------
                Lstar=Kstar;
                Jstar=[Aa Ba;
                  -Lstar*Aa -Lstar*Ba];
                Gstar=[Qa zeros(na,1);
                   zeros(1,na) R];
                Hstar=dlyap(Jstar'*sqrt(gh),Gstar);
                THETAstar=F.FromPtoTHETA(Hstar);
                % ----------------------------------------------------------------

                PAR.Qa=Qa;PAR.R=R;PAR.g=g;PAR.gh=gh;
                PAR.Aa=Aa;PAR.Ba=Ba;PAR.na=na;
                PAR.Kstar=Kstar;PAR.Pstar=Pstar;PAR.THETAstar=THETAstar;
                % ----------------------------------------------------------------

 
                %% Buffers (Discrete)
                BUFFER.k=[];BUFFER.d=[];
                BUFFER.u=[];BUFFER.x=[];BUFFER.y=[];BUFFER.e=[];BUFFER.ym=[];
                BUFFER.ei=[];BUFFER.du=[];
                BUFFER.tc=[];BUFFER.uc=[];
                BUFFER.xc=[];BUFFER.yc=[];
                BUFFER.ec=[];BUFFER.ymc=[];
                BUFFER.K=[];BUFFER.v0=[];BUFFER.rg=[];BUFFER.r=[];
                BUFFER.ki=[];
                BUFFER.erls=[];
                BUFFER.thetahat=[];
                BUFFER.prls=[];
                %% Buffers (Continuous)
                BUFFER.tc=[];
                BUFFER.uc=[];
                BUFFER.xc=[];
                BUFFER.yc=[];
                BUFFER.ec=[];
                BUFFER.ymc=[];
                
                %% Initial Gain, Noise and Disturbance
                S.K=PAR.Kstar*K0factor;
                %Eig=eig((Aa(1:2,1:2)*sqrt(gh)-Ba(1:2,1)*sqrt(gh)*Kstar(1,1:2)));
                %S.K(1,1:2)=place(Aa(1:2,1:2)*sqrt(gh),Ba(1:2,1)*sqrt(gh),Eig/20);

                PAR.Anoise=noise;
                S.dc=dc0;% Continuous Disturbance
                S.d=d0;% Discrete Disturbance

                %% Initial RLS state (THETA,Prls)
                L=S.K;
                J=[PAR.Aa PAR.Ba;
                  -L*PAR.Aa -L*PAR.Ba];
                G=[PAR.Qa zeros(PAR.na,1);
                   zeros(1,PAR.na) PAR.R];
                H0=dlyap(J'*sqrt(PAR.gh),G);
                THETAhat0=F.FromPtoTHETA(H0);

                S.thetahat=THETAhat0*THETA0factor;
                %S.thetahat=PAR.THETAstar*0.4;
                %S.thetahat=THETAhat0*1.2;
                PAR.prls0=eye(PAR.rankdesiredQ)*PRLS0factor;

                %% Initial Conditions
                S.x=PAR.x0;
                S.xm=PAR.xm0;
                S.xa=PAR.xa0;

                S.xold=PAR.x0;
                S.uold=0;
                S.xnew=[];
                S.xmnew=[];


                S.sumv0=0;
                S.v0=-S.xa'*PAR.Pstar*S.xa;
                
                S.prls=PAR.prls0;

                
        end
        
        function [S,BUFFER]=SignalsLQTint4DOF(k,PAR,S,BUFFER,h)
                [S.x,BUFFER.x]=F.SignalsLQTint(k,PAR.x,S.x,BUFFER.x,h);
                [S.y,BUFFER.y]=F.SignalsLQTint(k,PAR.y,S.y,BUFFER.y,h);
                [S.z,BUFFER.z]=F.SignalsLQTint(k,PAR.z,S.z,BUFFER.z,h);
                [S.yaw,BUFFER.yaw]=F.SignalsLQTint(k,PAR.yaw,S.yaw,BUFFER.yaw,h);
        end
        
        function [S,BUFFER]=SignalsLQTint(k,PAR,S,BUFFER,h)
                %% Input Disturbance
                if (k*h>100)
                    S.d=10*0;
                    
                end
                if (k*h>200)
                    S.d=0;
                    
                end
                
                %% Auxiliary Signals at k
                % Desired Trajectory at k
                S.ym=PAR.Cm*S.xm+PAR.Yss;
                % Discrete Plant Output at k
                S.y=PAR.Cd*S.x;
                % Discrete Tracking Error at k
                S.e=S.y-S.ym;

                % Delta State x(k)-x(k-1)
                S.dx=S.x-S.xold;
                % Discrete Plant Output at k-1
                S.yold=PAR.Cd*S.xold;
                % Integrator Input Error at k-1
                Iold=h*(PAR.Yss-S.yold);
                %% Read Plant-Augumented States at k
                % Augumented State at k -- Integrator and Reference
                S.xa=[S.dx;Iold;S.xm];
                % Delta Discrete Plant Control Signal at k
                S.du=-S.K*S.xa;
                %% Compute the Control uk
                U=S.uold+S.du;
                % Noise for persistence excitation
                noise=PAR.Anoise*rand(1,1);
                %noise=PAR.Anoise*chirp(k*h,1,200,10);
                U=U+noise;
                S.u=U;

                %% Get Reward rk
                S.r=-S.xa'*PAR.Qa*S.xa-S.du'*PAR.R*S.du;
                S.sumv0=S.sumv0+S.r*PAR.gh^k;

                %% BUFFER
                BUFFER.k=[BUFFER.k k];

                BUFFER.r=[BUFFER.r S.r];
                BUFFER.rg=[BUFFER.rg S.r*PAR.gh^k];
                BUFFER.v0=[BUFFER.v0 S.sumv0];

                BUFFER.K=[BUFFER.K S.K];
                BUFFER.x=[BUFFER.x S.x];
                BUFFER.y=[BUFFER.y S.y];
                BUFFER.e=[BUFFER.e S.e];
                BUFFER.u=[BUFFER.u S.u];
                BUFFER.d=[BUFFER.d S.d];
                BUFFER.ym=[BUFFER.ym S.ym];

        end

        function [S,BUFFER]=LeastSquaresLQTint(k,PAR,S,BUFFER,h)
                % RLS-Augumented State at k -- Include U for RLS estimation

                Z=[S.xa;S.du];

                Zbar=F.Fromx2xbar(Z);
        
                % Delta State x(k+1)-x(k)
                S.dxnew=S.xnew-S.x;
                % Discrete Plant Output at k
                S.y=PAR.Cd*S.x;
                % Integrator Input Error at k
                I=h*(PAR.Yss-S.y);
                % Augumented State at k+1 -- Integrator and Reference
                S.xanew=[S.dxnew;I;S.xmnew];
                        
                % RLS-Augumented State at k+1 -- Include DU for RLS estimation
                Znew=[S.xanew;-S.K*S.xanew];
                Zbarnew=F.Fromx2xbar(Znew);
                phiz=Zbar-PAR.gh*Zbarnew;



                %% Update THETAk(i) in the RLS 
                Erls=S.r-phiz'*S.thetahat;
                mu=1; 
                S.thetahatnew=S.thetahat+S.prls*phiz*Erls/(mu+phiz'*S.prls*phiz);
                S.prlsnew=(1/mu)*S.prls-(1/mu)*(S.prls*phiz*phiz'*S.prls)/(mu+phiz'*S.prls*phiz);

                BUFFER.ki=[BUFFER.ki k];
                BUFFER.erls=[BUFFER.erls norm(Erls)];
                BUFFER.thetahat=[BUFFER.thetahat norm(S.thetahat)/norm(PAR.THETAstar)];
                BUFFER.prls=[BUFFER.prls norm(S.prls)];

                S.thetahat=S.thetahatnew;
                S.prls=S.prlsnew;

        end
        %---------------------------------------------------
        %% Auxiliary Functions
        function [xbar]=Fromx2xbar(x)
            n=length(x);
            xbar=[(x.^2)'];

            for i=1:n-1
                for j=i+1:n
                    xbar=[xbar x(i)*x(j)];
                end
            end
            xbar=-xbar';
        end

        function [THETA]=FromPtoTHETA(P)
            % THETA correspond to the matrix P of order nXn
            n=max(size(P));

            THETA=[];
            N=n*(n+1)/2;

            indx=[];
            for i=1:n
                indx=[indx; [i i]];
            end

            for i=1:n-1
                for j=i+1:n
                    indx=[indx; [i j]];
                end
            end

            for i=1:n
                THETA(i)=P(indx(i,1),indx(i,2));
            end
            for i=n+1:N
                THETA(i)=2*P(indx(i,1),indx(i,2));
            end

            THETA=THETA';
        end

        function Pout=FromTHETAtoP(THETA,n)
            % THETA correspond to the submatrix P of order nXn

            P=[];
            N=n*(n+1)/2;

            indx=[];
            for i=1:n
                indx=[indx; [i i]];
            end

            for i=1:n-1
                for j=i+1:n
                    indx=[indx; [i j]];
                end
            end


            k=1;Pout1=[];Pout2=[];Pout=[];
            for i=1:n
                Pout1(indx(i,1),indx(i,2))=THETA(k);
                Pout2(indx(i,1),indx(i,2))=0;
                k=k+1;
            end
            for i=n+1:N
                Pout2(indx(i,1),indx(i,2))=THETA(k)/2;k=k+1;
            end

            Pout=Pout1+Pout2+Pout2';
        end

        function [Am,Cm,nm,xm0] = ParRef(s,h)
            S=collect(laplace(s));
            [nS,dS]=numden(S);
            num=eval(coeffs(nS,'All'));
            den=eval(coeffs(dS,'All'));
            [Amc,Bmc,Cmc,Dmc]=tf2ss(num,den);
            [Vm,Em]=eig(Amc);
            invVm=inv(Vm);

            Am=expm(Amc*h);Cm=Cmc;
            xm0=Bmc;

            %Am=-1;Cm=1;
            %xm0=1;

            nm=max(size(Am));
        end
        %---------------------------------------------------
    end
end


