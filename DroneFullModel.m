function [dxs,out]=DroneFullModel(xs,t,countt,UC,Config,out,windX,windY,windZ)
    %% PARAMETERS: Definition and ICs

    DATA=Config.DATA;
    FLAG=Config.FLAG;
    DIST=Config.DIST;
    UAV=Config.UAV;
    PAR=Config.PARuav;
    InPID=Config.InPID;
    dFilter=Config.dFilter;
    xs0=Config.xs0;     
    
    %% READ PLANT States at time t
    switch FLAG.PLANTtopology
       case {'UAVfull'}
            %% Nonlinear Plant
            % Mass Center Position and Velocity
            p=xs(1:3);
            v=xs(4:6);
            % Orientation
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
            dpitch=deul(2);
            droll=deul(1);
            % Mass Center Position and Velocity
            x=p(1);dx=v(1);
            y=p(2);dy=v(2);
            z=p(3);dz=v(3);


            xslen=13;               
    end
    %% READ INNER CTRL States at time t
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            % Inner Control States at time t
            InPID.states=xs(xslen+1:xslen+4);
            xslen=xslen+4;
    end      
    %% READ External Equiv. DISTURBANCE Filter States at time t
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            dFilter.states=xs(xslen+1:xslen+3);
            xslen=xslen+3;
    end
    %% VELOCITY CMD  at time t
    uc(1)=UC(1);uc(2)=UC(2);uc(3)=UC(3);uc(4)=UC(4);
    %uc=UC;
    CmdVel=uc;
    dCmdVel=CmdVel*0;
    %% INNER CONTROLLERS at time t
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            % ##########################################################
            % Input: CmdVel(1),dCmdVel(1), CmdVel(2), dCmdVel(2), CmdVel(3), dCmdVel(3), CmdVel(4), dCmdVel(4)
            % Input: roll, pitch, yaw, dx, dy, dz
            % Input: Omega, droll, dpitch, dyaw
            % Input: InPID.states
            % Output: Mc, fc
            % ##########################################################
            syaw=sin(yaw);cyaw=cos(yaw);
            sroll=sin(roll);croll=cos(roll);
            spitch=sin(pitch);cpitch=cos(pitch);

            % ##########################################################
            % z
            % ddz= Uz
            e=CmdVel(3)-dz;
            Uz=InPID.kd(3)*e+dCmdVel(3)+InPID.ki(3)*InPID.states(3);
            if (croll==0) disp('Error: croll=0')
            end
            if (cpitch==0) disp('Error: cpitch=0')
            end
            fc=(Uz+PAR.g)*UAV.M./(croll.*cpitch);


            % % ##########################################################
            % % XY part
            % % ##########################################################
            % % ddx = Ux
            e=CmdVel(1)-dx;

            Ux=InPID.kd(1)*e+dCmdVel(1)+InPID.ki(1)*InPID.states(1);
            % ddy = Uy
            e=CmdVel(2)-dy;
            Uy=InPID.kd(2)*e+dCmdVel(2)+InPID.ki(2)*InPID.states(2);
            % U=[Ux;Uy];
            % Ryaw=[cyaw -syaw;syaw cyaw];
            % invRyaw=[cyaw syaw;-syaw cyaw]
            % vaux=inv(Ryaw)*U/fc*UAV.M;
            % 
            invRyawc1=[cyaw;-syaw];
            invRyawc2=[syaw;cyaw];

            U=[Ux;Uy];


            if (fc==0) disp('Error: fc=0')
            end
            vaux=((invRyawc1.*U(1)+invRyawc2.*U(2))./fc)*UAV.M;

            % if (abs(vaux(1))>=1) 
            %     vd(1)=sign(vaux(1));
            % else
            %     vd(1)=vaux(1);
            % end
            % 
            % if (abs(vaux(2))>=1) 
            %     vd(2)=sign(vaux(2));
            % else
            %     vd(2)=vaux(2);
            % end
            % 

            % SAT (-1,1)
            if (vaux(1)>=1)
                vd(1)=0.9;
            else
                if (vaux(1)<=-1)
                    vd(1)=-0.9;
                else
                    vd(1)=vaux(1);
                end
            end

            % SAT (-1,1)
            if (vaux(2)>=1)
                vd(2)=0.9;
            else
                if (vaux(2)<=-1)
                    vd(2)=-0.9;
                else
                    vd(2)=vaux(2);
                end
            end


            % %Just for open-loop test
            % As=5*pi/180;DC=0;ws=2*pi*[1/400]; 
            % refsroll=(sin(t*ws)*As')+DC;
            % drefsroll=cos(t*ws)*(As.*ws)';
            % ddrefsroll=sin(t*ws)*(-As.*(ws.^2))';
            % refspitch=refsroll;
            % drefspitch=drefsroll;
            % ddrefspitch=ddrefsroll;


            % Assuming that rolld and pitchd belongs to (-pi/2,pi/2)
            % then, cosrolld>0, cospitchd>0.
            refsroll=-vd(2);
            refcroll=sqrt(1-refsroll.^2);
            refspitch=vd(1)./refcroll;
            drefsroll=0*refsroll;
            ddrefsroll=0*refsroll;
            drefspitch=0*refspitch;
            ddrefspitch=0*refspitch;

            % % ##########################################################
            Jx=UAV.J(1,1);Jy=UAV.J(2,2);Jz=UAV.J(3,3);
            % 
            % 
            % % ##########################################################
            % % Roll
            e=refsroll-sroll;
            de=drefsroll-croll.*droll;
            Uroll=InPID.kp(5)*e+InPID.kd(5)*de+ddrefsroll;
            Mroll=Jx*(Uroll+sroll.*droll.^2)./croll-dpitch.*dyaw*(Jy-Jz);
            % 
            % % ##########################################################
            % % Pitch
            e=refspitch-spitch;
            de=drefspitch-cpitch.*dpitch;
            Upitch=InPID.kp(6)*e+InPID.kd(6)*de+ddrefspitch;
            Mpitch=Jy*(Upitch+spitch.*dpitch.^2)./cpitch-droll.*dyaw*(Jz-Jx);
            % % ##########################################################
            % 
            % 
            % % ##########################################################
            % % Yaw
            Omegax=Omega(1);Omegay=Omega(2);
            e=CmdVel(4)-dyaw;
            Uyaw=InPID.kd(4)*e+dCmdVel(4)+InPID.ki(4)*InPID.states(4);
            Myaw=Jz*Uyaw-dpitch.*droll*(Jx-Jy)+(Jy-Jx)*Omegax.*Omegay;
            % 
            % 
            % % ##########################################################
            Mc=[Mroll;Mpitch;Myaw];
            % Output: fc,Mc
            f=fc;
            M=Mc;     
    end
    %% SIGNALS: perturbations, others
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            %% Wind Perturbation 
            % No disturbance
            if (t<DIST.twind)
                vw=[0;0;0];
            else
                %vw=[0;0;0];
                %vw=DATA(:,countt);
                vw=[windX{1}(countt);windY{1}(countt);windZ{1}(countt)];
            end
            %vw=zeros(3,1);
            %% Actuator
            % -----------------------------------------------
            % Input: M, f
            % Actuator 
            % -----------------------------------------------
            % Propellers' Thrust
            Fi=UAV.invCtrAllocation*[f;M];
            % Propellers' Angular Velocity
            dti=diag(UAV.vsi)*sqrt(abs(Fi)/UAV.kTi);
            % Check for Negative Thrust
            for i=1:UAV.nr
                if(~isempty(find(Fi(i)<=0)))
                    disp('Fi negative or null')
                end
            end
            % -----------------------------------------------
            %% Aerodynamic Drag
            % -----------------------------------------------
            % Fdrag
            taud=zeros(3,1);
            vr=v-vw;

            Fd=-R*UAV.KFd*R'*vr*norm(vr);
            switch (FLAG.DRAGtype)
                    case 0
                        Fd=zeros(3,1);
            end
            % 
            Fdrag=zeros(3,1);
            taudrag=zeros(3,1);
            for i=1:UAV.nr  
                switch (FLAG.DRAGtype)
                    case 0
                        Fdi=zeros(3,1);
                    case 1
                        Fdi=-UAV.KFdi*abs(dti(i))*(vr+R*cross(Omega,UAV.pbi{i}));
                    case 2
                        Fdi=-UAV.kFdi*abs(dti(i))*(eye(3,3)-R*UAV.mui{i}*UAV.mui{i}'*R')*(vr+R*cross(Omega,UAV.pbi{i}));
                end
                Fdrag=Fdrag+Fdi;
                taudrag=taudrag+cross(UAV.pbi{i},R'*Fdi);
            end
            % 
            sumFdi=Fdrag;
            Fdrag=Fdrag+Fd;
            taudrag=taudrag+taud;
            % 
            % taudist
            %Omegax=Omega(1);Omegay=Omega(2.:);
            sumdti=sum(dti,1);
            % 
            taudist=UAV.Iz*[Omega(2).*sumdti;-Omega(1).*sumdti;0];
            switch (UAV.DISTterm)
                case 0
                    taudist=taudist*0;
            end
            % -----------------------------------------------               
            %% Forces 
            Fresult=(-UAV.M*PAR.g*PAR.e3+R*f*PAR.e3+Fdrag);
            Fweight=-UAV.M*PAR.g*PAR.e3;
            Fcontrol=R*f*PAR.e3;
            %% External Equivalent Disturbance
            %Inner Dist. due only to Drag
            din=Fdrag/UAV.m;

            auxd(1)=(1/InPID.kd(1))*din(1)+dFilter.states(1);
            auxd(2)=(1/InPID.kd(2))*din(2)+dFilter.states(2);
            auxd(3)=(1/InPID.kd(3))*din(3)+dFilter.states(3);
            auxd(4)=0;  

            d=[auxd(1);auxd(2);auxd(3);auxd(4)];
    end
    %% Vector Field: Plants
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            %% NL Plant 
            qw=q(1);qv=q(2:4);
            dqw=-qv'*Omega/2;
            dqv=qw*Omega/2+cross(qv,Omega)/2;
            dq=[dqw;dqv];
            dv=(-UAV.M*PAR.g*PAR.e3+R*f*PAR.e3+Fdrag)/UAV.M;
            dOmega=inv(UAV.J)*(-cross(Omega,UAV.J*Omega)+M+taudrag+taudist);

            dxp=[v;dv;dq;dOmega];

            % ------------ Inner Controller Integrators  ---------------------------
            dintx=CmdVel(1)-dx;
            dinty=CmdVel(2)-dy;
            dintz=CmdVel(3)-dz;
            dintyaw=CmdVel(4)-dyaw;

            dint=[dintx;dinty;dintz;dintyaw];
            dxs=[dxp;dint];   
    end
    %% Vector Field: External Disturbance Filter
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            dxdFilter(1)=-(InPID.ki(1)/InPID.kd(1))*dFilter.states(1)-(InPID.ki(1)/(InPID.kd(1)^2))*din(1);
            dxdFilter(2)=-(InPID.ki(2)/InPID.kd(2))*dFilter.states(2)-(InPID.ki(2)/(InPID.kd(2)^2))*din(2);
            dxdFilter(3)=-(InPID.ki(3)/InPID.kd(3))*dFilter.states(3)-(InPID.ki(3)/(InPID.kd(3)^2))*din(3);

            dxs=[dxs;dxdFilter']; 
    end
    %% FINAL PHASE: Save DATA
    
    %% Basic
    out.vect=[out.vect t];
    out.vecCmdVel=[out.vecCmdVel CmdVel];
    out.vecdx=[out.vecdx dx];
    out.vecdy=[out.vecdy dy];
    out.vecdz=[out.vecdz dz];
    out.vecdyaw=[out.vecdyaw dyaw];
    out.vecx=[out.vecx x];
    out.vecy=[out.vecy y];
    out.vecz=[out.vecz z];
    out.vecyaw=[out.vecyaw yaw];
    switch FLAG.PLANTtopology
        case {'UAVfull'}
            out.vecOmega=[out.vecOmega Omega];
            out.vecq=[out.vecq q];
            out.vecroll=[out.vecroll roll];
            out.vecpitch=[out.vecpitch pitch];
            %% Forces and Wind
            out.vecvw=[out.vecvw vw];
            out.vecFdrag=[out.vecFdrag Fdrag];
            out.vecFweight=[out.vecFweight Fweight];
            out.vecFcontrol=[out.vecFcontrol Fcontrol];
            out.vecFresult=[out.vecFresult Fresult];
            out.vecFi=[out.vecFi Fi];
            out.vecsumFdi=[out.vecsumFdi sumFdi];
            out.vecFd=[out.vecFd Fd];  
            out.vecd=[out.vecd d];                         
    end
end