close all

switch FLAG_PLANT
      case 'DISCRETE'
            figure(1)
            subplot(2,1,1)
            plot(BUFFER.k*h,BUFFER.y,'.r','LineWidth',2)
            hold on
            plot(BUFFER.k*h,BUFFER.ym,'.k','LineWidth',2)
            legend('Discrete Output','Discrete Reference Model')
            hold off
            title('{\bf Position and Control (LQT Int)}','Interpreter','latex')
            grid
            ylabel('Position','Interpreter','latex')
           
            subplot(2,1,2)
            plot(BUFFER.k*h,BUFFER.u,'.r','LineWidth',2)
            legend('Discrete Control')
            grid
            ylabel('Control','Interpreter','latex')
            xlabel('Time','Interpreter','latex')
            
      case 'CONTINUOUS'
            figure(1)
            subplot(2,1,1)
            plot(BUFFER.tc,BUFFER.yc,'g','LineWidth',2)
            hold on
            plot(BUFFER.k*h,BUFFER.y,'.r','LineWidth',2)
            plot(BUFFER.tc,BUFFER.ymc,'b','LineWidth',2)
            plot(BUFFER.k*h,BUFFER.ym,'.k','LineWidth',2)
            hold off
            title('{\bf Position and Control (LQT Int)}','Interpreter','latex')
            legend('Continuous Output','Discrete Output','Continuous Reference Model','Discrete Reference Model')
            grid
            ylabel('Position','Interpreter','latex')
            
            subplot(2,1,2)
            plot(BUFFER.tc,BUFFER.uc,'b','LineWidth',2)
            hold on
            plot(BUFFER.k*h,BUFFER.u,'.r','LineWidth',2)
            hold off
            grid
            legend('Continuous Control','Discrete Control')
            ylabel('Control','Interpreter','latex')
            xlabel('Time','Interpreter','latex')
end



% figure(2)
% subplot(2,1,1)
% plot(BUFFER.k*h,BUFFER.r,'r','LineWidth',2)
% hold on
% plot(BUFFER.k*h,BUFFER.rg,'k','LineWidth',2)
% hold off
% title('{\bf Reward}','Interpreter','latex')
% grid
% ylabel('(a)','Interpreter','latex','Rotation',0)
% subplot(2,1,2)
% %plot(BUFFER.kc,BUFFER.uc,'r','LineWidth',2)
% plot(BUFFER.k*h,BUFFER.v0,'r','LineWidth',2)
% hold on
% yline(S.v0,'k','LineWidth',2)
% yline(sum(BUFFER.rg),'g','LineWidth',2)
% %plot(BUFFER.k*h,-diag(BUFFER.x'*PstarARE*BUFFER.x),'c','LineWidth',2)
% 
% hold off
% grid
% ylabel('(b)','Interpreter','latex','Rotation',0)
% 
% 
% figure(3)
% subplot(3,1,1)
% hold on
% for k=1:NT
%     plot(Vti{k}*h,VE{k})
% end
% hold off
% subplot(3,1,2)
% hold on
% for k=1:NT
%     plot(Vti{k}*h,VTHETAhat{k})
% end
% hold off
% %plot(CHgT*h,Gain(1,:),'b*')
% %plot(CHgT*h,GainNorm,'b*')
% %hold on
% %yline(KstarARE(1),'r','LineWidth',1)
% %yline(GainNormStar,'r','LineWidth',1)
% %hold off
% subplot(3,1,3)
% hold on
% for k=1:NT
%     plot(Vti{k}*h,Vprls{k})
% end
% hold off
% 
% figure(4)
% plot(CHgT*h,GainNorm,'b*')
