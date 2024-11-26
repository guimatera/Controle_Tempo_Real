close all

switch FLAG_PLANT
      case 'DISCRETE'
            figure(1)
            subplot(4,1,1)
            plot(buffer{1}.k*h,buffer{1}.y,'.r','LineWidth',2)
            hold on
            plot(buffer{1}.k*h,buffer{1}.ym,'.k','LineWidth',2)
            legend('Discrete Output','Discrete Reference Model')
            hold off
            title('{\bf Position and Orientation}','Interpreter','latex')
            grid
            ylabel('X','Interpreter','latex')
            subplot(4,1,2)
            plot(buffer{2}.k*h,buffer{2}.y,'.r','LineWidth',2)
            hold on
            plot(buffer{2}.k*h,buffer{2}.ym,'.k','LineWidth',2)
            hold off
            grid
            ylabel('Y','Interpreter','latex')
            subplot(4,1,3)
            plot(buffer{3}.k*h,buffer{3}.y,'.r','LineWidth',2)
            hold on
            plot(buffer{3}.k*h,buffer{3}.ym,'.k','LineWidth',2)
            hold off
            grid
            ylabel('Z','Interpreter','latex')
            subplot(4,1,4)
            plot(buffer{4}.k*h,buffer{4}.y,'.r','LineWidth',2)
            hold on
            plot(buffer{4}.k*h,buffer{4}.ym,'.k','LineWidth',2)
            hold off
            grid
            ylabel('Yaw','Interpreter','latex')
            xlabel('Time','Interpreter','latex')
            
            
            figure(2)
            subplot(4,1,1)
            plot(buffer{1}.k*h,buffer{1}.u,'.r','LineWidth',2)
            title('{\bf Control of Position and Orientation}','Interpreter','latex')
            hold on
            hold off
            grid
            ylabel('X','Interpreter','latex')
    
            subplot(4,1,2)
            plot(buffer{2}.k*h,buffer{2}.u,'.r','LineWidth',2)
            grid
            ylabel('Y','Interpreter','latex')
      
            subplot(4,1,3)
            plot(buffer{3}.k*h,buffer{3}.u,'.r','LineWidth',2)
            grid
            ylabel('Z','Interpreter','latex')
            
            subplot(4,1,4)
            plot(buffer{4}.k*h,buffer{4}.u,'.r','LineWidth',2)
            grid
            ylabel('Yaw','Interpreter','latex')
            xlabel('Time','Interpreter','latex')
            
            
            figure(3)
            plot3(buffer{1}.y, buffer{2}.y, buffer{3}.y, '.b')
            title('{\bf Position}','Interpreter','latex')
            hold on
            plot3(buffer{1}.ym, buffer{2}.ym, buffer{3}.ym, '.r')
            grid
            xlabel('X','Interpreter','latex')
            ylabel('Y','Interpreter','latex')
            zlabel('Z','Interpreter','latex')
            legend('Saída Discreta','Modelo de Referência Discreto')


     case {'CONTINUOUSlinear','CONTINUOUSdrone'}
            figure(1)
            subplot(4,1,1)
            plot(buffer{1}.tc,buffer{1}.yc,'g','LineWidth',2)
            hold on
            plot(buffer{1}.k*h,buffer{1}.y,'.r','LineWidth',2)
            plot(buffer{1}.tc,buffer{1}.ymc,'b','LineWidth',2)
            plot(buffer{1}.k*h,buffer{1}.ym,'.k','LineWidth',2)
            hold off
            title('{\bf Position and Orientation}','Interpreter','latex')
            legend('Continuous Output','Discrete Output','Continuous Reference Model','Discrete Reference Model')
            grid
            ylabel('X','Interpreter','latex')
            
            subplot(4,1,2)
            plot(buffer{2}.tc,buffer{2}.yc,'g','LineWidth',2)
            hold on
            plot(buffer{2}.k*h,buffer{2}.y,'.r','LineWidth',2)
            plot(buffer{2}.tc,buffer{2}.ymc,'b','LineWidth',2)
            plot(buffer{2}.k*h,buffer{2}.ym,'.k','LineWidth',2)
            hold off
            grid
            ylabel('Y','Interpreter','latex')
            
            subplot(4,1,3)
            plot(buffer{3}.tc,buffer{3}.yc,'g','LineWidth',2)
            hold on
            plot(buffer{3}.k*h,buffer{3}.y,'.r','LineWidth',2)
            plot(buffer{3}.tc,buffer{3}.ymc,'b','LineWidth',2)
            plot(buffer{3}.k*h,buffer{3}.ym,'.k','LineWidth',2)
            hold off
            grid
            ylabel('Z','Interpreter','latex')
            
            subplot(4,1,4)
            plot(buffer{4}.tc,buffer{4}.yc*(180/pi),'g','LineWidth',2)
            hold on
            plot(buffer{4}.k*h,buffer{4}.y*(180/pi),'.r','LineWidth',2)
            plot(buffer{4}.tc,buffer{4}.ymc*(180/pi),'b','LineWidth',2)
            plot(buffer{4}.k*h,buffer{4}.ym*(180/pi),'.k','LineWidth',2)
            hold off
            grid
            ylabel('Yaw','Interpreter','latex')
            xlabel('Time','Interpreter','latex')
            
            figure(2)
            
            subplot(4,1,1)
            plot(buffer{1}.tc,buffer{1}.uc,'b','LineWidth',2)
            hold on
            plot(buffer{1}.k*h,buffer{1}.u,'.r','LineWidth',2)
            hold off
            title('{\bf Control of Position and Orientation}','Interpreter','latex')
            legend('Continuous Control','Discrete Control')
            grid
            ylabel('X','Interpreter','latex')
            
            subplot(4,1,2)
            plot(buffer{2}.tc,buffer{2}.uc,'b','LineWidth',2)
            hold on
            plot(buffer{2}.k*h,buffer{2}.u,'.r','LineWidth',2)
            hold off
            grid
            ylabel('Y','Interpreter','latex')
            
            subplot(4,1,3)
            plot(buffer{3}.tc,buffer{3}.uc,'b','LineWidth',2)
            hold on
            plot(buffer{3}.k*h,buffer{3}.u,'.r','LineWidth',2)
            hold off
            grid
            ylabel('Z','Interpreter','latex')
            
            subplot(4,1,4)
            plot(buffer{4}.tc,buffer{4}.uc,'b','LineWidth',2)
            hold on
            plot(buffer{4}.k*h,buffer{4}.u,'.r','LineWidth',2)
            hold off
            grid
            ylabel('Yaw','Interpreter','latex')
            xlabel('Time','Interpreter','latex')
            
            
            figure(3)
            plot3(buffer{1}.yc, buffer{2}.yc, buffer{3}.yc, 'm')
            hold on
            plot3(buffer{1}.y, buffer{2}.y, buffer{3}.y, '.r')
            hold on
            plot3(buffer{1}.ymc, buffer{2}.ymc, buffer{3}.ymc, 'b')
            hold on
            plot3(buffer{1}.ym, buffer{2}.ym, buffer{3}.ym, '.k')
            title('{\bf Position}','Interpreter','latex')
            grid
            legend('Saída Contínua','Saída Discreta','Modelo de Referência Contínuo','Modelo de Referência Discreto')
            xlabel('X','Interpreter','latex')
            ylabel('Y','Interpreter','latex')
            zlabel('Z','Interpreter','latex')
end

% figure(4)
% subplot(2,1,1)
% plot(buffer{1}.k*h,buffer{1}.r,'r','LineWidth',2)
% hold on
% plot(buffer{1}.k*h,buffer{1}.rg,'k','LineWidth',2)
% hold off
% title('{\bf Reward}','Interpreter','latex')
% grid
% ylabel('(a)','Interpreter','latex','Rotation',0)
% subplot(2,1,2)
% %plot(buffer{1}.kc,buffer{1}.uc,'r','LineWidth',2)
% plot(buffer{1}.k*h,buffer{1}.v0,'r','LineWidth',2)
% hold on
% yline(s{1}.v0,'k','LineWidth',2)
% yline(sum(buffer{1}.rg),'g','LineWidth',2)
% %plot(buffer{1}.k*h,-diag(buffer{1}.x'*PstarARE*buffer{1}.x),'c','LineWidth',2)
% 
% hold off
% grid
% ylabel('(b)','Interpreter','latex','Rotation',0)
% 
% 
% figure(5)
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
% figure(6)
% plot(CHgT*h,GainNorm,'b*')
