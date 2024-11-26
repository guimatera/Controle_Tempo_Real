close all

buffer=bufferx;

vtc=buffer.tc;
%bufferx=buffer;
%buffery=buffer;
%bufferz=buffer;
%bufferyaw=buffer;


%% Position XYZ
figure(1)
subplot(4,4,4)
plot(vtc,bufferx.ymc-bufferx.yc,'r','LineWidth',2)
hold on
hold off
title('{\bf Erro}','Interpreter','latex')
grid
ylabel('(a)','Interpreter','latex','Rotation',0)
subplot(4,4,8)
plot(vtc,buffery.ymc-buffery.yc,'r','LineWidth',2)
hold on
hold off
grid
ylabel('(b)','Interpreter','latex','Rotation',0)
subplot(4,4,12)
plot(vtc,bufferz.ymc-bufferz.yc,'r','LineWidth',2)
hold on
hold off
grid
ylabel('(c)','Interpreter','latex','Rotation',0)
subplot(4,4,16)
plot(vtc,(bufferyaw.ymc-bufferyaw.yc)*(180/pi),'r','LineWidth',2)
hold on
hold off
grid
ylabel('(d)','Interpreter','latex','Rotation',0)
xlabel('$t$ (s)','Interpreter','latex')

subplot(4,4,[1:3 5:6 9:11 13:15])
plot3(bufferx.ymc,buffery.ymc,bufferz.ymc,'k','LineWidth',3)
hold on
plot3(bufferx.yc,buffery.yc,bufferz.yc,'r','LineWidth',2)
plot3(bufferx.ymc(1),buffery.ymc(1),bufferz.ymc(1),'oc','LineWidth',6)
plot3(bufferx.ymc(end),buffery.ymc(end),bufferz.ymc(end),'og','LineWidth',6)
xlabel('$p_x,p_{x_d}$ (m)','Interpreter','latex')
ylabel('$p_y,p_{y_d}$ (m)','Interpreter','latex')
zlabel('$p_z,p_{z_d}$ (m)','Interpreter','latex')
title('{\bf XYZ Traj.}','Interpreter','latex')
zlim([0 12])
grid
%legend('Traj. Desejada', 'Modelo Completo','Modelo de Ordem 1','Interpreter','latex')
hold off


figure(2)
subplot(4,1,1)
plot(vtc,bufferx.yc,'r','LineWidth',2)
hold on
plot(vtc,bufferx.ymc,'k','LineWidth',2)
hold off
title('{\bf Position}','Interpreter','latex')
grid
ylabel('(a)','Interpreter','latex','Rotation',0)
subplot(4,1,2)
plot(vtc,buffery.yc,'r','LineWidth',2)
hold on
plot(vtc,buffery.ymc,'k','LineWidth',2)
hold off
grid
ylabel('(b)','Interpreter','latex','Rotation',0)
subplot(4,1,3)
plot(vtc,bufferz.yc,'r','LineWidth',2)
hold on
plot(vtc,bufferz.ymc,'k','LineWidth',2)
hold off
grid
ylabel('(c)','Interpreter','latex','Rotation',0)
subplot(4,1,4)
plot(vtc,(bufferyaw.yc)*(180/pi),'r','LineWidth',2)
hold on
plot(vtc,(bufferyaw.ymc)*(180/pi),'k','LineWidth',2)
hold off
grid
ylabel('(d)','Interpreter','latex','Rotation',0)
xlabel('$t$ (s)','Interpreter','latex')

%% Control
% figure(3)
% subplot(4,1,1)
% plot(vtc,bufferx.uc,'r','LineWidth',2)
% hold on
% %plot(vtc,bufferx.ymc,'k','LineWidth',2)
% hold off
% title('{\bf Control}','Interpreter','latex')
% grid
% ylabel('(a)','Interpreter','latex','Rotation',0)
% subplot(4,1,2)
% plot(vtc,buffery.uc,'r','LineWidth',2)
% hold on
% %plot(vtc,buffery.ymc,'k','LineWidth',2)
% hold off
% grid
% ylabel('(b)','Interpreter','latex','Rotation',0)
% subplot(4,1,3)
% plot(vtc,bufferz.uc,'r','LineWidth',2)
% hold on
% %plot(vtc,bufferz.ymc,'k','LineWidth',2)
% hold off
% grid
% ylabel('(c)','Interpreter','latex','Rotation',0)
% subplot(4,1,4)
% plot(vtc,(bufferyaw.uc),'r','LineWidth',2)
% hold on
% %plot(vtc,(bufferyaw.ymc),'k','LineWidth',2)
% hold off
% grid
% ylabel('(d)','Interpreter','latex','Rotation',0)
% xlabel('$t$ (s)','Interpreter','latex')