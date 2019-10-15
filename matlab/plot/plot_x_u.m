function [] = plot_x_u(T,X,U)
% inputs
% X: matrix of states, each state is a column,
% NOT TO BE CONFUSED WITH X-AXIS POSITION "X"!!!
%
% U: matrix of controls, each control is a column

ha = tight_subplot(4,2,[.1 .2],[.08 .01],[.12 .1]);
set(0,'defaultlinelinewidth',2);
set(0,'defaultAxesFontSize',13)

%% states
% plot X, Y, Z
axes(ha(1));
hold on;
plot(T,X(1,:),'Color',clr('blue'),'LineStyle',':')
plot(T,X(2,:),'Color',clr('red'),'LineStyle','--');
plot(T,X(3,:),'Color',clr('black'),'LineStyle','-');
xlabel('t (s)');
ylabel('(m)');
legend('X','Y','Z');

% plot \psi, \theta, \phi
axes(ha(2));
hold on;
plot(T,X(4,:),'Color',clr('dark_brown'),'LineStyle',':')
plot(T,X(5,:),'Color',clr('purple'),'LineStyle','--');
plot(T,X(6,:),'Color',clr('sea_green'),'LineStyle','-');
xlabel('t (s)');
ylabel('(rad)');
legend('\psi','\theta','\phi');

% plot alpha
axes(ha(3));
%ylim([-1 6]);
hold on;
plot(T,X(7,:),'Color',clr('maroon'),'LineStyle','-');
plot(T,X(8,:),'Color',clr('light_green'),'LineStyle','--');
plot(T,X(9,:),'Color',clr('light_blue'),'LineStyle','-.');
plot(T,X(10,:),'Color',clr('dark_brown'),'LineStyle',':');
ymin = min(min(X(7:10,:)));
ymax = max(max(X(7:10,:)));
ha(3).YLim = [ymin-1, ymax+1];
xlabel('t (s)');
ylabel('(rad)');
legend('\alpha_1','\alpha_2','\alpha_3','\alpha_4');

% plot d
axes(ha(5));
hold on;
plot(T,X(11,:),'Color',clr('cyan'),'LineStyle','-');
plot(T,X(12,:),'Color',clr('light_brown'),'LineStyle','-');
plot(T,X(13,:),'Color',clr('black'),'LineStyle',':');
plot(T,X(14,:),'Color',clr('purple'),'LineStyle','--');
xlabel('t (s)');
ylabel('(m)');
legend('d_1','d_2','d_3','d_4');

% plot gamma dot
axes(ha(7));
hold on;
plot(T,X(33,:),'Color',clr('light_green'),'LineStyle','-');
plot(T,X(34,:),'Color',clr('red'),'LineStyle','-');
plot(T,X(35,:),'Color',clr('black'),'LineStyle',':');
plot(T,X(36,:),'Color',clr('sea_green'),'LineStyle','--');
xlabel('t (s)');
ylabel('(rad/s)');
ylim([-3000 3000]);
yticks([-3000 0 3000]);
yticklabels({'-3000','0','3000'});
lgn = legend('$\dot{\gamma}_1$','$\dot{\gamma}_2$','$\dot{\gamma}_3$','$\dot{\gamma}_4$');
set(lgn,'Interpreter','latex');
% move ylabel closer to y-axis
ylbl = get(gca,'ylabel');
set(ylbl,'Units','Normalized','Position',[-0.15, 0.5, 0]);

%% controls
% plot arm torques
axes(ha(4));
hold on;
plot(T,U(1,:),'Color',clr('cyan'),'LineStyle','-');
plot(T,U(2,:),'Color',clr('maroon'),'LineStyle','--');
plot(T,U(3,:),'Color',clr('dark_brown'),'LineStyle',':');
plot(T,U(4,:),'Color',clr('light_blue'),'LineStyle','-.');
xlabel('t (s)');
ylabel('(N m)');
legend('\tau_{\alpha1}','\tau_{\alpha2}','\tau_{\alpha3}','\tau_{\alpha4}');

% plot arm forces
axes(ha(6));
hold on;
plot(T,U(5,:),'Color',clr('purple'),'LineStyle','-');
plot(T,U(6,:),'Color',clr('black'),'LineStyle','--');
plot(T,U(7,:),'Color',clr('light_green'),'LineStyle','-.');
plot(T,U(8,:),'Color',clr('light_blue'),'LineStyle',':');
xlabel('t (s)');
ylabel('(N)');
legend('f_{d1}','f_{d2}','f_{d3}','f_{d4}');

% plot propeller torques
axes(ha(8));
hold on;
plot(T,U(9,:),'Color',clr('cyan'),'LineStyle','-');
plot(T,U(10,:),'Color',clr('purple'),'LineStyle','--');
plot(T,U(11,:),'Color',clr('black'),'LineStyle',':');
plot(T,U(12,:),'Color',clr('red'),'LineStyle','-.');
%ylim_current = ylim;    % get current y limits
% set lower y limit less than zero to show speed 4
%ylim([-.5, ylim_current(2)]);
xlabel('t (s)');
ylabel('(N m)');
legend('\tau_{\gamma1}','\tau_{\gamma2}','\tau_{\gamma3}','\tau_{\gamma4}');

% set figure properties
hf = gcf;
hf.Units = 'normalized';
hf.Position = [0 0 .35 .88];

end

