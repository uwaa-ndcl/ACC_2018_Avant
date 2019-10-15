% use LQR to stabilize the quadrotor from a perturbed initial state

clear all;

% reminder:
% x(1:18) = [X,Y,Z,Psi,Theta,Phi,al1,al2,al3,al4,d1,d2,d3,d4,g1,g2,g3,g4]
% (18 states), Psi = yaw, Theta = pitch, Phi = roll
% u = [arm torques 1-4, arm length forces 1-4, rotor torques 1-4]
% (12 inputs)

%% setup and initial conditions
load_hover_setup; % run script

% initial propeller speeds (at hover)
dg1_0 = -sqrt((1/c1)*(mt*g/4));
dg2_0 = sqrt((1/c1)*(mt*g/4));
dg3_0 = -sqrt((1/c1)*(mt*g/4));
dg4_0 = sqrt((1/c1)*(mt*g/4));

% initial state
x0 = [0; 0; .30;  % rc (X, Y, Z)
      0; .3; .2; % eta (Euler angs): yaw (psi) - pitch (theta) - roll (phi)
      pi/4; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;  % alpha
      (const.l + .5*const.l)*ones(4,1);             % d
      0; 0; 0; 0;   % gamma
      0; 0; 0;      % rc dot
      0; 0; 0;      % eta dot
      0; 0; 0; 0;   % alpha dot
      0; 0; 0; 0;   % d dot
      dg1_0; dg2_0; dg3_0; 0];  % gamma dot

%draw_quadrotor_flat_figure(x_bar,pi/2 + pi/4,pi/2 + pi/4,const)

%% STAGE I: use PD control
% state we would like the vehicle to be at, at the end of Stage I (arm
% reconfiguration and rotor braking)
x_fall_end = [0; 0; 0;  % rc (X, Y, Z)
      x0(4:6); % p (Euler angles): yaw (psi) - pitch (theta) - roll (phi)
      x_bar(7:10);      % alpha
      x_bar(11:14);     % d
      0; 0; 0; 0;       % gamma
      0; 0; 0;          % rc dot
      0; 0; 0;          % eta dot
      0; 0; 0; 0;       % alpha dot
      0; 0; 0; 0;       % d dot
      0; 0; 0; 0];      % gamma dot

% PD gains - these were tuned by hand
k_pa = -3;          % proportional, arm angle
k_pd = -50;         % proportional, arm length
k_pr = -.0001;      % proportional, rotor speed
k_da = -.1;         % derivative, arm angle
k_dd = -4;          % derivative, arm length

u_1_p = @(t,x) [k_pa*(x(7:10) - x_fall_end(7:10)); 
                k_pd*(x(11:14) - x_fall_end(11:14));
                k_pr*x(33:36)];
u_1_d = @(t,x) [k_da*x(25:28);
                k_dd*x(29:32);
                0; 0; 0; 0;];

% control for Stage I
u_1 = @(t,x) u_1_p(t,x) + u_1_d(t,x);

% trajectory for Stage I
T_1 = [t0:(t1-t0)/100:t1]';
[~, X_1] = ode45(@(t,x) dx(t,x,u_1,const),T_1,x0);
X_1 = X_1';
xf_1 = X_1(:,end);
xf_err = xf_1 - x_bar;
xf_err_val = norm(xf_err(7:14));
U_1 = [];   % control at each time output, filled in the loop below
for j=1:length(T_1)   
    U_1 = [U_1, u_1(T_1(j),X_1(:,j))];
end
%plot_x_u(T_1,X_1,U_1);
%plot(T_1,X_1(33:36,:)); % plot prop speeds

%% STAGE II: LQR & control
% Matlab's lqr command was changed after version 2018b, and my code won't
% work with the new version of the command. So, if the Matlab version is
% less than 2018a, load the lqr results rather than re-computing them.
if verLessThan('matlab', '9.6') % 9.5 is Matlab 2018b, 9.6 is Matlab 2019a
    [K, attempts] = lqr_adapt(nx,nu,x_bar,u_hover,t1+1);
else
    load('lqr_results', 'K');
end

% integration output times
% t_extra: extra simulation time after arms have gotten to desired place
t_extra = 2.5;  
t2 = t1 + t_extra;
t_inc = (t2-t0)/300;    % timespan increment for output data
T = [t0:t_inc:t2]';     % timepoints for integration

% control for Stage II
v_bar = @(t,x) portion(u_hover(t,x),1,11);
v = @(t,x) v_bar(t,x) - K*(x - x_bar); % eq. 14
u_2 = @(t,x) [v(t,x); taug4(t)];

%% STAGES I and II
% combined control for Stages I and II
u = @(t,x) (t<=t1)*u_1(t,x) + (t>t1)*u_2(t,x);

% integrate the dynamics
[~,X] = ode45(@(t,x) dx(t,x,u,const),T,x0);
X = X';   % transpose X, so each column becomes the state at a given time
U = [];   % control at each time output, filled in the loop below
for j=1:length(T)   
    U = [U, u(T(j),X(:,j))];
end

% did the arm lengths go beyond their limits?
d_max_undershoot = dmax - max(max(X(11:14,:))); % positive value is good
d_min_overshoot = min(min(X(11:14,:))) - dmin;  % positive value is good

% times for snapshot plot
T_snapshots = linspace(0,.6,10)';

%% plots
% animation
%draw_quadrotor(X(:,end),const); % 2D plot
animate_quadrotor(T,X,const);

% generate Fig. 3 & Fig. 4
plot_snapshots(X,T,T_snapshots,const); % Figure 3
figure;
plot_x_u(T,X,U); % Figure 4

% to save a figure, first make the plot window as large as possible, then
% run the line below:
% print('-opengl','-dpng','-r600','myPngFile')
