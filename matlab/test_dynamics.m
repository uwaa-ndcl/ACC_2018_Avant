% simple example to show dynamics
clear all;

% reminder:
% x(1:18) = [X,Y,Z,Psi,Theta,Phi,al1,al2,al3,al4,d1,d2,d3,d4,g1,g2,g3,g4]
% (18 states), Psi = yaw, Theta = pitch, Phi = roll
% u = [arm torques 1-4, arm length forces 1-4, rotor torques 1-4]
% (12 inputs)

% constants
load_constants; % run script

% system variables
nq = 18;            % number of generalized coordinates
nx = 2*nq;          % number of states
nu = 12;            % number of control inputs

% initial state
g_dot_hov = sqrt((mt*g/4)/c1); % gamma dot at hover
x0 = [0; 0; 0;  % rc (X, Y, Z)
      0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
      pi/4; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;  % alpha
      dmid; dmid; dmid; dmid;   % d
      .4; .4; .4; .4;           % gamma (these are arbitrary)
      0; 0; 0;                  % rc dot
      0; 0; 0;                  % eta dot
      0; 0; 0; 0;               % alpha dot
      0; 0; 0; 0;               % d dot
      0; 0; 0; 0];              % gamma dot

% thrust forces for hover (Newtons)
f1 = @(t) mt*g/4;
f2 = @(t) mt*g/4;
f3 = @(t) mt*g/4;
f4 = @(t) mt*g/4;

% thrust forces for going up and down in cycles
% (pointless: just an example)
om = .8*pi;  % angular frequency
amp = .4;      % amplitude
sinusoid = @(t) (1 + amp*cos(om*t));
f1 = @(t) sinusoid(t)*f1(t);
f2 = @(t) sinusoid(t)*f2(t);
f3 = @(t) sinusoid(t)*f3(t);
f4 = @(t) sinusoid(t)*f4(t);

% arm torques
taua1 = @(t) -(c2/c1)*f1(t);    % "tau alpha 1"
taua2 = @(t) (c2/c1)*f2(t);
taua3 = @(t) -(c2/c1)*f3(t);
taua4 = @(t) (c2/c1)*f4(t);

% arm extension forces
fd1 = @(t) 0;                   % "f d 1"
fd2 = @(t) 0;
fd3 = @(t) 0;
fd4 = @(t) 0;

% rotor torques
taug1 = @(t) -(c2/c1)*f1(t);    % "tau gamma 1"
taug2 = @(t) (c2/c1)*f2(t);
taug3 = @(t) -(c2/c1)*f3(t);
taug4 = @(t) (c2/c1)*f4(t);

% full control input
u = @(t,x) [taua1(t); taua2(t); taua3(t); taua4(t);  % arm rotation torques
            fd1(t); fd2(t); fd3(t); fd4(t);          % arm slide forces
            taug1(t); taug2(t); taug3(t); taug4(t)]; % rotor torques

% integrate
t0 = 0;
t1 = 5;
T = linspace(t0,t1,100); % times to save integration results
[~,X] = ode45(@(t,x) dx(t,x,u,const),T,x0);
X = X'; % make each column represent the state at a time

% determine control at timepoints
for i=1:length(T)
    U(:,i) = u(T(i),X(:,i));
end

% run these commented lines for various plots
%plot_x_u(T,X,U);
%draw_quadrotor(X(:,1),const);
%animate_quadrotor(T,X,const);
%T_snapshots = linspace(t0,t1,2)'; % times to show snapshots
%plot_snapshots(X,T,T_snapshots,const);