%% parameters for the hover reconfiguration
load_constants; % run script

% states, controls, times
nq = 18;            % number of generalized coordinates
nx = 2*nq;          % number of states
nu = 12;            % number of control inputs
nv = 11;            % number of reduced control inputs
t0 = 0;
t1 = .25;           % time when arms are where they should be

% hover parameters
f4 = @(t) 0;
f1 = @(t) (t<=t1)*0 + (t>t1)*(mt*g/4);
f2 = @(t) (t<=t1)*0 + (t>t1)*(mt*g/2);
f3 = @(t) (t<=t1)*0 + (t>t1)*(mt*g/4);
d24 = 1.05*l;
d13 = 1.95*l;
%dh = @(t) (d13*d24*g*mt)/(d13*(g*mt - 4*f4(t)) + 4*d24*f4(t)); % why?
dh = .1365; % results in final alpha of about 51 degrees

% alpha and d as functions of time
sinah = @(t) dh*(mt - 4*f4(t)/g)/(d13*(mc + 4*ma) + 2*l*(mb - ma)); %eq. 12
ah = @(t) asin(sinah(t));
ah_f = ah(t1);
dh_f = dh;

% eq. 11, hover feasibility:
% l*mt <= (dmax*(mc + 4*ma) + 2*l*(mb - ma))*sin(ah_f)

% arm torques
taua1 = @(t) -(c2/c1)*f1(t);     % "tau alpha 1"
taua2 = @(t) (c2/c1)*f2(t);
taua3 = @(t) -(c2/c1)*f3(t);
taua4 = @(t) (c2/c1)*f4(t);

% propeller torques
taug1 = @(t) -(c2/c1)*f1(t);     % "tau gamma 1"
taug2 = @(t) (c2/c1)*f2(t);
taug3 = @(t) -(c2/c1)*f3(t);
taug4 = @(t) (c2/c1)*f4(t);

% propeller speeds
dg1 = @(t) -sqrt((1/c1)*f1(t));     % "d gamma_1 squared",
dg2 = @(t) sqrt((1/c1)*f2(t));      % units of 1/(10^6) [rad/(s^2)]
dg3 = @(t) -sqrt((1/c1)*f3(t));
dg4 = @(t) sqrt((1/c1)*f4(t));

% state and control for hover equilibrium
x_bar = [0; 0; 0;  % rc (X, Y, Z)
      0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
      pi/4 - ah_f; pi/2 + pi/4; pi + pi/4 + ah_f; 3*pi/2 + pi/4;  % alpha
      d13; dh_f; d13; dh_f;     % d
      0; 0; 0; 0;       % gamma (these are arbitrary)
      0; 0; 0;          % rc dot
      0; 0; 0;          % eta dot
      0; 0; 0; 0;       % alpha dot
      0; 0; 0; 0;       % d dot
      dg1(t1+1); dg2(t1+1); dg3(t1+1); dg4(t1+1)];  % gamma dot

u_hover = @(t,x) [taua1(t); taua2(t); taua3(t); taua4(t);% arm rot. torques
                  0; 0; 0; 0;                            % arm slide forces
                  taug1(t); taug2(t); taug3(t); taug4(t)];  % rotor torques                
dx_bar = dx(t1+1,x_bar,u_hover,const);