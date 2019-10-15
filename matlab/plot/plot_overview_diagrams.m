% make diagrams for Figure 1 and Figure 2 in the ACC paper
clear all;

% constants
load_constants; % run script

% mode for the 3D plot:
% options: 'initial', 'narrow', 'hover', 'flipper', 'hover_with_dead'
mode = 'initial';

% initial state
if strcmp(mode, 'initial')
    x0 = [0; 0; 0;  % rc (X, Y, Z)
          0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
          pi/4; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;  % alpha
          dmid; dmid; dmid; dmid;     % d
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);  % gamma
          0; 0; 0;      % rc dot
          0; 0; 0;      % eta dot
          0; 0; 0; 0;   % alpha dot
          0; 0; 0; 0;   % d dot
          0; 0; 0; 0];  % gamma dot
end

% narrow
if strcmp(mode, 'narrow')
    x0 = [0; 0; 0;
          0; .4; 0;
          pi/4 - .2; pi/2 + pi/4 + .2; pi + pi/4 - .2; 3*pi/2 + pi/4 + .2;
          dmid; dmid; dmid; dmid;
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);
          0; 0; 0;
          0; 0; 0;
          0; 0; 0; 0;
          0; 0; 0; 0;
          500; 2200; 2200; 500];
end

% hover
if strcmp(mode, 'hover')
    x0 = [0; 0; 0;
          0; 0; 0;
          pi/4; pi/2 + pi/4 - .4; pi + pi/4; 3*pi/2 + pi/4 + .4;
          dmid - .3*(dmax-dmin);
          dmid + .4*(dmax-dmin);
          dmid - .3*(dmax-dmin);
          dmid + .4*(dmax-dmin);
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);
          0; 0; 0;
          0; 0; 0;
          0; 0; 0; 0;
          0; 0; 0; 0;
          100; 1700; 9000; 1700];
end

% flipper
if strcmp(mode, 'flipper')
    x0 = [0; 0; 0;
          0; .4; -.4;
          pi/4; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;
          dmid; dmid; dmid; dmid;
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);
          0; 0; 0;
          0; 0; 0;
          0; 0; 0; 0;
          0; 0; 0; 0;
          nan; 1800; 1800; 1800];
end

% hover with dead
if strcmp(mode, 'hover_with_dead')
    x0 = [0; 0; 0;
          0; 0; 0;
          pi/4; pi/2 + pi/4 - .6; pi + pi/4; 3*pi/2 + pi/4 + .6;
          dmid - .3*(dmax-dmin);
          dmid + .4*(dmax-dmin);
          dmid - .3*(dmax-dmin);
          dmid + .4*(dmax-dmin);
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);
          0; 0; 0;
          0; 0; 0;
          0; 0; 0; 0;
          0; 0; 0; 0;
          nan; 1700; 9000; 1700];
end

%% draw first figure: system overview (3D)
figure;
axis equal;
axis tight;
ax = gca;
ax.Position = [0 0 1 1];
ax.Visible = 'off';
az = 55;
el = 35;
view(az,el);
draw_quadrotor(x0,const);

%% draw second figure: 2D hover diagram
a13 = deg2rad(45);

f1 = mt*g/4;
f2 = mt*g/2;
f3 = mt*g/4;
f4 = 0;

d24 = 1.05*l;
d13 = 1.95*l;
dh = (d13*d24*g*mt)/(d13*(g*mt - 4*f4) + 4*d24*f4);
sinah = dh*(mt - 4*f4/g)/(d13*(mc + 4*ma) + 2*l*(mb - ma));
ah = asin(sinah);

dg1 = -sqrt((1/c1)*f1);% "d gamma_1 squared", units of 1/(10^6) [rad/(s^2)]
dg2 = sqrt((1/c1)*f2);
dg3 = -sqrt((1/c1)*f3);
dg4 = sqrt((1/c1)*f4);

% hover state
x0 = [0; 0; 0;  % rc (X, Y, Z)
      0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
      -a13; pi/2; pi + a13; 3*pi/2;          % alpha
      d13; d24; d13; d24;                                   % d
      deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);   % gamma
      0; 0; 0;              % rc dot
      0; 0; 0;              % eta dot
      0; 0; 0; 0;           % alpha dot
      0; 0; 0; 0;           % d dot
      dg1; dg2; dg3; dg4];  % gamma dot

% draw the figure
figure;
axis equal;
ax = gca;
ax.Visible = 'off';
draw_quadrotor_flat_figure(x0,const);

% for saving, first make the plot window as large as possible, then
% run the line below:
% print('-opengl','-dpng','-r600','myPngFile')