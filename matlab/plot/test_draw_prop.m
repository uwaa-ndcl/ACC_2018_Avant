% test drawing of a propeller

% standard rotation matrices
Rz = @(ang) [cos(ang) -sin(ang) 0;
             sin(ang) cos(ang)  0;
             0        0         1]; % rotate around z-axis
Ry = @(ang) [cos(ang) 0 -sin(ang);
             0        1 0;
             sin(ang) 0 cos(ang)]; % rotate around y-axis
Rx = @(ang) [1 0        0
             0 cos(ang) -sin(ang);
             0 sin(ang) cos(ang)]; % rotate around x-axis

% rotation matrix with respect to inertial frame
RDiA = Rz(deg2rad(5))*Rz(deg2rad(15))*Rz(deg2rad(10));

% speed and draw
gamma_dot = 4;
pos = [0; 0; 0]; % location in space of prop center
prop_rad = .0889; % 7"-diameter prop
draw_prop(gamma_dot,RDiA,pos,prop_rad,'CCW');
axis equal;