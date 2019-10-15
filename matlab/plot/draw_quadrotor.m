function draw_quadrotor(x,const)
% draws a 3D quadrotor at the state x

% quadrotor body parameters
n = 18;         % number of generalized coordinates
ac = const.ac;
hc = const.hc;
l = const.l;
aal = const.aal;
aah = const.aah;
aa = sqrt(const.aal^2 + const.aah^2);
ab = const.ab;
ar = const.ar;
hr = const.hr;
ap = const.ap;

% convert x vector to cell array so entries can easily be reassigned
q = x(1:n);
qCell = num2cell(q);
[X,Y,Z,Psi,Tht,Phi,a1,a2,a3,a4,d1,d2,d3,d4,g1,g2,g3,g4] = qCell{:};
qDot = x(n+1:2*n);
qDotCell = num2cell(qDot);
[~,~,~,~,~,~,~,~,~,~,~,~,~,~,dg1,dg2,dg3,dg4] = qDotCell{:};
rc = [X; Y; Z];

% unit vectors
e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];

% default arm angles
al01 = pi/4;
al02 = pi + pi/4;
al03 = 2*pi + pi/4;
al04 = 3*pi + pi/4;

% counter-clockwise x,y,z-axis rotation matrices
Rz = @(ang) [cos(ang), -sin(ang), 0;
             sin(ang),  cos(ang), 0;
             0,                0, 1];
Ry = @(ang) [ cos(ang), 0, sin(ang);
             0,         1,        0;
             -sin(ang), 0, cos(ang)];
Rx = @(ang) [1,        0,         0;
             0, cos(ang), -sin(ang);
             0, sin(ang), cos(ang)];
         
% arm to body frame rotation matrices
RC1B = Rz(a1);
RC2B = Rz(a2);
RC3B = Rz(a3);
RC4B = Rz(a4);

% body to inertial frame rotation matrix
RBA = Rz(Psi)*Ry(Tht)*Rx(Phi);

% arm frame vectors
rCaiE = l*e1;           % arm frame, end of inner arm
rCb1B = (d1 - l)*e1;    % arm frame, beginning of outer arm
rCb2B = (d2 - l)*e1;
rCb3B = (d3 - l)*e1;
rCb4B = (d4 - l)*e1;
rCr1 = d1*e1;           % body frame, rotor (and also end of outer arm)
rCr2 = d2*e1;
rCr3 = d3*e1;
rCr4 = d4*e1;

% inertia frame, end of inner arm
ra1E = rc + RBA*RC1B*rCaiE;
ra2E = rc + RBA*RC2B*rCaiE;
ra3E = rc + RBA*RC3B*rCaiE;
ra4E = rc + RBA*RC4B*rCaiE;

% inertia frame, beginnning of outer arm
rb1B = rc + RBA*RC1B*rCb1B;
rb2B = rc + RBA*RC2B*rCb2B;
rb3B = rc + RBA*RC3B*rCb3B;
rb4B = rc + RBA*RC4B*rCb4B;

% inertia frame, rotor (and also end of outer arm)
rr1 = rc + RBA*RC1B*rCr1;
rr2 = rc + RBA*RC2B*rCr2;
rr3 = rc + RBA*RC3B*rCr3;
rr4 = rc + RBA*RC4B*rCr4;

% coordinates of the top and bottom of main body and rotor cylinders
rc_top = rc + RBA*(hc/2)*e3;
rc_bot = rc + RBA*(-hc/2)*e3;
rr1_top = rr1 + RBA*(hr/2)*e3;
rr1_bot = rr1 + RBA*(-hr/2)*e3;
rr2_top = rr2 + RBA*(hr/2)*e3;
rr2_bot = rr2 + RBA*(-hr/2)*e3;
rr3_top = rr3 + RBA*(hr/2)*e3;
rr3_bot = rr3 + RBA*(-hr/2)*e3;
rr4_top = rr4 + RBA*(hr/2)*e3;
rr4_bot = rr4 + RBA*(-hr/2)*e3;

% define cylinders
np = 30;    % number of points for each cylinder

% draw cylinders
cCol = [0 0 1];     % color of main body
aCol = [0 0 0];     % color of inner arm
bCol = [.5 .5 .5];  % color of outer arm
rCol = [1 0 0];     % color of rotor

Cylinder(rc_top,rc_bot,ac,np,cCol,1,0);     % center body
Cylinder(rc,ra1E,aah,np,aCol,1,0);          % inner arm 1
Cylinder(rc,ra2E,aah,np,aCol,1,0);
Cylinder(rc,ra3E,aah,np,aCol,1,0);
Cylinder(rc,ra4E,aah,np,aCol,1,0);
Cylinder(rb1B,rr1,ab,np,bCol,1,0);          % outer arm 1
Cylinder(rb2B,rr2,ab,np,bCol,1,0);
Cylinder(rb3B,rr3,ab,np,bCol,1,0);
Cylinder(rb4B,rr4,ab,np,bCol,1,0);
Cylinder(rr1_top,rr1_bot,ar,np,rCol,1,0);   % rotor 1
Cylinder(rr2_top,rr2_bot,ar,np,rCol,1,0);
Cylinder(rr3_top,rr3_bot,ar,np,rCol,1,0);
Cylinder(rr4_top,rr4_bot,ar,np,rCol,1,0);

% propellers
% arm to body frame rotation matrices
RDiCi = @(ang) [cos(ang), -sin(ang), 0;
                sin(ang), cos(ang), 0;
                0, 0, 1];
RD1A = RBA*RC1B*RDiCi(g1);
RD2A = RBA*RC2B*RDiCi(g2);
RD3A = RBA*RC3B*RDiCi(g3);
RD4A = RBA*RC4B*RDiCi(g4);

% position of center of props
rp1 = rr1_top + RBA*(.2*hr)*e3;
rp2 = rr2_top + RBA*(.2*hr)*e3;
rp3 = rr3_top + RBA*(.2*hr)*e3;
rp4 = rr4_top + RBA*(.2*hr)*e3;

draw_prop(dg1,RD1A,rp1,ap,'CW');
draw_prop(dg2,RD2A,rp2,ap,'CCW');
draw_prop(dg3,RD3A,rp3,ap,'CW');

% don't draw rotor 4 if its velocity is a NaN
if ~isnan(dg4)
    draw_prop(dg4,RD4A,rp4,ap,'CCW');
end

end