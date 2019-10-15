function draw_quadrotor_flat_figure(x,const)
% draw quadrotor at state x in a top-down, body-fixed frame

% quadrotor body parameters
n = 18; % number of generalized coordinates
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
[X,Y,Z,Psi,Tht,Phi,a1,a2,a3,a4,d1,d2,d3,d4] = qCell{:};
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

% arm to body frame rotation matrices
Rz = @(ang) [cos(ang), -sin(ang), 0;
             sin(ang),  cos(ang), 0;
                    0,         0, 1];
RC1B = Rz(a1);
RC2B = Rz(a2);
RC3B = Rz(a3);
RC4B = Rz(a4);

% rotor positions in arm frame
rCr1 = d1*e1;
rCr2 = d2*e1;
rCr3 = d3*e1;
rCr4 = d4*e1;

% rotor positions in body frame (also positions of end of distal arm)
rBr1 = RC1B*rCr1;
rBr2 = RC2B*rCr2;
rBr3 = RC3B*rCr3;
rBr4 = RC4B*rCr4;

% component colors
cCol = [0 0 1]; % color of main body
aCol = [0 0 0]; % color of proximal arm
bCol = [.5 .5 .5]; % color of distal arm
rCol = [1 0 0]; % color of rotor

%% draw distal arms
% distal arm 1
rect.faces = [1 2 3 4]; % connect all 4 vertices
% rectangle of distal arm 1 on x-axis, each column is a vertex
vert1x = [d1-l, d1-l, d1, d1;
          -ab, ab, ab, -ab];
vert2x = [d2-l, d2-l, d2, d2;
          -ab, ab, ab, -ab];
vert3x = [d3-l, d3-l, d3, d3;
          -ab, ab, ab, -ab];     
vert4x = [d4-l, d4-l, d4, d4;
          -ab, ab, ab, -ab]; 
      
% convert vertices to body frame
% note: RC1B(1:2,1:2) is the 2D version of RC1B, i.e. it doesn't consider z
% components
vert1 = (RC1B(1:2,1:2)*vert1x);
vert2 = (RC2B(1:2,1:2)*vert2x);
vert3 = (RC3B(1:2,1:2)*vert3x);
vert4 = (RC4B(1:2,1:2)*vert4x);

% below: vert1' is used becuase patch takes row vectors
patch(rect,'Vertices',vert1','FaceColor',bCol,'EdgeColor','none');
patch(rect,'Vertices',vert2','FaceColor',bCol,'EdgeColor','none');
patch(rect,'Vertices',vert3','FaceColor',bCol,'EdgeColor','none');
patch(rect,'Vertices',vert4','FaceColor',bCol,'EdgeColor','none');

%% draw proximal arms
% proximal arm 1
rect.faces = [1 2 3 4]; % connect all 4 vertices

% rectangle of proximal arm (this is the same for all 4 arms) on x-axis,
% each column is a vertex
vertx = [0, 0, l, l;
         -aah, aah, aah, -aah];

% convert vertices to body frame
% note: RC1B(1:2,1:2) is the 2D version of RC1B, i.e. it doesn't consider z
% components
vert1 = (RC1B(1:2,1:2)*vertx);
vert2 = (RC2B(1:2,1:2)*vertx);
vert3 = (RC3B(1:2,1:2)*vertx);
vert4 = (RC4B(1:2,1:2)*vertx);

% below: vert1' is used becuase patch takes row vectors
patch(rect,'Vertices',vert1','FaceColor',aCol);
patch(rect,'Vertices',vert2','FaceColor',aCol);
patch(rect,'Vertices',vert3','FaceColor',aCol);
patch(rect,'Vertices',vert4','FaceColor',aCol);

%% draw center body
hold on;
rectangle('Position',[-ac -ac 2*ac 2*ac],'Curvature',[1 1],...
    'FaceColor',cCol,'EdgeColor','none');

%% draw rotors
% the "rectangle" command will actually draw a circle with the option
% "'Curvature',[1 1]"
rectangle('Position',[rBr1(1)-ar, rBr1(2)-ar, 2*ar, 2*ar],...
    'Curvature',[1 1],'FaceColor',rCol,'EdgeColor','none');
rectangle('Position',[rBr2(1)-ar, rBr2(2)-ar, 2*ar, 2*ar],...
    'Curvature',[1 1],'FaceColor',rCol,'EdgeColor','none');
rectangle('Position',[rBr3(1)-ar, rBr3(2)-ar, 2*ar, 2*ar],...
    'Curvature',[1 1],'FaceColor',rCol,'EdgeColor','none');
rectangle('Position',[rBr4(1)-ar, rBr4(2)-ar, 2*ar, 2*ar],...
    'Curvature',[1 1],'FaceColor',rCol,'EdgeColor','none');

%% draw props
draw_prop_2D(x(33),x(7)+x(15),rBr1,ap);
draw_prop_2D(x(34),x(8)+x(16),rBr2,ap);
draw_prop_2D(x(35),x(9)+x(17),rBr3,ap);
draw_prop_2D(x(36),x(10)+x(18),rBr4,ap);

end