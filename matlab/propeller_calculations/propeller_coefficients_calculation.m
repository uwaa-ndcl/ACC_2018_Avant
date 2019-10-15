% calculation of thrust and torque coefficients for example propellers
% the data is from the following database:
% http://m-selig.ae.illinois.edu/props/propDB.html
%
% we used "Volume 2" of the database:
% https://m-selig.ae.illinois.edu/props/volume-2/propDB-volume-2.html
% on this page, we found the thrust and power coefficients (CT and CP)
% by navigating to
% Volume 2 > "Propeller Name" > Performace Static CT0 and CP0
%
% the thrust, torque, and power coefficients equations are given by
% force:        T = rho*(n^2)*(D^4)*CT      (N)
% torque:       Q = rho*(n^2)*(D^5)*CQ      (N*m)
% CP to CQ:     CP = CQ*2*pi
%
% in these equations, n is assumed to be in units of rev/sec
% we instead define the angular velocity as omega, which has units of
% rad/sec:
% n = (rev2rad)*omega                       (rev2rad = 1/(2*pi))
%
% then, the forces and torque are
% force:    T = rho*(rev2rad)^2*(omega^2)*(D^4)*CT
% torque:   Q = rho*(rev2rad)^2*(omega^2)*(D^5)*CQ
%
% in our paper, we define the thrust and torque coefficients to be
% proportional to the square of the angular velocity of the propeller:
% T = c1*(omega^2)
% Q = c2*(omega^2)
%
% so we have
% c1 = rho*(rev2rad)^2*(D^4)*CT             (N*s^2/rad^2)
% c2 = rho*(rev2rad)^2*(D^5)*CQ             (N*m*s^2/rad^2)

clear all;

% constants and functions
rev2rad_sq = (1/(2*pi))^2;      % ratio of revolutions^2 to radians^2
rho = 1.225;                    % density of air, kg/m^3
inch2m = .0254;                 % inches to meters
CQ_of_CP = @(CP) CP/(2*pi);     % torque coeff. in terms of power coeff.
c1 = @(D, CT) rho*(D^4)*CT*rev2rad_sq;
c2 = @(D, CP) rho*(D^5)*CQ_of_CP(CP)*rev2rad_sq;

% GWS 5x4.3
D = inch2m*5; % diameter, in meters
CT0 = .15;
CP0 = .075;

% calculate c1 and c2
c1(D, CT0)
c2(D, CP0)

%{
% APC Free Flight 9x4:
D = inch2m*9;
CT0 = .09;
CP0 = .04;

% APC Sport 9x6
D = inch2m*9;
CT0 = .12;
CP0 = .06;

% APC Free Flight 4.2x4
D = inch2m*4.2;
CT0 = .13;
CP0 = .11;

% GWS 9x5
D = inch2m*9;
CT0 = .11;
CP0 = .04;

% GWS 5x3
D = inch2m*5;
CT0 = .11;
CP0 = .04;
%}