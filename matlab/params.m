function const = params()
% constants in Table I of the paper
%
% define the quadrotor parameters as const, a structure
% default units: mass [kg], radius [m], height [m]

% gravity constant [m/s^2]
const.g = 9.81;

% thrust and torque coefficients
% GWS 5x4.3 (see propeller_calculations folder)
const.c1 = 1.2e-6;
const.c2 = 1.2e-8;
const.ap = .0635; % prop radius: 5"-diameter

% center body: mass (mc), radius (ac), and height (hc)
const.mc = .4;
const.ac = .05;
const.hc = .07;

% proximal and distal arms: length
const.l = .13;

% proximal arm: mass (ma), inner radius (aal), outer radius (aah)
const.ma = .06;
const.aal = .015;
const.aah = .022;

% distal arm: mass (mb) and radius (ab)
const.mb = .03;
const.ab = .014;

% rotor: mass (mr), radius (ar), and height (hr)
const.mr = .03;
const.ar = .015;
const.hr = .05;

% propeller inertia about rotational axis
const.Jpzz = 4e-6; % kg*m^2

end