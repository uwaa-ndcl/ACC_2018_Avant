% make animations for different arm motions
clear all;

% constants
load_constants; % run script

% mode: either 'rotation', 'extension', or 'hover_configurations'
mode = 'hover_configurations';

%% draw arm rotation
if strcmp(mode, 'rotation')
    % initial state
    x0 = [0; 0; 0;  % rc (X, Y, Z)
          0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
          pi/4 - .6; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;  % alpha
          dmid; dmid; dmid; dmid;     % d
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);  % gamma
          0; 0; 0;      % rc dot
          0; 0; 0;      % eta dot
          0; 0; 0; 0;   % alpha dot
          0; 0; 0; 0;   % d dot
          0; 0; 0; 0];  % gamma dot

    % final state
    xf = [0; 0; 0;  % rc (X, Y, Z)
          0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
          pi/4 + .6; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;  % alpha
          dmid; dmid; dmid; dmid;     % d
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);  % gamma
          0; 0; 0;      % rc dot
          0; 0; 0;      % eta dot
          0; 0; 0; 0;   % alpha dot
          0; 0; 0; 0;   % d dot
          0; 0; 0; 0];  % gamma dot

    x_all = [x0, xf];
    i_all = [0 1];
    n_frames = 60;
    i_samp = linspace(0,1,n_frames);
    X_forwards = interp1(i_all,x_all',i_samp);
    X_forwards = X_forwards';

    X_backwards = fliplr(X_forwards);
    X_backwards(:,1) = [];% 1st column is same as last column of X_forwards
    X = [X_forwards, X_backwards];
    X(:,end) = []; % last column is same as first
    animate_quadrotor(i_samp,X,const);
end

%% draw arm extension
if strcmp(mode, 'extension')
    x0 = [0; 0; 0;  % rc (X, Y, Z)
          0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
          pi/4; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;          % alpha
          dmid - .3*(dmax - dmin); dmid; dmid; dmid;            % d
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);   % gamma
          0; 0; 0;      % rc dot
          0; 0; 0;      % eta dot
          0; 0; 0; 0;   % alpha dot
          0; 0; 0; 0;   % d dot
          0; 0; 0; 0];  % gamma dot

    % final state
    xf = [0; 0; 0;  % rc (X, Y, Z)
          0; 0; 0; % Euler angles: yaw (psi) - pitch (theta) - roll (phi)
          pi/4; pi/2 + pi/4; pi + pi/4; 3*pi/2 + pi/4;          % alpha
          dmid + .3*(dmax - dmin); dmid; dmid; dmid;            % d
          deg2rad(75); deg2rad(87); deg2rad(20); deg2rad(35);   % gamma
          0; 0; 0;      % rc dot
          0; 0; 0;      % eta dot
          0; 0; 0; 0;   % alpha dot
          0; 0; 0; 0;   % d dot
          0; 0; 0; 0];  % gamma dot

    % draw arm extension
    x_all = [x0, xf];
    i_all = [0 1];
    n_frames = 60;
    i_samp = linspace(0,1,n_frames);
    X_forwards = interp1(i_all,x_all',i_samp);
    X_forwards = X_forwards';

    X_backwards = fliplr(X_forwards);
    X_backwards(:,1) = []; %1st column is same as last column of X_forwards
    X = [X_forwards, X_backwards];
    X(:,end) = []; % last column is same as first
    animate_quadrotor(i_samp,X,const);
end

%% draw hover configurations
if strcmp(mode, 'hover_configurations')
    % hover parameters
    tf = 1;
    f4 = @(t) ((tf-t)/tf)*mt*g/4;
    f1 = @(t) mt*g/4;
    f2 = @(t) mt*g/2 - f4(t);
    f3 = @(t) mt*g/4;
    dmin = 1*l;
    dmax = 2*l;
    d24 = 1.05*l;
    d13 = 1.95*l;
    dh = @(t) (d13*d24*g*mt)/(d13*(g*mt - 4*f4(t)) + 4*d24*f4(t)); % why?

    sinah = @(t) dh(t)*(mt - 4*f4(t)/g)/(d13*(mc + 4*ma) + 2*l*(mb - ma));
    ah = @(t) asin(sinah(t));
    
    % hover feasibility
    % l*mt <= (dmax*(mc + 4*ma) + 2*l*(mb - ma))*sin(ah_f)

    taua1 = @(t) -(c2/c1)*f1(t);     % "tau alpha 1"
    taua2 = @(t) (c2/c1)*f2(t);
    taua3 = @(t) -(c2/c1)*f3(t);
    taua4 = @(t) (c2/c1)*f4(t);

    dg1 = @(t) -sqrt((1/c1)*f1(t));     % "d gamma_1 squared",
    dg2 = @(t) sqrt((1/c1)*f2(t));      % units of 1/(10^6) [rad/(s^2)]
    dg3 = @(t) -sqrt((1/c1)*f3(t));
    dg4 = @(t) sqrt((1/c1)*f4(t));

    n_frames = 60;
    i_samp = linspace(0,1,n_frames);
    t_samp = linspace(0,tf,n_frames);
    X_forwards = [];
    for i = 1:n_frames
        t_i = t_samp(i);
        x_i = [0; 0; 0;  % rc (X, Y, Z)
              0; 0; 0; % Euler angs: yaw (psi) - pitch (theta) - roll (phi)
              pi/4 - ah(t_i);  % alpha
              pi/2 + pi/4;
              pi + pi/4 + ah(t_i);
              3*pi/2 + pi/4;
              d13; dh(t_i); d13; dh(t_i);     % d
              0; 0; 0; 0;   % gamma (these are arbitrary)
              0; 0; 0;      % rc dot
              0; 0; 0;      % eta dot
              0; 0; 0; 0;   % alpha dot
              0; 0; 0; 0;   % d dot
              dg1(t_i); dg2(t_i); dg3(t_i); dg4(t_i)];  % gamma dot

        X_forwards = [X_forwards, x_i];
    end
    size(X_forwards)

    % draw
    X_backwards = fliplr(X_forwards);
    X_backwards(:,1) = []; %1st column is same as last column of X_forwards
    X = [X_forwards, X_backwards];
    X(:,end) = []; % last column is same as first
    animate_quadrotor(i_samp,X,const);
end
