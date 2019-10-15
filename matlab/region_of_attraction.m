% use LQR to stabilize the quadrotor from a perturbed initial state
clear all;

% reminder:
% x(1:18) = [X,Y,Z,Psi,Theta,Phi,al1,al2,al3,al4,d1,d2,d3,d4,g1,g2,g3,g4]
% (18 states), Psi = yaw, Theta = pitch, Phi = roll
% u = [arm torques 1-4, arm length forces 1-4, rotor torques 1-4]
% (12 inputs)

%% setup and initial conditions
load_hover_setup; % run script

%% LQR adapt
% Matlab's lqr command was changed after version 2018b, and my code won't
% work with the new version of the command. So, if the Matlab version is
% less than 2018a, load the lqr results rather than re-computing them.
if verLessThan('matlab', '9.6') % 9.5 is Matlab 2018b
    [K, attempts] = lqr_adapt(nx,nu,x_bar,u_hover,t1+1);
else
    load('lqr_results', 'K');
end

%% control (original)
% integration output times
% t_extra: extra simulation time after arms have gotten to desired place
t_extra = 2.5;  
t2 = t1 + t_extra;
t_inc = (t2-t0)/300; % timespan increment for output data
T = [t0:t_inc:t2]'; % timepoints for integration

% controls for Stage II
v_bar = @(t,x) portion(u_hover(t,x),1,11);
v = @(t,x) v_bar(t,x) - K*(x - x_bar);

u_2 = @(t,x) [v(t,x); taug4(t)];
u = @(t,x) u_2(t,x);

%% region of attraction: iterate over initial conditions
% roll and pitch sample points
n_incs = 20;            % number of increments
mx_roll = deg2rad(85);  % max roll to test
mx_pitch = mx_roll;     % max pitch to test
inc = 2*mx_roll/n_incs;
roll = [-mx_roll:inc:mx_roll]; % roll = phi
pitch = [-mx_pitch:inc:mx_pitch];

% first, turn some warnings into errors
warnID_1 = 'MATLAB:nearlySingularMatrix';
warnID_2 = 'MATLAB:ode45:IntegrationTolNotMet';
warnstate_1 = warning('error', warnID_1);
warnstate_2 = warning('error', warnID_2);

% integrate all initial conditions
% pitch_mat & roll_mat: matrices of pitch and roll values, entries
% correspond to those in xf_err and roa
pitch_mat = [];
roll_mat = [];
for i=1:length(pitch)
    pitch_i = pitch(i);
    for j=1:length(roll)
        roll_j = roll(j);
        pitch_mat(i,j) = pitch_i;
        roll_mat(i,j) = roll_j;
        x0 = x_bar + [0; 0; 0; 0; pitch_i; roll_j; zeros(30,1)];
        
        % try to integrate
        try
            [~,X] = ode45(@(t,x) dx(t,x,u,const),T,x0);
            X = X'; % transpose X, so each column is one state
        catch ME % catch integration failure
            X = NaN*ones(nx,length(T)); % fill X with NaNs
            errs_roa{i,j} = ME.identifier;  % save error
        end
        
        X_roa(:,:,i,j) = X;
    end
    disp(['processed ', num2str(i*j), ...
          ' of ', num2str(length(pitch)*length(roll)), ' samples'])
end

% norms of final position and attitude errors (xf_err)
xf_err = zeros(length(pitch),length(roll));
for i=1:length(pitch)
    for j=1:length(roll)
        X_ij = X_roa(:,:,i,j);
        xf_err(i,j) = norm(X_ij(1:6,end)-x_bar(1:6));
    end
end

% run the commented lines below to save/load results
%save('region_of_attraction_results.mat', 'errs_roa', 'pitch_mat', 'roll_mat', 'xf_err')
%load('region_of_attraction_results.mat')

% determine whether each integration converged
% if integration diverged, then the corresponding element of xf_err will be
% NaN
roa = not(isnan(xf_err)); % stable points, 1=stable, 0=unstable

%% plot
plot_region_of_attraction(pitch_mat, roll_mat, roa);