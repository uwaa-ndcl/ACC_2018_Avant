function plot_snapshots(X,T,Tvals,const)
% plot multiple views in time on one plot

% set all rotor 4 velocities except 1st to NaN,
% so that the rotor won't be plotted
X(36,1) = X(34,1); % draw failed rotor with initial hover thrust
X(36,2:end) = NaN;

% find values of T closest to each Tvals entry
for i=1:length(Tvals)
    [min_i(i) ind_i(i)] = min(abs(T-Tvals(i)));
    T_i(i) = T(ind_i(i));
    X_i(:,i) = X(:,ind_i(i));
end

f = figure;

% find min and max of 3D space over all time, and set axis accordingly
max_d = max(max(abs(X(11:14,:)))); % the max abs. value of rotor distances
% find min and max of 3D space over all time, and set axis accordingly
xmin = min(X(1,:));
xmax = max(X(1,:));
ymin = min(X(2,:));
ymax = max(X(2,:));
zmin = min(X(3,:));
zmax = max(X(3,:));
xlim([xmin - max_d - 2*const.ar, xmax + max_d + 2*const.ar]);
ylim([ymin - max_d - 2*const.ar, ymax + max_d + 2*const.ar]);
zlim([zmin - max_d - 2*const.ar, zmax + max_d + 2*const.ar]);

% aximuth and elevation for all plots
az = 0;
el = 35;
xdir_shift = .5;% increment for plots in x-direction, so they don't overlap

% plot
axis equal;
view(az, el);
axis off;
hold on;
for i=1:length(Tvals)
    X_shift = [xdir_shift*i; zeros(36-1,1)];
    draw_quadrotor(X_i(:,i) + X_shift,const);
end

end