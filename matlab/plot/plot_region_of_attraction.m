function plot_region_of_attraction(pitch_mat, roll_mat, roa)
% INPUTS:
% pitch_mat: matrix of pitch values
% roll_mat: matrix of roll values
% roa: matrix of 0's and 1's (0 = unstable, 1 = stable), in which the
%      elements corresponds to the those in pitch_mat and roll_mat

figure;
hold on;

%% plot results of simulations
Rz = @(ang) [cos(ang), -sin(ang), 0;
             sin(ang),  cos(ang), 0;
             0,                0, 1];
Ry = @(ang) [ cos(ang), 0, sin(ang);
             0,         1,        0;
             -sin(ang), 0, cos(ang)];
Rx = @(ang) [1,        0,         0;
             0, cos(ang), -sin(ang);
             0, sin(ang), cos(ang)];

dot_size = 20;
h1 = plot(pitch_mat(roa==1),roll_mat(roa==1),...
    'LineStyle','none','Marker','.','MarkerEdgeColor',...
     clr('light_green'),'MarkerSize',dot_size); % stable
h2 = plot(pitch_mat(roa==0),roll_mat(roa==0),...
    'LineStyle','none','Marker','.','MarkerEdgeColor','r',...
    'MarkerSize',dot_size); % unstable

%% plot Euler angle singularity
% theta = pitch, phi = roll
n_bnd = 400;    % # of thetas in phi singularity "band"
angle_bnd = linspace(-pi/2,pi/2,n_bnd); % sampled angles "band"
% 3-2-1 Euler angles have singularity when theta (pitch) is pi/2 or -pi/2

% this code was used to generate the original plot, it is wrong!
%h3 = plot(angle_bnd,(pi/2)*ones(length(angle_bnd),1),'b-','LineWidth',3);
%h4 = plot(angle_bnd,(-pi/2)*ones(length(angle_bnd),1),'b-','LineWidth',3);

% this is the correct code
h3 = plot((pi/2)*ones(length(angle_bnd),1),angle_bnd,'b-','LineWidth',3);
h4 = plot((-pi/2)*ones(length(angle_bnd),1),angle_bnd,'b-','LineWidth',3);


%% labels
set(gca,'TickLabelInterpreter','latex')

xticks([-pi/2 -pi/4 0 pi/4 pi/2]);
xticklabels({
    '$-\frac{\pi}{2}$',
    '$-\frac{\pi}{4}$',
    '0',
    '$\frac{\pi}{4}$',
    '$\frac{\pi}{2}$'});
yticks([-pi/2 -pi/4 0 pi/4 pi/2]);

yticklabels({
    '$-\frac{\pi}{2}$',
    '$-\frac{\pi}{4}$',
    '0',
    '$\frac{\pi}{4}$',
    '$\frac{\pi}{2}$'});
xtl = get(gca,'XTickLabel');
set(gca,'XTickLabel',xtl,'FontSize',18);
ytl = get(gca,'XTickLabel');
set(gca,'XTickLabel',ytl,'FontSize',18);

xlabel('pitch \theta (rad)','FontSize',14);
ylabel('roll \phi (rad)','FontSize',14);
degree_symbol = sprintf('%c', char(176));
lgd = legend([h1,h2,h3],{'stable','unstable','Euler angles singularity'},...
    'FontSize',12,...
    'Location','east');
axis equal;

% this is the only way I can make it look nice
ax = gca;
ax.Position = ax.Position + [0 0 -.4 0];

% run the commented line below to save image:
%print('-opengl','-dpng','-r300','region_of_attraction');

end