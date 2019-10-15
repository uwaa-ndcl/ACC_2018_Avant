function draw_prop_2D(gamma_dot,aplusg,pos,prop_rad)
% INPUTS
% gamma_dot: signed angular velocity of propeller
% aplusg: alpha + gamma
% pos: position of center of propeller
% prop_rad: propeller radius in meters

% constants
pCol = clr('dark_purple');

% parameters which describe how to draw props:
% prop speeds larger than this will be drawn as a disk
gamma_dot_min = 1100;

% prop speeds larger than this will not result in darker disks
gamma_dot_max = 2100;

%% if propeller is going slow: draw it as two blades and a hub
gamma_dot = abs(gamma_dot);     % magnitude of propeller speed
if gamma_dot < gamma_dot_min
    
    % propeller blade as an ellipse
    x_a = prop_rad/2;   % length of x semi-axis
    y_a = .2*x_a;
    g1 = hgtransform;
    g2 = hgtransform;
    rectangle('Position',[-x_a,-y_a,2*x_a,2*y_a],...
        'Curvature',[1,1],'FaceColor',pCol,'EdgeColor','none','Parent',g1);
    rectangle('Position',[-x_a,-y_a,2*x_a,2*y_a],...
        'Curvature',[1,1],'FaceColor',pCol,'EdgeColor','none','Parent',g2);
    g1.Matrix = makehgtform('translate',...
        [pos(1)+x_a*cos(aplusg),pos(2)+x_a*sin(aplusg),0],...
        'zrotate',aplusg);
    g2.Matrix = makehgtform('translate',...
        [pos(1)+x_a*cos(aplusg+pi),pos(2)+x_a*sin(aplusg+pi),0],...
        'zrotate',aplusg+pi);
    
    % propeller hub as a cylinder
    scl = 12.5; % scale factor
    hub_rad = prop_rad/scl;
    g3 = hgtransform;
    r_hub = rectangle('Position',[-hub_rad,-hub_rad,2*hub_rad,2*hub_rad],...
        'Curvature',[1,1],'FaceColor',pCol,'EdgeColor','none','Parent',g3);
    g3.Matrix = makehgtform('translate',[pos(1),pos(2),0]);
    
    
%% if propeller is going fast: draw it as a disk
elseif gamma_dot >= gamma_dot_min
    
    % for drawing purposes, don't draw anything darker than gamma_dot_max
    gamma_dot = min(gamma_dot,gamma_dot_max);
    
    % interpolate
    alph = (gamma_dot-gamma_dot_min)/(gamma_dot_max-gamma_dot_min);

    g1 = hgtransform;
    rectangle('Position',[-prop_rad,-prop_rad,2*prop_rad,2*prop_rad],...
        'Curvature',[1,1],'FaceColor',[pCol, alph],...
        'EdgeColor','none','Parent',g1);
    g1.Matrix = makehgtform('translate',[pos(1),pos(2),0]);
    
end

end