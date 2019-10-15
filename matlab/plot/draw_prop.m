function draw_prop(gamma_dot,RDiA,pos,prop_rad,rot_dir)
% INPUTS
% gamma_dot: signed angular velocity of propeller
% RDiA: rotation matrix from propeller frame to inertial frame
% pos: position of center of propeller
% prop_rad: propeller radius in meters
% rot_dir: direction propeller is spinning, 'CW' or 'CCW'

% functions
e3 = [0; 0; 1];
% Rx: counter-clockwise rotate around x-axis
Rx = @(ang) [1,        0,        0;
             0, cos(ang), -sin(ang);
             0, sin(ang), cos(ang)];

% constants
prop_thck = .01;
pCol = [1 0 1];     % propeller color
pCol = clr('dark_purple');

% parameters which describe how to draw props
% prop speeds larger than this will be drawn as a disk
gamma_dot_thresh = 10;

% prop speeds smaller than this will not result in lighter disks
gamma_dot_disk_lo = 1500;

% prop speeds larger than this will not result in darker disks
gamma_dot_disk_hi = 2000;

trans_lo = .15; % minimum transparency of disks
trans_hi = .6;  % maximum transparency of disks

%% if propeller is going slow: draw it as two blades and a hub
gamma_dot = abs(gamma_dot);     % magnitude of propeller speed
if gamma_dot < gamma_dot_thresh
    
    % propeller blade as an ellipsoid
    n = 50;             % points in mesh
    x_a = prop_rad/2;   % length of x semi-axis
    y_a = .2*x_a;       % length of y semi-axis
    z_a = .001*x_a;     % length of z semi-axis
    
     % unrotated/untranslated blade
    [Xb,Yb,Zb] = ellipsoid(0,0,0,x_a,y_a,z_a,n);
    
    % unrotated/untranslated propeller hub (drawn as a cylinder)
    [Xh,Yh,Zh] = cylinder(prop_rad/12.5,n); % unrotated/untranslated hub
    Zh = Zh - .5; % now it is centered at the origin
    Zh = prop_thck*Zh; % now it is the correct height
    
    % rotate and translate shapes
    shft = [x_a; 0; 0];   % center of desired location, translational shift
    fthr = deg2rad(25);   % feathering angle
    
    % if the propeller is turning counter-clockwise, make sure the blades
    % are angled the right way
    if strcmp(rot_dir,'CW')
        fthr = -fthr;
    end
    
    % translate and rotate blades
    for i=1:size(Xb,1)
        for j=1:size(Xb,2)
            % first blade
            pt_ij = [Xb(i,j); Yb(i,j); Zb(i,j)];
            pt_ij_1 = RDiA*Rx(fthr)*(pt_ij + shft);
            Xb1(i,j) = pos(1) + pt_ij_1(1);
            Yb1(i,j) = pos(2) + pt_ij_1(2);
            Zb1(i,j) = pos(3) + pt_ij_1(3);
            
            % second blade
            xyz_ij_2 = RDiA*Rx(-fthr)*(pt_ij - shft);
            Xb2(i,j) = pos(1) + xyz_ij_2(1);
            Yb2(i,j) = pos(2) + xyz_ij_2(2);
            Zb2(i,j) = pos(3) + xyz_ij_2(3);
        end
    end
    
    % translate and rotate hub
    for i=1:size(Xh,1)
        for j=1:size(Xh,2)
            pt_ij = [Xh(i,j); Yh(i,j); Zh(i,j)];
            pt_ij_m = RDiA*pt_ij;
            Xh(i,j) = pos(1) + pt_ij_m(1);
            Yh(i,j) = pos(2) + pt_ij_m(2);
            Zh(i,j) = pos(3) + pt_ij_m(3);
        end
    end
    
    % plot shapes
    bOpts = {'FaceColor',pCol,'EdgeColor','none'}; % blade plot options
    hOpts = {'FaceColor',pCol,'EdgeColor','none'}; % hub plot options
    surf(Xb1,Yb1,Zb1,bOpts{:}); % blade 1
    hold on;
    surf(Xb2,Yb2,Zb2,bOpts{:}); % blade 2
    mesh(Xh,Yh,Zh,hOpts{:}); % hub cylinder body
    patch(Xh(1,:),Yh(1,:),Zh(1,:),Zh(1,:),hOpts{:}); % hub cylinder bottom
    patch(Xh(end,:),Yh(end,:),Zh(end,:),Zh(end,:),hOpts{:}); % hub cyl. top
    
    
%% if propeller is going fast: draw it as a disk
elseif gamma_dot >= gamma_dot_thresh
    
    % don't draw anything lighter than lightest disk
    gamma_dot = max(gamma_dot,gamma_dot_disk_lo);
    
    % don't draw anything darker than darkest disk
    gamma_dot = min(gamma_dot,gamma_dot_disk_hi);
    
    % create a disk, and rotate it
    pthick = .005;   % propeller thickness
    thts = linspace(0,2*pi+.1,100);  % angles of the disk
    pt = [];    % array of points on a disk centered at the origin
    for i=1:length(thts)
        pti = RDiA*[prop_rad*sin(thts(i)); prop_rad*cos(thts(i)); 0];
        pt = [pt, pti];
    end
    pp = pos + pt;  % shift the disk to where it should be
    
    % style
    pfspecs.LineStyle = 'none';
    pfspecs.FaceAlpha = interp1([gamma_dot_disk_lo gamma_dot_disk_hi],...
        [trans_lo trans_hi],gamma_dot);
    pfspecs.FaceLighting = 'gouraud'; % flat (default), gouraud, none
    fill3(pp(1,:),pp(2,:),pp(3,:),pCol,pfspecs);
    
end

end