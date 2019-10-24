function animate_quadrotor(T,X,const)
% show the quadrotor flying around given a matrix of states x_all

save = 0; % save flag, save each frame as an image?

% setup plot window
figure;
axis equal;
ax = gca;
ax.Position = [0 0 1 1];
ax.Visible = 'off';
% the max absolute value of all rotor distances
max_d = max(max(abs(X(11:14,:))));

% find min and max of 3D space over all time, and set axis accordingly
xmin = min(X(1,:));
xmax = max(X(1,:));
ymin = min(X(2,:));
ymax = max(X(2,:));
zmin = min(X(3,:));
zmax = max(X(3,:));
extra = .05;
xlim([xmin - max_d - 2*const.ar - extra, xmax + max_d + 2*const.ar + extra]);
ylim([ymin - max_d - 2*const.ar - extra, ymax + max_d + 2*const.ar + extra]);
zlim([zmin - max_d - 2*const.ar - extra, zmax + max_d + 2*const.ar + extra]);

% set the perspective of the 3D plot
az = 45;
el = 25;
view(az, el);

% make an animation by drawing the quadrotor at each time
for i=1:size(X,2)
    cla;
    draw_quadrotor(X(:,i),const);
    drawnow;
    
    if save
        % save frames to be made into animation
        % remember to pause before first frame & maximize the figure window
        current_dir = pwd; % in Windows
        file_name_i = strcat(current_dir,'\test\ani_',num2str(i,'%03.f'));
        print('-opengl','-dpng','-r300',file_name_i);

        % create a file whose title is the frame rate
        fr = 1/(T(2)-T(1)); % frame rate of T and X
        fr_path = strcat(current_dir,'\test\framerate_',...
            num2str(fr),'.txt');
        fid = fopen(fr_path,'w');
        fclose(fid);
    end
end

end

