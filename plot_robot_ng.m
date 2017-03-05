function hout = plot_robot_ng(x, y, z, theta1, theta2, redValue, greenValue, blueValue, hin)
% 
% X, Y, and Z specify the desired position of the origin of the third frame
% with respect to frame zero.
%
% The robot in the configuration is specified by the joint angle inputs
%
% The colour of the paiting tool.  
% Each color value should range from 0 to 1.  Set the color to all zeros
% (black) to plot no point of light for this pose.
%
% The final input argument, which is optional, is a vector of handles from
% a previous call to this function.  When provided, this function updates
% the plot rather than replacing it.

%% CHECK THE INPUTS

% Put joint angles into a column vector
thetas = [theta1 theta2]';

% Check all of the thetas for NaN (not-a-number).
if (sum(isnan(thetas)) > 0)
    
    warning('NaN passed in for one or more joint angles.');
    
    % Put the robot in the home configuration and turn it red.
    theta1 = 0;
    theta2 = 0;
    pcolor = [1 0 0];
    
else
    
    % Set the robot's color to be normal, and don't modify the joint angles
    pcolor = .8*[1 .88 .75];
    
end


%% FORWARD KINEMATICS

[points_to_plot, x30, y30, z30] = robot_fk_ng(theta1, theta2,z);

% Save the origin of frame 3
o3 = points_to_plot(:,end);


%% DESIRED END-EFFECTOR COORDINATE FRAME
            
% Desired rotation matrix from joint angles.
Rdes = [cos(theta1+theta2) -sin(theta1+theta2) 0;
        sin(theta1+theta2) cos(theta1+theta2) 0;
        0                  0                  1];

% Desired origin position
odes = [x ; y ; z];

% Homogeneous transformation.
Hdes = [Rdes odes; 0 0 0 1];

% Homogeneous representation.
odes = [odes ; 1];

% Length of coordinate frame vectors, in mm.
vlen = 25;

% Calculate the coordinates of line segments showing the x, y, and z unit
% vectors of the desired location for frame 3 in frame 0.
xdes = [odes (Hdes * [vlen 0 0 1]')];
ydes = [odes (Hdes * [0 vlen 0 1]')];
zdes = [odes (Hdes * [0 0 vlen 1]')];


%% PLOT ROBOT

if (nargin == 9)

    % The user has passed in an array of handles. Split them out. 
    hrobot = hin(1);
    hx30 = hin(2);
    hy30 = hin(3);
    hz30 = hin(4);
    hxdes = hin(5);
    hydes = hin(6);
    hzdes = hin(7);
    hjoints = hin(8);
    hshadow = hin(9);

    % Update the robot's position.
    set(hrobot, 'xdata', points_to_plot(1,:), 'ydata', points_to_plot(2,:), 'zdata', points_to_plot(3,:),'color',pcolor);
    
    % Update the robot's joints.
    set(hjoints, 'xdata', points_to_plot(1,2:2:4), 'ydata', points_to_plot(2,2:2:4), 'zdata', points_to_plot(3,2:2:4));
    
    % Update the shadow of the tip on the paper
    set(hshadow, 'xdata', x, 'ydata', y);

    % Update the position of the axes of frame 3.
    set(hx30, 'xdata', x30(1,:), 'ydata', x30(2,:), 'zdata', x30(3,:));
    set(hy30, 'xdata', y30(1,:), 'ydata', y30(2,:), 'zdata', y30(3,:));
    set(hz30, 'xdata', z30(1,:), 'ydata', z30(2,:), 'zdata', z30(3,:));
    
    % If the color is not black, add this tip position to the plot.
    if ((((redValue ~= 0) || (greenValue ~= 0)) || blueValue ~= 0) && z == 0)
        hold on
        plot3(o3(1),o3(2),o3(3),'.','color',[redValue greenValue blueValue],'markersize',15);
        hold off
    end
    
    % Update the position of the desired pose for frame 3.
    set(hxdes, 'xdata', xdes(1,:), 'ydata', xdes(2,:), 'zdata', xdes(3,:));
    set(hydes, 'xdata', ydes(1,:), 'ydata', ydes(2,:), 'zdata', ydes(3,:));
    set(hzdes, 'xdata', zdes(1,:), 'ydata', zdes(2,:), 'zdata', zdes(3,:));

    % Copy the inputted array of handles to the output.
    hout = hin;
else
    
    % This is the first time this function is being called.
    
    % Plot the robot in 3D using big dots at the points and thick lines
    % to connect neighboring points.  Keep a handle to the plot.
    hrobot = plot3(points_to_plot(1,:), points_to_plot(2,:), points_to_plot(3,:), '.-','linewidth',7,'color',pcolor);
    
    % Plot the canvas
    patch([30 30 240 240],[-148.5 148.5 148.5 -148.5],[-1 -1 -1 -1],'FaceColor','white');
    
    % Plot the base of the robot
    patch([-15 -15 15 15],[-15 15 15 -15],[0 0 0 0],'FaceColor',0.5*pcolor);
    hold on
    
    % Plot the robot joints
    hjoints = scatter3(points_to_plot(1,2:2:4),points_to_plot(2,2:2:4),points_to_plot(3,2:2:4),150,'y','filled');
    
    % Plot the shadow of the tip on the paper
    hshadow = scatter3(x,y,0,10,[0.7 0.7 0.7],'filled');
    
    % Plot the x, y, and z axes of frame 3, keeping handles.
    hx30 = plot3(x30(1,:), x30(2,:), x30(3,:), 'g:', 'linewidth',2);
    hy30 = plot3(y30(1,:), y30(2,:), y30(3,:), 'g--', 'linewidth',2);
    hz30 = plot3(z30(1,:), z30(2,:), z30(3,:), 'g-', 'linewidth',2);
    
    % If the color is not black, plot the tip position.
    if (((redValue ~= 0) || (greenValue ~= 0)) || blueValue ~= 0)
        plot3(o3(1),o3(2),o3(3),'.','color',[redValue greenValue blueValue],'markersize',35);
    end
        
    % Plot the x, y, and z axes of the desired pose for frame 3.
    hxdes = plot3(xdes(1,:), xdes(2,:), xdes(3,:), ':', 'linewidth',2,'color',[.5 0 0]);
    hydes = plot3(ydes(1,:), ydes(2,:), ydes(3,:), '--', 'linewidth',2,'color',[.5 0 0]);
    hzdes = plot3(zdes(1,:), zdes(2,:), zdes(3,:), '-', 'linewidth',2,'color',[.5 0 0]);
    
    hold off
    
    xlabel('X (mm)')
    ylabel('Y (mm)')
    zlabel('Z (mm)')
    set(gca,'color',[.45 .59 .82])
    axis equal vis3d
    axis([-150 300 -200 200 -10 200])
    view(45,20)

    % Assemble handles into a vector to return.
    hout = [hrobot; hx30; hy30; hz30; hxdes; hydes; hzdes; hjoints; hshadow];

end
