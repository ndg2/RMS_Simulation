%% ng_test_painting.m

%% SETUP
clear
clc

pause on; 
GraphingTimeDelay = 0.05;

%% CREATE JOINT ANGLE SEQUENCE

% Initialize the function that calculates positions, orientations, and
% colors.  It returns the duration of the light painting in seconds.
duration = ng_get_poc();

% Create time vector.
tstep = 0.10001;
t = 0:tstep:duration;

% Preallocate space for all the history variables.
ox_history = zeros(length(t),1);
oy_history = zeros(length(t),1);
oz_history = zeros(length(t),1);
phi_history = zeros(length(t),1);
r_history = zeros(length(t),1);
g_history = zeros(length(t),1);
b_history = zeros(length(t),1);
thetas_history = zeros(length(t),3);

% Step through the time vector, filling the histories.
for i = 1:length(t)
    [~, ox_history(i), oy_history(i), oz_history(i), r_history(i), g_history(i), b_history(i)] = ng_get_poc(t(i));
end

%% TEST

disp('Starting the test.')
disp('Click in this window and press control-c to stop the code.')

% Plot the robot in the home position to get the plot handles.
figure(1); clf
h = plot_robot_ng(254,0,0,0,0,0,0,0);

for i = 1:length(ox_history)
    
    % Pull the current values of ox, oy, and oz from their histories. 
    ox = ox_history(i);
    oy = oy_history(i);
    oz = oz_history(i);
    
    % Pull the current values of red, green, and blue from their histories.
    r = r_history(i);
    g = g_history(i);
    b = b_history(i);
        
    % IK
    allthetas = robot_ik_ng(ox, oy, oz);
    
    if (i == 1)
        % Pass in the home position as the current position.
        thetas_history(i,:) = ng_choose_solution(allthetas, [0 0 0]')';
    else        
        % Choose the solution to show.
        thetas_history(i,:) = ng_choose_solution(allthetas, thetas_history(i-1,:)')';
    end

    % Plot the robot.      
    plot_robot_ng(ox,oy,oz,thetas_history(i,1),thetas_history(i,2),r,g,b,h);
    title(['PORTRAIT DRAWING ROBOT - Pose ' num2str(i)])
    pause(GraphingTimeDelay)
    
end

disp('Done with the test.')
