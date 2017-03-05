%% SETUP

clear
home
pause on;
GraphingTimeDelay = 0.3; % pause between positions in seconds

%% CHOOSE INPUT PARAMETERS

% Test type.
testType = 3;

switch(testType)

    case 1 % Manually modified
        
        ox_history = 127; % mm
        oy_history = -127; % mm
        oz_history = 0; % mm
        
    case 2 % Linear interpolation between two points
        
        % Time vector.
        t = (0:0.2:2*pi)';

        % Starting configuration.
        ox_start = 127; % mm
        oy_start = -127; % mm
        oz_start = 0; % mm

        % Ending configuration.
        ox_end = 127; % mm
        oy_end = 127; % mm
        oz_end = 0; % mm
        
        % Interpolation
        ox_history = ox_start * ones(length(t),1) + (ox_end - ox_start)*(t - t(1))./(t(end) - t(1));
        oy_history = oy_start * ones(length(t),1) + (oy_end - oy_start)*(t - t(1))./(t(end) - t(1));
        oz_history = oz_start * ones(length(t),1) + (oz_end - oz_start)*(t - t(1))./(t(end) - t(1));
        
    case 3 % A circle parallel to the xy plane.

        % Define the radius, z-value and centre coordinates of the circle.
        radius = 60; % mm
        z_offset = 0; % mm
        x_center = 120; % mm
        y_center = 0; % mm
        
        % Time vector.
        t = (0:0.2:2*pi)';
        
        % Compute circle
        ox_history = x_center + radius * cos(t);
        oy_history = y_center + radius * sin(t);
        oz_history = z_offset * ones(size(t));
                
    otherwise
        error(['Unknown testType: ' num2str(testType)])
end    
    

%% TEST

disp('Starting the test.')
disp('Click in this window and press control-c to stop the code.')

% Plot the robot once in home position
figure(1)
h = plot_robot_ng(254,0,0,0,0,0,0,0);

for i = 1:length(ox_history)
    
    ox = ox_history(i);
    oy = oy_history(i);
    oz = oz_history(i);

    % Do IK
    thetas = robot_ik_ng(ox, oy, oz);
    
    % For each solution
    s = size(thetas);
    for j = 1:s(2)
        
        % Plot the robot       
        plot_robot_ng(ox,oy,oz,thetas(1,j),thetas(2,j),0,0,0,h);
        title(['Test ' num2str(testType) ' - Pose ' num2str(i) ' - Solution ' num2str(j)])
        pause(GraphingTimeDelay)
    end
    pause(GraphingTimeDelay)
    
end

disp('Done with the test.')
