function [duration, x, y, z, r, g, b] = ng_get_poc(t)
%% ng_get_poc.m
%
% Calculates the position and color (poc) for the robot's end-effector at
% the specified point in time.
%
% The only input is the present time (t) in seconds.  The painting begins
% at t = 0, when the robot must be in the home pose. If called without a
% value for time, the function initializes and returns only the duration.
%
%    t: present time, in seconds
%
% The first output is the total duration of the painting, in seconds. The
% calling function needs this information so it can tell when to stop.
%
%    duration: time needed to do the entire light painting, in seconds
%
% The next three outputs (x, y, z) are the coordinates where the robot's
% end-effector tip should be.
%
%     x: x-coordinate of the origin of frame 3 in frame 0, in mm
%     y: y-coordinate of the origin of frame 3 in frame 0, in mm
%     z: z-coordinate of the origin of frame 3 in frame 0, in mm
%
% The last three outputs (r, g, b) are the red, green, and blue components
% of the color that the robot's tip should be on.
%
%     r: red value, from 0 to 1
%     g: green value, from 0 to 1
%     b: blue value, from 0 to 1

%% DEFINE PERSISTENT VARIABLES
persistent xvia yvia zvia rvia gvia bvia 
persistent tipspeed tvia dur


%% HANDLE INITIALIZATION

% No argument call.
if (nargin == 0)
    
    % Load the painting file
    load painting
    
    % Pull the lists of x, y, and z positions out of the painting matrix.
    xvia = painting(:,1);
    yvia = painting(:,2);
    zvia = painting(:,3);
    
    % Pull the lists of red, green, and blue color components out.
    rvia = painting(:,4);
    gvia = painting(:,5);
    bvia = painting(:,6);
    
    % Calculate the distance traveled between neighboring via points,
    % assuming linear interpolation.
    distances = sqrt((diff(xvia)).^2 + (diff(yvia)).^2 + (diff(zvia)).^2);
    
    % Define the speed the tip should move in mm per second.
    tipspeed = 15;
    
    % Calculate durations for travel between neighboring via points,
    % assuming linear interpolation.
    durations = distances / tipspeed;
    
    % Calculate time at each via point using the cumulative sum function.
    tvia = cumsum(durations);
    
    % Add a zero at the start of tvia, since time starts at zero.
    tvia = [0 ; tvia];
    
    % The painting's duration in seconds is the time of the final via
    % point.
    dur = tvia(end);   
    duration = dur;
    
    % Return from initialization.
    return
    
end


%% HANDLE REGULAR CALLS

% Assign value to duration output.
duration = dur;

% We look for the first via point time that is greater than the current
% time. Subtract 1 to get to the index of the first point.
traj = find(t < tvia,1) - 1;

% Linearly interpolate all values.
x = linear_trajectory_ng(t,tvia(traj),tvia(traj+1),xvia(traj),xvia(traj+1));
y = linear_trajectory_ng(t,tvia(traj),tvia(traj+1),yvia(traj),yvia(traj+1));
z = linear_trajectory_ng(t,tvia(traj),tvia(traj+1),zvia(traj),zvia(traj+1));
r = linear_trajectory_ng(t,tvia(traj),tvia(traj+1),rvia(traj),rvia(traj+1));
g = linear_trajectory_ng(t,tvia(traj),tvia(traj+1),gvia(traj),gvia(traj+1));
b = linear_trajectory_ng(t,tvia(traj),tvia(traj+1),bvia(traj),bvia(traj+1));
