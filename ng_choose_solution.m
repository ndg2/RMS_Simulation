function thetas = ng_choose_solution(allSolutions, thetasnow)
%% ng_choose_solution.m
%
% Chooses the best inverse kinematics solution from all of the solutions
% passed in. The decision is based on the robot's current configuration.
%
% The first input (allSolutions) is a matrix that contains the joint angles
% needed to place the robot's end-effector at the desired position.
%
% The second input is a vector of the robot's current joint angles
% (thetasnow) in radians.

% Check if joint angles are given between -pi and pi and check if they are
% within the joint limits. Create a matrix with the valid solutions.
counter = 0;
for k=1:2
    valid = 1;
    for m=1:2
        if m==1
            while allSolutions(m,k)>pi
                allSolutions(m,k) = allSolutions(m,k)-2*pi;
            end
            while allSolutions(m,k)<-pi
                allSolutions(m,k) = allSolutions(m,k)+2*pi;
            end
            if allSolutions(m,k)>35/180*pi || allSolutions(m,k)<-145/180*pi %Joint 1 limits
                valid = 0;
            end
        end
        if m==2
            while allSolutions(m,k)>pi
                allSolutions(m,k) = allSolutions(m,k)-2*pi;
            end
            while allSolutions(m,k)<-pi
                allSolutions(m,k) = allSolutions(m,k)+2*pi;
            end
            if allSolutions(m,k)>pi || allSolutions(m,k)<0 %Joint 2 limits
                valid = 0;
            end
        end
    end
    
    if counter == 0 && valid == 1
        validsolutions=allSolutions(:,k);
        counter = 1;
    elseif valid == 1
        validsolutions=[validsolutions allSolutions(:,k)];
        counter = counter + 1;
    end
end

%To decide which of the valid solutions we should choose, we add the
% absolute value of the differences of each angle with the current angle,
% and check which gives us the smallest number
diff=zeros([counter 1]);
for i=1:counter
    for j=1:2
        diff(i)=diff(i)+abs(thetasnow(j)-validsolutions(j,i));
    end
end

%Now, we choose the solution that differs less from our actual position
[~,M]=min(diff);
thetas=validsolutions(:,M);

