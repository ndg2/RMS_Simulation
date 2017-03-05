th1=-145/180*pi:0.08:35/180*pi; %Theta1 range to analyse
th2=0:0.08:pi; %Theta2 range to analyse
d3=-47; %Display workspace on paper
patch([30 30 230 230],[-148.5 148.5 148.5 -148.5],[-1 -1 -1 -1],'FaceColor','white','LineWidth',2); %Draw paper
hold on
for i=1:length(th1)
    for j=1:length(th2)
        %Calculate homogenous transformation
        T=[ cos(th1(i) + th2(j)), -sin(th1(i) + th2(j)), 0, 142*cos(th1(i) + th2(j)) + 142*cos(th1(i));
            sin(th1(i) + th2(j)),  cos(th1(i) + th2(j)), 0, 142*sin(th1(i) + th2(j)) + 142*sin(th1(i));
            0,                     0, 1,                                    d3 + 47;
            0,                     0, 0,                                         1];
        pos=T(1:3,end); %Take position vector of end-effector
        plot(pos(1),pos(2),'x'); %Plot x and y components
        hold on
    end
end
camroll(90) %Rotate to have x-axis pointing up
title('Workspace Analysis')
xlabel('X axis')
ylabel('Y axis')
