function  visuSolgenia(x,u,t)

% parameters
MinX = -10;
MaxX = 20;
MinY = -10;
MaxY = 20;
FigureNumber = 555;
FigureName = 'Animation';

ShapeHull = [-3.7,3.05,4.3,3.05,-3.7,-3.7;-1.25,-1.25,0,1.25,1.25,-1.25];
ShapeProp = [-0.3,-0.3,0.3,0.3,-0.3,-0.3,-0.5,-0.5,-0.5;0,-0.15,-0.15,0.15,0.15,0,0,0.25,-0.25];

f = figure(FigureNumber);
set(f,'name',FigureName)

ActuatorState = u;
Pose = x(1:3);
Yaw = Pose(3);

Hull = RotationMatrix2d(Yaw)*ShapeHull+[Pose(1)*ones(1,6);Pose(2)*ones(1,6)];

figure(FigureNumber)
hold off
plot(Hull(1,:),Hull(2,:),'linewidth',2,'color','k');
hold on

SteerPos = ActuatorState(2);
LPropRate = ActuatorState(1)*60/1000;
Prop = RotationMatrix2d(Yaw)*(RotationMatrix2d(SteerPos)*ShapeProp-[2.9*ones(1,9);zeros(1,9)])+[Pose(1)*ones(1,9);Pose(2)*ones(1,9)];
ArrowPropRate = [0,LPropRate,LPropRate-0.2*sign(LPropRate),LPropRate,LPropRate-0.2*sign(LPropRate);0,0,0.1,0,-0.1];
ArrowPropRate = RotationMatrix2d(Yaw)*(RotationMatrix2d(SteerPos)*ArrowPropRate-[2.9*ones(1,5);zeros(1,5)])+[Pose(1)*ones(1,5);Pose(2)*ones(1,5)];
LBugRate = ActuatorState(3)*60/2000;
ArrowBugRate = [0,LBugRate,LBugRate-0.2*sign(LBugRate),LBugRate,LBugRate-0.2*sign(LBugRate);0,0,0.1,0,-0.1];
ArrowBugRate = RotationMatrix2d(Yaw)*(RotationMatrix2d(pi/2)*ArrowBugRate+[3.7*ones(1,5);zeros(1,5)])+[Pose(1)*ones(1,5);Pose(2)*ones(1,5)];
plot(Prop(1,:),Prop(2,:),'linewidth',2,'color','k');
plot(ArrowPropRate(1,:),ArrowPropRate(2,:),'linewidth',4,'color','r');
plot(ArrowBugRate(1,:),ArrowBugRate(2,:),'linewidth',4,'color','r');

plot(Pose(1),Pose(2),'o','Markersize',5,'color','k');

axis equal
axis([MinX,MaxX,MinY,MaxY]);
xlabel('x (m)','interpreter','latex')
ylabel('y (m)','interpreter','latex')
title(['Simulation Time = ' num2str(t) ' s'],'interpreter','latex')
set(gca,'TickLabelInterpreter','latex');
set(gca,'fontsize',20)
set(gcf,'color','white')
drawnow

    function R = RotationMatrix2d(psi)
        %Rotate positive
        R = [ cos(psi), -sin(psi);
            sin(psi),  cos(psi)];
    end
end

