function [traj] = trajectory(type, dt)

    if strcmp(type, 'point')
        traj = [3 ; 3];
    end
    
    if strcmp(type, 'line')
        cx = 0:dt:10;
        cy = ones(size(cx))*3;
        traj = [cx ; cy];
    end
    
    if strcmp(type, 'sine')
        cx = 0:dt:10;
        cy = sin(cx);
        traj = [cx ; cy];
    end
    
    if strcmp(type, 'akker1')
        cx1 = 1:dt:3;
        cy1 = ones(size(cx1))*0;
        cx3 = flip(1:dt:3);
        cy3 = ones(size(cx1))*-1.5;
        cx5 = 1:dt:3;
        cy5 = ones(size(cx1))*-3;
        cx = [cx1 cx3 cx5];
        cy = [cy1 cy3 cy5];
        traj = [cx; cy];
    end
    
    if strcmp(type, 'akker2')
        cx1 = 1:dt:3;
        cy1 = ones(size(cx1))*0;
        th =  flip(-pi/2:dt:pi/2);
        cx2 = 0.75 * cos(th) + 3;
        cy2 = 0.75 * sin(th) - 0.75;
        cx3 = flip(1:dt:3);
        cy3 = ones(size(cx1))*-1.5;
        th =  pi/2:dt:3*pi/2;
        cx4 = 0.75 * cos(th) + 1;
        cy4 = 0.75 * sin(th) - 2.25;
        cx5 = 1:dt:3;
        cy5 = ones(size(cx1))*-3;
        cx = [cx1 cx2 cx3 cx4 cx5];
        cy = [cy1 cy2 cy3 cy4 cy5];
        traj = [cx; cy];
    end
    
    if strcmp(type, 'akker3')
        cx = [1 9 9 1 1 9];
        cy = [0 0 -1.5 -1.5 -3 -3];
        traj = [cx; cy];
    end

end