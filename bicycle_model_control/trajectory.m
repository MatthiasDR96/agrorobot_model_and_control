function [traj] = trajectory(type)

    if strcmp(type, 'sine')
        cx = 0:0.1:10;
        cy = sin(cx / 2.0) .* cx / 2.0 ;
        traj = [cx ; cy];
    end
    
    if strcmp(type, 'akker')
        cx1 = 1:0.1:4;
        cy1 = ones(size(cx1))*1.5;
        cx3 = flip(1:0.1:4);
        cy3 = ones(size(cx1))*0;
        cx5 = 1:0.1:4;
        cy5 = ones(size(cx1))*-1.5;
        traj = [cx1 cx3 cx5; cy1 cy3 cy5];
    end

end