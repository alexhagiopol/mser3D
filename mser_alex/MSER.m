classdef MSER < handle
    properties
        data;
    end
    methods
        %initialize
        function M = MSER(ellipse,seed,frame)
            M.data = [ellipse;seed,frame];            
        end
        %return info
        function ellipse = getEllipse(M)
            ellipse = M.data(1:5);
        end
        function seed = getSeed(M)
            seed = M.data(6);
        end
        function num = getFrameNum(m)
            num = M.data(7);
        end
    end    
end