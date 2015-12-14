classdef MSER < handle
    properties
        data;
    end
    methods
        %initialize
        function M = MSER(ellipse,seed,frame,Ravg,Gavg,Bavg)
            M.data = [ellipse;seed;frame;Ravg;Gavg;Bavg];            
        end
        %return info
        function ellipse = getEllipse(M)
            ellipse = M.data(1:5);
        end
        function seed = getSeed(M)
            seed = M.data(6);
        end
        function num = getFrameNum(M)
            num = M.data(7);
        end
        function avg = getAvgColor(M)
            avg = M.data(8:10);
        end
    end    
end