classdef frame < handle
    properties
        num;
        mserBrightMatrx;
        mserDarkMatrix;
        inputImg;
        outputImg;
        MinDiversity;
        MinArea;
        MaxArea;        
    end
    methods
        %Initialize
        function F = frame(num_, inputImg_, MinDiversity_, MinArea_, MaxArea_)
            F.num = num_;
            F.inputImg = inputImg_;
            F.MinDiversity = MinDiversity_;
            F.MinArea = MinArea_;
            F.MaxArea = MaxArea_;
            %run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
        end
        %Perform MSER and store MSER data in matrix
        function compute_mser(F)
            [Bright, BrightEllipses] = vl_mser(inputImg,'MinDiversity',F.MinDiversity,'MinArea',F.MinArea,'MaxArea',F.MaxArea,'BrightOnDark',1,'DarkOnBright',0);
        end
        function visualize(F)
        
        end
    end
end