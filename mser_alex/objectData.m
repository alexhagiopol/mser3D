classdef objectData < handle
    properties
        totalFrames;
        frameNums;
        pixelData;
        ellipseData;
    end
    methods
        %initialize
        function OD = objectData
            OD.totalFrames = 0;
            OD.frameNums = [];
            OD.pixelData = {};
            OD.ellipseData = [];
        end   
        %update with frame information
        function addFrameData(OD,frameNum,pixels,ellipses)
            OD.totalFrames = OD.totalFrames + 1;
            OD.frameNums = [OD.frameNums,frameNum];
            OD.pixelData{OD.totalFrames} = pixels;
            OD.ellipseData = [OD.ellipseData,ellipses];
        end   
        function printText(OD)
            for i = 1:OD.totalFrames
                disp(['Frame ',num2str(OD.frameNums(i)),'******************']);
                disp('Pixels');
                disp(OD.pixelData{i});
                disp('Ellipses');
                disp(OD.ellipseData(:,i));
            end
        end
        function printImage(OD, rSize, cSize, figNum)
            canvas = 255*ones(rSize,cSize,'uint8'); 
            brightLevel = 255;
            brightChange = floor(255 / OD.totalFrames); 
            for i = 1:OD.totalFrames
                brightLevel = brightLevel - brightChange;
                canvas(OD.pixelData{i}) = brightLevel;                
            end
            figure(figNum);
            imshow(canvas);
            title('MSER Change Over Multiple Frames');
            set(gca,'FontSize',16,'fontWeight','bold');
            hold on
            for i = 1:OD.totalFrames
                plot(OD.ellipseData(1,i),OD.ellipseData(2,i),'r.');                                
            end
            hold off
        end
        function measurements = getMsmts(OD)
            import gtsam.*
            measurements = Point2Vector;
            for i = 1:OD.totalFrames
                point = Point2(OD.ellipseData(1,i),OD.ellipseData(2,i));
                measurements.push_back(point);                             
            end 
        end
    end
end