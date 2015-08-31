classdef objectFarm < handle
    properties
        objects; %objects data structure
        LSGs; %"last group sizes" [oldest group size, second older group size, ... ];
        currentF; %current frame number
    end
    methods
        %initialize
        function OF = objectFarm(LSG, numFrames)
            OF.LSGs = -1*ones(numFrames,1);
            OF.LSGs(1) = LSG;
            OF.currentF = 1;
        end
        %add object
        function addObject(OF, object)
            OF.objects = [OF.objects, object];
        end
        %update the last group size
        function updateLSG(LSG)
            OF.currentF = OF.currentF + 1;
            OF.LSGs(OF.currentF) = LSG;
        end
        %Produce an image with the MSERs painted on. Must pass original
        %grayscale due to VL feat requirements
        function Q = getImage(OF,I,f)
            %uses syntax from alex11_ransac_tracking.m
            S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
            Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result
            for i = 1:length(OF.objects)
                obj = OF.objects(i);
                if obj.last_seen == f
                    mserS=vl_erfill(I,obj.getLatestMSER().getSeed());       
                    %Convert region from grayscale dimensions to RGB dimensions
                    [rS,cS] = ind2sub(size(S), mserS);
                    mserC1 = sub2ind(size(Q), rS, cS, 1*ones(length(rS),1));
                    mserC2 = sub2ind(size(Q), rS, cS, 2*ones(length(rS),1));
                    mserC3 = sub2ind(size(Q), rS, cS, 3*ones(length(rS),1));
                    %Assign colors
                    Q(mserC1) = obj.color(1);
                    Q(mserC2) = obj.color(2);  
                    Q(mserC3) = obj.color(3);
                end
                %figure;
                %imshow(Q);
            end
        end
    end
end