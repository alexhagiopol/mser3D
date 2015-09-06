classdef objectFarm < handle
    properties
        objects; %objects data structure
    end
    methods        
        %constructor
        function OF = objectFarm()

        end
        
        %add object
        function addObject(OF, object)
            OF.objects = [OF.objects, object];
        end
        
        %Produce an image with the MSERs painted on. Must pass original
        %grayscale due to VL feat requirements
        function Q = getImage(OF,I,f)
            %uses syntax from alex11_ransac_tracking.m
            S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
            Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result
            %ugh, linear time search...need a hash table solution...
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
        
        %perform object matching for member objects in a given frame to
        %non-member objects in a given objectFarm
        %TODO: lookback_num = number of frames to look back in; 0 is default
        function matchObjects(OF, prev_f, new_f, threshold, other_farm)
            prev_ellipses = [];
            current_ellipses = [];
            %find objects in previous frame(s) and extract their ellipse
            %info
            %ugh, linear time search...need a hash table solution...
            for i = 1:length(OF.objects)
                obj = OF.objects(i);
                if obj.last_seen == prev_f
                    ellipse = [obj.getLatestMSER().getEllipse();i]; %associate index of object with its ellipse to retrieve later
                    prev_ellipses = [prev_ellipses,ellipse];
                end
            end
            %extract ellipses from external farm
            for i = 1:length(other_farm.objects)
                obj = other_farm.objects(i);
                ellipse = [obj.getLatestMSER().getEllipse();i]; %associate index of object with its ellipse to retrieve later
                current_ellipses = [current_ellipses,ellipse];
            end
            match_summary = OF.matchEllipses(prev_ellipses, current_ellipses);
            OF.absorbMSERs(other_farm, match_summary, prev_f, new_f, threshold);
        end
        
        %return match information based on ellipse matching
        function match_summary = matchEllipses(OF, prevEllipses, newEllipses, prev_f, new_f)
            numPrevRegions = size(prevEllipses,2);
            numNewRegions = size(newEllipses,2);
            %matches is a matrix that contains the score
            match_scores = zeros(numNewRegions,numPrevRegions);    
            %consider each possible match and assign a score in match_scores matrix
            for p = 1:numPrevRegions
                for n = 1:numNewRegions
                    differences = abs(newEllipses(1:5,n) - prevEllipses(1:5,p));
                    differences(1:2,:) = differences(1:2,:)*20; %increase importance of location
                    score = -1*mean(abs(differences./newEllipses(1:5,n)));
                    match_scores(n,p) = score;
                end
            end

            %{
            match_summary explanation:
            prev regions object index:   8,   6,    9,   4,   3
            curr regions object index:   9,   5,    4,   8,   1
            match score:               2.1, 3.4, -5.6, 1.1, 1.4  
            %}           
            
            if numPrevRegions <= numNewRegions
                match_summary = -1*ones(3,numPrevRegions); 
                for x = 1:numPrevRegions
                    [max_score, index] = max(match_scores(:));
                    [n,p] = ind2sub(size(match_scores),index);
                    match_scores(n,:) = -Inf; %ensure future match uniqueness
                    match_scores(:,p) = -Inf; %ensure future match uniqueness
                    if max_score > -Inf
                        match_summary(1,x) = prevEllipses(6,p);
                        match_summary(2,x) = newEllipses(6,n);
                        match_summary(3,x) = max_score;            
                    end
                end
            else
                match_summary = -1*ones(3,numNewRegions); 
                for x = 1:numNewRegions                    
                    [max_score, index] = max(match_scores(:));
                    [n,p] = ind2sub(size(match_scores),index);
                    match_scores(n,:) = -Inf; %ensure future match uniqueness
                    match_scores(:,p) = -Inf; %ensure future match uniqueness
                    if max_score > -Inf
                        match_summary(1,x) = prevEllipses(6,p);
                        match_summary(2,x) = newEllipses(6,n);
                        match_summary(3,x) = max_score;                  
                    end
                end
            end
        end
        
        %Takes matched MSER info and adds it to existing objects. Takes new
        %MSER info and makes new objects. 
        function absorbMSERs(OF, other_farm, match_summary, prev_f, new_f, threshold)
            %{
            match_summary explanation:
            prev regions object index:   8,   6,    9,   4,   3
            curr regions object index:   9,   5,    4,   8,   1
            match score:               2.1, 3.4, -5.6, 1.1, 1.4  
            %}  
            
            %keep track of qhich new objects have matches
            matched_new_objects = zeros(1,length(other_farm.objects));
            %input matched MSER information into existing objects
            for i = 1:size(match_summary,2)
                if match_summary(3,i) > threshold
                    matched_new_objects(match_summary(2,i)) = 1;
                    OF.objects(match_summary(1,i)).addMSER(other_farm.objects(match_summary(2,i)).getLatestMSER());
                    OF.objects(match_summary(1,i)).updateLastFrame(new_f);
                end
            end
            
            %add new objects corresponding to unmatched MSERs
            for i = 1:length(matched_new_objects)
                if matched_new_objects(i) == 0
                    OF.addObject(other_farm.objects(i));
                end
            end
        end
        
        %Returns coordinates of matched MSERs in frame f and frame f - 1
        function matches = getMatchCoords(OF, f)
            matches = [];
            if f ~= 1
                for o = 1:length(OF.objects)
                    obj = OF.objects(o);
                    if obj.last_seen == f && length(obj.msers) > 1                        
                        matches = [matches,[obj.msers(obj.recent_i).getEllipse();obj.msers(obj.recent_i - 1).getEllipse()]];
                    end
                end
            end
        end
        
        %Shows all groups of MSERs of size > min_size
        function showTracks(OF, min_size)
            
        end
    end
end

















