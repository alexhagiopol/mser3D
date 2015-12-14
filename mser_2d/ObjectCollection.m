classdef ObjectCollection < handle
    properties
        objects; %objects data structure
        frames;
    end
    methods        
        %constructor
        function OC = ObjectCollection(r,c,n)
            OC.frames = cast(zeros(r,c,n),'UINT8');
        end
        
        %add object
        function addObject(OC, object)
            OC.objects = [OC.objects, object];
        end
        
        %add GRAYSCALE image to retrieve later
        function addImage(OC, n, I)
            OC.frames(:,:,n) = I; 
        end
        
        %Produce an image with the MSERs painted on. Must pass original
        %grayscale due to VL feat requirements
        function Q = getImage(OC,I,f)
            %uses syntax from alex11_ransac_tracking.m
            S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
            Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result
            for i = 1:length(OC.objects)
                obj = OC.objects(i);
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
        function matchObjects(OC, prev_f, new_f, threshold, other_farm)
            prev_ellipses = [];
            current_ellipses = [];
            %find objects in previous frame(s) and extract their ellipse
            %info
            %ugh, linear time search...need a hash table solution...
            for i = 1:length(OC.objects)
                obj = OC.objects(i);
                if obj.last_seen == prev_f || obj.last_seen == prev_f - 1 || obj.last_seen == prev_f - 2 || obj.last_seen == prev_f - 3 || obj.last_seen == prev_f - 4 || obj.last_seen == prev_f - 5
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
            match_summary = OC.matchEllipses(prev_ellipses, current_ellipses);
            OC.absorbMSERs(other_farm, match_summary, prev_f, new_f, threshold);
        end
        
        %return match information based on ellipse matching
        function match_summary = matchEllipses(OC, prevEllipses, newEllipses, prev_f, new_f)
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
        function absorbMSERs(OC, other_farm, match_summary, prev_f, new_f, threshold)
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
                    OC.objects(match_summary(1,i)).addMSER(other_farm.objects(match_summary(2,i)).getLatestMSER());
                    OC.objects(match_summary(1,i)).updateLastFrame(new_f);
                end
            end
            
            %add new objects corresponding to unmatched MSERs
            for i = 1:length(matched_new_objects)
                if matched_new_objects(i) == 0
                    OC.addObject(other_farm.objects(i));
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
        
        %Shows all tracks of MSERs of size > min_size. Opens new figure for
        %each tack.
        function showTracks(OC, min_size,rSize,cSize,fig_num_start) 
            close all;
            fig_num = fig_num_start;
            %look at all objects
            for o = 1:length(OC.objects)
                obj = OC.objects(o);
                %if object meets track length criteria
                if length(obj.msers) > min_size
                    bwcanvas = 255*ones(rSize,cSize,'uint8'); 
                    brightLevel = 255;
                    brightChange = floor(255 / size(obj.msers,2));
                    %Paint each MSER region with a slightly different
                    %brightness to show motion
                    figure(fig_num);                      
                    for m = 1:length(obj.msers)
                        mser = obj.msers(m);
                        pixels = vl_erfill(OC.frames(:,:,mser.getFrameNum()), mser.getSeed());
                        brightLevel = brightLevel - brightChange;
                        bwcanvas(pixels) = brightLevel;                         
                    end
                    %Plot red crosses at the center of each MSER. Hack
                    %below is necessary because MATLAB plot() function
                    %craps itself when you have many consective figures
                    rgbcanvas = cast(zeros(size(bwcanvas,1),size(bwcanvas,2),3),'UINT8');
                    rgbcanvas(:,:,1) = bwcanvas;
                    rgbcanvas(:,:,2) = bwcanvas;
                    rgbcanvas(:,:,3) = bwcanvas;
                    for m = 1:length(obj.msers)
                        mser = obj.msers(m);
                        ctr = floor(mser.data(1:2)); %vl_ertr(mser.data(1:5));
                        rgbcanvas(ctr(1)-3:ctr(1)+3,ctr(2),1) = 255;
                        rgbcanvas(ctr(1),ctr(2)-3:ctr(2)+3,1) = 255;
                    end
                    %display
                    imshow(rgbcanvas);
                    title(['MSER Track from Frame #',num2str(obj.last_seen - size(obj.msers,2) + 1),' to Frame #',num2str(obj.last_seen)]);
                    set(gca,'FontSize',16,'fontWeight','bold');
                    fig_num = fig_num + 1; 
                end                
            end           
        end
        
        %Same concept as showTracks() except it makes a cool video of the
        %MSER tracks in motion
        function makeTrackVideo(OC, min_size,rSize,cSize,fig_num)
            %% Set up video output
            writer = VideoWriter('Object_Tracks_Movie','Uncompressed AVI'); %AVI required because mp4 doesnt work on Matlab Linux :(
            writer.FrameRate = 7;
            open(writer);
            two_pane_fig = figure(fig_num);
            set(two_pane_fig, 'Position', [0,0,2100,700]);
            %% Look at every object   
            obj_num = 1;
            for o = 1:length(OC.objects)
                obj = OC.objects(o);
                %if object meets track length criteria
                if length(obj.msers) > min_size 
                    left_display = 255*zeros(rSize,cSize,3,'uint8');
                    right_display = 255*ones(rSize,cSize,3,'uint8');
                    bright_level = 255;
                    bright_change = floor(255 / size(obj.msers,2));                                         
                    for m = 1:length(obj.msers)
                        mser = obj.msers(m);
                        mser_pixels = vl_erfill(OC.frames(:,:,mser.getFrameNum()), mser.getSeed());
                        bright_level = bright_level - bright_change;
                        
                        tempR = right_display(:,:,1);
                        tempG = right_display(:,:,2);
                        tempB = right_display(:,:,3);
                        tempR(mser_pixels) = bright_level;
                        tempG(mser_pixels) = 0;
                        tempB(mser_pixels) = 0;
                        right_display(:,:,1) = tempR;
                        right_display(:,:,2) = tempG;
                        right_display(:,:,3) = tempB;
                        
                        orig_bwimage = OC.frames(:,:,mser.getFrameNum());
                        tempR = orig_bwimage;
                        tempG = orig_bwimage;
                        tempB = orig_bwimage;
                        tempR(mser_pixels) = 255;
                        tempG(mser_pixels) = 0;
                        tempB(mser_pixels) = 0;
                        left_display(:,:,1) = tempR;
                        left_display(:,:,2) = tempG;
                        left_display(:,:,3) = tempB;
                        
                        for m_1 = 1:m
                            mser_1 = obj.msers(m_1);
                            ctr = floor(mser_1.data(1:2)); %vl_ertr(mser.data(1:5));
                            right_display(ctr(1)-3:ctr(1)+3,ctr(2),1) = 0;
                            right_display(ctr(1)-3:ctr(1)+3,ctr(2),2) = 255;
                            right_display(ctr(1)-3:ctr(1)+3,ctr(2),3) = 255;
                            right_display(ctr(1),ctr(2)-3:ctr(2)+3,1) = 0;
                            right_display(ctr(1),ctr(2)-3:ctr(2)+3,2) = 255;
                            right_display(ctr(1),ctr(2)-3:ctr(2)+3,3) = 255;
                        end
                        
                        frame = [left_display,right_display];
                        imshow(frame);
                        title(['MSER Track from Frame #',num2str(obj.last_seen - size(obj.msers,2) + 1),' to Frame #',num2str(obj.last_seen)]);
                        set(gca,'FontSize',16,'fontWeight','bold');
                        
                        hold on;
                        %vl_plotframe(vl_ertr(mser.data(1:5)), 'r.');                          
                        title(['Current track #',num2str(obj_num),' over ',num2str(length(obj.msers)),' frames. Current video frame #',num2str(mser.getFrameNum()),' out of ',num2str(size(OC.frames,3)),' video frames.']);
                        set(gca,'FontSize',16,'fontWeight','bold');
                        %% Produce and write video frame 
                        frame = frame2im(getframe(two_pane_fig));
                        writeVideo(writer,frame);
                    end
                    obj_num = obj_num + 1;
                end                
            end             
            close(writer);
        end
        
        %Write the measurements made in CSV 
        function exportMserMeasurementsInGroups(OC, min_size, filename)            
            rowNumber = 0;
            dlmwrite(filename,[-1,-1]); %use this as a way to clear the file
            for o = 1:length(OC.objects)
                obj = OC.objects(o);
                if length(obj.msers) > min_size
                    obj = OC.objects(o);
                    vectorToWrite = zeros(1,5+6*length(obj.msers));
                    %dlmwrite(filename,o,'delimiter',',','roffset',rowNumber,'coffset',0,'-append'); %use o as an object ID in CSV file
                    %dlmwrite(filename,length(obj.msers),'delimiter',',','roffset',rowNumber,'coffset',1,'-append'); %write how many msers we will send per line
                    vectorToWrite(1) = o;
                    vectorToWrite(2) = length(obj.msers);
                    for m = 1:length(obj.msers)
                        colNumber = 3 + m*6 - 6; %each mser data group is 6 numbers long
                        mser = obj.msers(m);                        
                        frameNum = mser.getFrameNum();
                        %dlmwrite(filename,frameNum,'delimiter',',','roffset',rowNumber,'coffset',colNumber,'-append')
                        %dlmwrite(filename,mser.getEllipse,'delimiter',',','roffset',rowNumber,'coffset',colNumber + 1,'-append')
                        vectorToWrite(colNumber) = frameNum;
                        vectorToWrite(colNumber + 1: colNumber + 5) = vl_ertr(mser.getEllipse); %vl_ertr necessary to go from XY to RC system
                    end
                    vectorToWrite(end - 2: end) = obj.getColor';
                    %dlmwrite(filename,obj.getColor,'delimiter',',','roffset',rowNumber,'coffset',colNumber + 6,'-append')
                    dlmwrite(filename,vectorToWrite,'delimiter',',','-append');
                    rowNumber = rowNumber + 1;
                    disp(['Wrote object #',num2str(o),' to ',filename]);
                end                
            end
        end
        
        function mser_counts  = computeTrackLengths(OC)
            disp(['Number of tracks = ',num2str(length(OC.objects))]);
            mser_counts = ones(1,length(OC.objects));
            for i = 1:length(OC.objects)
                obj = OC.objects(i);
                mser_counts(i) = size(obj.msers,2);
            end            
        end
        
        function good_tracks_per_frame = computeGoodTracksPerFrame(OC, threshold)
            good_tracks_per_frame = zeros(1,size(OC.frames,3));
            for i = 1:size(OC.frames,3)
                for j = 1:length(OC.objects)
                    obj = OC.objects(j);
                    if size(obj.msers,2) > threshold
                        for k = 1:size(obj.msers,2)
                            if obj.msers(k).getFrameNum() == i
                                good_tracks_per_frame(i) = good_tracks_per_frame(i) + 1;
                                break;
                            end                            
                        end                      
                    end
                end
            end
        end
    end
end

















