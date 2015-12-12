%% Initialize
clc
%clear
close all
run('../vlfeat-0.9.20/toolbox/vl_setup') % start up vl_feat
vl_version verbose

%% Import video

disp('Starting Video Import');
readerobj = VideoReader('../datasets/through_the_cracks_jing.mov', 'tag', 'myreader1');
vidFrames = read(readerobj);
N = get(readerobj, 'NumberOfFrames');
disp('Video Import Finished');

%% Set tuning constants 
start = 1; %start at custom frame number. Default = 1.
stop = N;  %end at custom frame number. Default = N.
MinDiversity = 0.7; %VL Feat tuning constant
MinArea = 0.005; %VL Feat tuning constant
MaxArea = 0.03; %0.09 %VL Feat tuning constant
% Alex tuning constant
threshold = -1; %-1 %Score threshold needed for two regions to be considered to come from the same object. A higher score indicates higher similarity.

%% Set up video output
writer = VideoWriter('Naive_Tracking_Movie','Uncompressed AVI'); %AVI required because mp4 doesnt work on Matlab Linux :(
writer.FrameRate = 10;
open(writer);
three_pane_fig = figure(1);
set(three_pane_fig, 'Position', [0,0,2100,700]); 

%% Process first video frame 
f = start;
% Read image from video and resize + grayscale
C = vidFrames(:,:,:,f);
C = imresize(C,0.5);
I=rgb2gray(C);
%Detect MSERs
[Bright, BrightEllipses] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0);
numRegs = size(BrightEllipses,2);
%Create object farm!
mainOC = ObjectCollection(size(I,1),size(I,2),stop - start + 1); %tell the object data structure that the last #numRegs objects are associated with the last frame 
mainOC.addImage(f,I);
%Create objects!
for i = 1:numRegs
    myMSER = MSER(BrightEllipses(:,i),Bright(i),f); %store MSER info
    color = randi([0,255],3,1); %generate random color
    %Object ID: 1.012 = frame #1, region #12. That's why we divide by 1000.
    myObject = Object(myMSER,f+i/1000,f,color); %create new object for each mser
    mainOC.addObject(myObject); %add to big data structure
end
prevIm = mainOC.getImage(I,f);

%% Process rest of frames + make video
for f=start + 1:stop
    %% Read image, resize, grayscale, make data structures, & detect MSERs
    C = vidFrames(:,:,:,f);
    C = imresize(C,0.5);
    I=rgb2gray(C);
    [Bright, BrightEllipses] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0); %Set to compute dark on bright!!!
    mainOC.addImage(f,I);
    numRegs = size(Bright,1); %number of regions
    tempOC = ObjectCollection(size(I,1),size(I,2),1); %tell the object data structure that the last #numRegs objects are associated with the last frame 
    %Create objects!
    for i = 1:numRegs
        myMSER = MSER(BrightEllipses(:,i),Bright(i),f); %store MSER info
        color = randi([0,255],3,1); %generate random color
        %Object ID: 1.012 = frame #1, region #12. That's why we divide by 1000.
        myObject = Object(myMSER,f+i/1000,f,color); %create new object for each mser
        tempOC.addObject(myObject); %add to big data structure
    end
    %Perform matching
    mainOC.matchObjects(f-1,f,threshold,tempOC);
    newIm = mainOC.getImage(I,f);
    
    %% Display results
    %Show raw image, prev frame, new frame
    hold on;
    frames = [C,prevIm,newIm];
    imshow(frames)
    title(['Original Frame # ',num2str(f), '                          MSER Previous Frame                           MSER Current Frame ']);
    set(gca,'FontSize',13);    
    %Get matches between current and past frame in matrix form
    matches = mainOC.getMatchCoords(f);
    %Change from XY elipses to RC ellipses
    if size(matches,2) > 0
        matchesTrans = [vl_ertr(matches(1:5,:));vl_ertr(matches(6:10,:))];
        %Shift locations over because of image concatenation
        matchesTrans(1,:) = matchesTrans(1,:) + 2*size(C,2);
        matchesTrans(6,:) = matchesTrans(6,:) + size(C,2);    
        %Draw lines
        for i = 1:size(matchesTrans,2)
            if matchesTrans(6,i) > 0 && matchesTrans(7,i) > 0 %check if match actually exists 
                line([matchesTrans(1,i),matchesTrans(6,i)],[matchesTrans(2,i),matchesTrans(7,i)],'Color',[1, 1, 1]);
            end
        end
    end
    %% Produce and write video frame 
    frame = frame2im(getframe(three_pane_fig));
    writeVideo(writer,frame);    
    prevIm = newIm;
    %pause;    
end
close(writer);

%% Show MSER tracks in stills and videos. WARNING: These two commands could take hours!
mainOC.showTracks(3,size(I,1),size(I,2),2);
mainOC.makeTrackVideo(5,size(I,1),size(I,2),2);

%% Compute and display statistics
mser_counts = mainOC.computeTrackLengths();
stdev = std(mser_counts);
avg = mean(mser_counts);
maxi = max(mser_counts);
mini = min(mser_counts);
disp(['Avg length of tracks = ',num2str(avg)]);
disp(['Max length of tracks = ',num2str(maxi)]);
disp(['Min length of tracks = ',num2str(mini)]);
disp(['STDEV length of tracks = ',num2str(stdev)]);
fig = figure;
set(fig, 'Position', [0,0,800,500]);
edges = [round(mini):1:round(avg+stdev),50];
histogram(mser_counts,edges);  
ylim([0,450]);
ylabel('Number of Tracks');
xlabel('Number of MSERs per Track');
title('MSER Track Statistics Using Multiple (6) Frame Matching for Dark on Bright MSERs');
line([avg,avg],[0,450],'Color',[1,0,0],'LineStyle','-');
line([avg + stdev,avg + stdev],[0,450],'Color',[1,0,0],'LineStyle','--');

fig2 = figure;
set(fig2, 'Position', [0,0,1500,500]);
tracks_per_frame = mainOC.computeGoodTracksPerFrame(round(stdev + avg));
plot(tracks_per_frame,'b*');
ylim([0,18]);
ylabel('Number of Good Tracks');
xlabel('Frame #');
title('Good Dark on Bright Tracks Visible in Each Video Frame');
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    