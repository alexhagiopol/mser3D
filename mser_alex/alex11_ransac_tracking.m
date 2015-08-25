%% Initialize
clc
clear
close all
run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose
import gtsam.*

%% Import video
disp('Starting Video Import');
readerobj = VideoReader('../videos_input/through_the_cracks_jing.mov', 'tag', 'myreader1');
vidFrames = read(readerobj);
N = get(readerobj, 'NumberOfFrames');
disp('Video Import Finished');

%% Set tuning constants 
start = 1; %start at custom frame number. Default = 1.
stop = N;  %end at custom frame number. Default = N.
MinDiversity = 0.7; %VL Feat tuning constant
MinArea = 0.005; %VL Feat tuning constant
MaxArea = 0.03; %VL Feat tuning constant
% Alex tuning constant
threshold = -1; %Score threshold needed for two regions to be considered to come from the same object. A higher score indicates higher similarity.

%% Set up video output
writer = VideoWriter('Nearest_Neighbor_Tracking_1','Uncompressed AVI'); %AVI required because mp4 doesnt work on Matlab Linux :(
writer.FrameRate = 30;
open(writer);
frameTimes = zeros(N,1);
two_pane_fig = figure(1);
%set(two_pane_fig, 'Position', [0,0,2100,700]); 

%% Process first video frame 
f = start;
% Read image from video and resize + grayscale
C = vidFrames(:,:,:,f);
C = imresize(C,0.5);
I=rgb2gray(C);
%Detect MSERs
[Bright, BrightEllipses] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0);
%Set up image data structures for output
S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result
%Choose random starting colors for regions
numRegs = size(BrightEllipses,2);
LastBrightColors = randi([0,255],3,numRegs);
%Fill MSERs w/ colors!
for i=numRegs:-1:1
    mserS=vl_erfill(I,Bright(i));       
    %Convert region from grayscale dimensions to RGB dimensions
    [rS,cS] = ind2sub(size(S), mserS);
    mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
    mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
    mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));
    %Assign colors
    Q(mserC1) = LastBrightColors(1,i);
    Q(mserC2) = LastBrightColors(2,i);  
    Q(mserC3) = LastBrightColors(3,i);
end
% Save this to display later
prevQ = Q;
LastBrightEllipses = BrightEllipses;

%% Process rest of frames + make video
for f=start + 1:stop
    %% Read image, resize, grayscale, make data structures, & detect MSERs
    C = vidFrames(:,:,:,f);
    C = imresize(C,0.5);
    I=rgb2gray(C);    
    S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
    Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result        
    [Bright, BrightEllipses] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0);
    numRegs = size(Bright,1); %number of regions
    currentBrightColors = ones(3,numRegs);
    matches = [BrightEllipses; zeros(size(BrightEllipses))]; %data structure to store matched mser information
    %% Matches data format:
    %{
    ***Matches are above + below eachother. If the match values are all equal to 0, there is no match***
    current_center_X_1          current_center_X_2           ...
    current_center_Y_1          current_center_Y_2           ...
    current_cov_1_1             current_cov_1_2              ...
    current_cov_2_1             current_cov_2_2              ... 
    current_cov_3_1             current_cov_3_2              ... 
    prev_match_center_X_1       prev_match_center_X_2        ...
    prev_match_center_Y_1       prev_match_center_Y_2        ...
    prev_match_cov1_1           prev_match_cov_1_2           ...
    prev_match_cov2_1           prev_match_cov_2_2           ...
    prev_match_cov3_1           prev_match_cov_3_2           ...
    %}
    %% Compare new MSERs to previous MSERs & perform matching step
    brightScores = compareRegionEllipses(LastBrightEllipses,BrightEllipses);
    %analyze matches 
    %% Choose MSER colors based on matching
    for i=numRegs:-1:1
        %% Get mser pixel locations from VL feat and convert region from grayscale dimensions to RGB dimensions:
        mserS=vl_erfill(I,Bright(i));        
        [rS,cS] = ind2sub(size(S), mserS); 
        mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
        mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
        mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));          
        %% Decide if a match exists based on threshold
        if brightScores(2,i) > threshold %Assign same color as "closest" mser
            Q(mserC1) = LastBrightColors(1,brightScores(1,i)); %Assign color
            Q(mserC2) = LastBrightColors(2,brightScores(1,i)); %Assign color 
            Q(mserC3) = LastBrightColors(3,brightScores(1,i)); %Assign color
            currentBrightColors(:,i) = LastBrightColors(:,brightScores(1,i)); %update colors
            matches(6:end,i) = LastBrightEllipses(:,brightScores(1,i)); %place matched ellipse in correct place in matches data structure 
        else
            random_color = randi([0,255],3,1); %Generate random color
            Q(mserC1) = random_color(1,1); %Assign color
            Q(mserC2) = random_color(2,1); %Assign color
            Q(mserC3) = random_color(3,1); %Assign color
            currentBrightColors(:,i) = random_color(:,1); %update color
        end                 
    end
        
    %% Show large concatenated image
    hold on;
    frames = [C,prevQ,Q];
    imshow(frames)
    title(['Original Frame # ',num2str(f), '                          MSER Previous Frame                           MSER Current Frame ']);
    set(gca,'FontSize',13);    
    
    %% Transform ellipses
    BrightEllipsesTrans = vl_ertr(BrightEllipses); %Transpose from XY elipses to RC ellipses
    LastBrightEllipsesTrans = vl_ertr(LastBrightEllipses); %Transpose from XY elipses to RC ellipses
    matchesTrans = [vl_ertr(matches(1:5,:));vl_ertr(matches(6:10,:))];
    
    BrightEllipsesTrans(1,:) = BrightEllipsesTrans(1,:) + 2*size(C,2); %Shift ellipse position to account for concatenated image
    LastBrightEllipsesTrans(1,:) = LastBrightEllipsesTrans(1,:) + size(C,2); %Shift ellipse position to account for concatenated image
    matchesTrans(1,:) = matchesTrans(1,:) + 2*size(C,2);
    matchesTrans(6,:) = matchesTrans(6,:) + size(C,2);    
    
    %% Plot ellipses
    %vl_plotframe(BrightEllipsesTrans, 'w.');
    %vl_plotframe(LastBrightEllipsesTrans, 'w.'); 
    
    %% Plot lines showing connections
    for i = 1:numRegs
        if matchesTrans(6,i) ~= 0 && matchesTrans(7,i) ~= 0 %check if match actually exists 
            line([matchesTrans(1,i),matchesTrans(6,i)],[matchesTrans(2,i),matchesTrans(7,i)],'Color',[1, 1, 1]);
        end
    end    
    
    %% Produce and write video frame 
    frame = frame2im(getframe(two_pane_fig));
    writeVideo(writer,frame);    
    
    %% Update data structures
    LastBrightEllipses = BrightEllipses;
    LastBrightColors = currentBrightColors;
    prevQ = Q;
    
    pause
end

close(writer);









