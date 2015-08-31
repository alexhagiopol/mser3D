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
writer.FrameRate = 10;
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
    %Generate matches   
    [matches, matchSummary, currentBrightColors] = generateMatches(LastBrightEllipses,BrightEllipses,LastBrightColors, threshold);
    %% Choose MSER colors based on matching
    for i=numRegs:-1:1
        %% Get mser pixel locations from VL feat and convert region from grayscale dimensions to RGB dimensions:
        if matchSummary(1,i) > 0 && matchSummary(2,i) > threshold %color in MSERs only if they get matched
            mserS=vl_erfill(I,Bright(i));        
            [rS,cS] = ind2sub(size(S), mserS); 
            mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
            mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
            mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1)); 
            Q(mserC1) = currentBrightColors(1,i); %Assign color
            Q(mserC2) = currentBrightColors(2,i); %Assign color
            Q(mserC3) = currentBrightColors(3,i); %Assign color 
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
        if matchesTrans(6,i) > 0 && matchesTrans(7,i) > 0 %check if match actually exists 
            line([matchesTrans(1,i),matchesTrans(6,i)],[matchesTrans(2,i),matchesTrans(7,i)],'Color',[1, 1, 1]);
        end
    end    
    
    %% Produce and write video frame 
    frame = frame2im(getframe(two_pane_fig));
    writeVideo(writer,frame);    
    
    %% Update data structures
    lastNumRegs = size(LastBrightEllipses,2);
    LastBrightEllipses = [BrightEllipses, LastBrightEllipses];
    LastBrightColors = [currentBrightColors, LastBrightColors];
    if lastNumRegs > numRegs
        LastBrightEllipses = LastBrightEllipses(:,1:2*numRegs);
        LastBrightColors = LastBrightColors(:,1:2*numRegs);
    end
    
    prevQ = Q;    
    %pause
end

close(writer);









