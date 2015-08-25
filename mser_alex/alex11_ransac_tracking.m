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


%% Tuning constants 
start = 1; %start at custom frame number. Default = 1.
stop = N;  %end at custom frame number. Default = N.
MinDiversity = 0.7; %VL Feat tuning constant
MinArea = 0.005; %VL Feat tuning constant
MaxArea = 0.03; %VL Feat tuning constant
%Alex tuning constant
threshold = -1; %Score threshold needed for two regions to be considered to come from the same object. A higher score indicates higher similarity.

%% Set up video output
writer = VideoWriter('Nearest_Neighbor_Tracking_1','Uncompressed AVI'); %AVI required because mp4 doesnt work on Matlab Linux :(
writer.FrameRate = 30;
open(writer);
frameTimes = zeros(N,1);
two_pane_fig = figure(1);
set(two_pane_fig, 'Position', [0,0,2100,700]); 

%% Color choices
%                 red, orange, yellow, green, blue, purple, pink
brightRcolorset = [255, 255, 255,   0,   0, 127, 255];
brightGcolorset = [  0, 128, 255, 255, 128,   0,   0];
brightBcolorset = [  0,   0,   0,   0, 255, 255, 255]; 

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
LastBrightColors = zeros(1,numRegs);
for i = 1:numRegs
    LastBrightColors(1,i) = randi([1,7],1,1);
end
%Fill MSERs w/ colors!
for i=numRegs:-1:1
    mserS=vl_erfill(I,Bright(i));       
    %Convert region from grayscale dimensions to RGB dimensions
    [rS,cS] = ind2sub(size(S), mserS);
    mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
    mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
    mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));
    Q(mserC1) = brightRcolorset(LastBrightColors(i));
    Q(mserC2) = brightGcolorset(LastBrightColors(i));  
    Q(mserC3) = brightBcolorset(LastBrightColors(i));
end
% Save this to display later
prevQ = Q;
LastBrightEllipses = BrightEllipses;

%% Process rest of frames + make video
for f=start + 1:stop
    %Read image from video and resize + grayscale
    C = vidFrames(:,:,:,f);
    C = imresize(C,0.5);
    I=rgb2gray(C);    
    %Detect MSERs
    [Bright, BrightEllipses] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0);
    %Compare new MSERs to previous MSERs - perform matching step
    brightScores = compareRegionEllipses(LastBrightEllipses,BrightEllipses);
    %Set up image data structures for output
    S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
    Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result    
    currentBrightColors = ones(1,size(Bright,1));    
    %Fill bright MSERs
    for i=size(Bright,1):-1:1
        mserS=vl_erfill(I,Bright(i));       
        %Convert region from grayscale dimensions to RGB dimensions
        [rS,cS] = ind2sub(size(S), mserS);
        mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
        mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
        mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));          
        %Check thershold
        if brightScores(2,i) > threshold %Assign same color as "closest" mser
            colorIndex = LastBrightColors(brightScores(1,i));            
        else
            colorIndex = randi([1,7],1,1); %if no closest mser, choose a new random color
        end
        Q(mserC1) = brightRcolorset(colorIndex);
        Q(mserC2) = brightGcolorset(colorIndex);  
        Q(mserC3) = brightBcolorset(colorIndex);
        currentBrightColors(i) = colorIndex;                 
    end
        
    %% Orignial, MSERs with color and lines
    figure(two_pane_fig);
    frames = [C,prevQ,Q];
    imshow(frames)
    title(['Original Frame # ',num2str(f), '                          MSER Previous Frame                           MSER Current Frame ']);
    set(gca,'FontSize',13);
      
    %% Plot ellipses
    BrightEllipsesTrans = vl_ertr(BrightEllipses); %Transpose from XY elipses to RC ellipses
    LastBrightEllipsesTrans = vl_ertr(LastBrightEllipses); %Transpose from XY elipses to RC ellipses
    BrightEllipsesTrans(1,:) = BrightEllipsesTrans(1,:) + 2*size(C,2); %Shift ellipse position to account for concatenated image
    LastBrightEllipsesTrans(1,:) = LastBrightEllipsesTrans(1,:) + size(C,2); %Shift ellipse position to account for concatenated image
    vl_plotframe(BrightEllipsesTrans, 'w.');
    vl_plotframe(LastBrightEllipsesTrans, 'w.');
    
    %% Produce and write video frame 
    frame = frame2im(getframe(two_pane_fig));
    writeVideo(writer,frame);
    
    %% Update data structures
    LastBrightEllipses = BrightEllipses;
    LastBrightColors = currentBrightColors;
    prevQ = Q;
    %pause 
end

close(writer);









