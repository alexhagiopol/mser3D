% alex02_mser
% Needs VL Feat library
% Uses code from frank01_mser

clc
close all
run('vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose


%% Import video (comment this section out after you import once and store video frames in workspace; this takes a long time and a lot of memory)
%{
disp('Starting Video Import');
readerobj = VideoReader('60fps_Paul_Video_1.mp4', 'tag', 'myreader1');
vidFrames = read(readerobj);
N = get(readerobj, 'NumberOfFrames');
disp('Video Import Finished');
%}

%% Set up video output
writer = VideoWriter('MSER_Auto_Rally_Tracking_0','MPEG-4');
writer.Quality = 100;
writer.FrameRate = 30;
open(writer);
frameTimes = zeros(N,1);
myFigure = figure(1);
set(myFigure, 'Position', [0,0,600,600]); 

%% Set up color choices
%{
"Nice" colors in RGB:
red     = 255   0   0 
orange  = 255 128   0
yellow  = 255 255   0
green   =   0 255   0
blue    =   0 128 255
purple  = 127   0 255
pink    = 255   0 255
%}
brightRcolorset = [255, 255, 255,   0,   0, 127, 255];
brightGcolorset = [  0, 128, 255, 255, 128,   0,   0];
brightBcolorset = [  0,   0,   0,   0, 255, 255, 255]; 
darkRcolorset = floor(brightRcolorset/2);
darkGcolorset = floor(brightGcolorset/2);
darkBcolorset = floor(brightBcolorset/2);

%% Set up data structures for remembering colors and regions
%LastBright = [0];
LastBrightEllipses = [0;0;0;0;0];
LastBrightColors = [1];
%LastDark = [0];
LastDarkEllipses = [0;0;0;0;0];
LastDarkColors = [1];

%% Threshold for region equality 
threshold = -2; 
memoryLoss = 1.3;
%% Process each video frame 
for f=200:N
    %% Read image from video and resize + grayscale
    C = vidFrames(:,:,:,f); 
    C = imresize(C,0.25,'bicubic');   
    I=rgb2gray(C);
    I=adapthisteq(I);    
    %% Detect MSERs
    [Bright, BrightEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.005,'MaxArea',0.03,'BrightOnDark',1,'DarkOnBright',0);
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.005,'MaxArea',0.03,'BrightOnDark',0,'DarkOnBright',1);
    %% Compare new MSERs to previous MSERs
    brightScores = compareRegionEllipses(LastBrightEllipses,BrightEllipses);
    darkScores = compareRegionEllipses(LastDarkEllipses,DarkEllipses);
    %% Set up image data structures for output
    S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
    Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result    
    currentBrightColors = ones(1,size(Bright,1));
    currentDarkColors = ones(1,size(Dark,1));
    %% Fill bright MSERs
    for i=size(Bright,1):-1:1
        mserS=vl_erfill(I,Bright(i));       
        %Convert region from grayscale to RGB
        [rS,cS] = ind2sub(size(S), mserS);
        mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
        mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
        mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));          
        %Check thershold
        if brightScores(2,i) > threshold
            colorIndex = LastBrightColors(brightScores(1,i));
            %Assign colors to regions
            Q(mserC1) = brightRcolorset(colorIndex);
            Q(mserC2) = brightGcolorset(colorIndex);
            Q(mserC3) = brightBcolorset(colorIndex);
        else
            colorIndex = mod(i,7) + 1; 
            %disp(brightScores(2,i));
            %Assign colors to regions; darker colors for new regions
            Q(mserC1) = floor(brightRcolorset(colorIndex)/5);
            Q(mserC2) = floor(brightGcolorset(colorIndex)/5);
            Q(mserC3) = floor(brightBcolorset(colorIndex)/5);
        end
        currentBrightColors(i) = colorIndex;       
                
    end
    %Update data structures
    %LastBright = Bright;
    LastBrightEllipses = [BrightEllipses, LastBrightEllipses(:,1:cast(ceil(size(LastBrightEllipses,2))/memoryLoss,'uint8'))];
    LastBrightColors = [currentBrightColors, LastBrightColors(:,1:cast(ceil(size(LastBrightColors,2))/memoryLoss,'uint8'))];    
    %% Fill dark MSERs
    for i=1:size(Dark,1)
        mserS=vl_erfill(I,Dark(i));      
        %Convert region from grayscale to RGB
        [rS,cS] = ind2sub(size(S), mserS);
        mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
        mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
        mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));                 
        %Check thershold
        if darkScores(2,i) > threshold
            colorIndex = LastDarkColors(darkScores(1,i));
            %Assign colors to regions
            Q(mserC1) = darkRcolorset(colorIndex);
            Q(mserC2) = darkGcolorset(colorIndex);
            Q(mserC3) = darkBcolorset(colorIndex);
        else
            colorIndex = mod(i,7) + 1; 
            %disp(darkScores(2,i));
            %Assign colors to regions; darker colors for new regions
            Q(mserC1) = floor(darkRcolorset(colorIndex)/5);
            Q(mserC2) = floor(darkGcolorset(colorIndex)/5);
            Q(mserC3) = floor(darkBcolorset(colorIndex)/5);
        end
        currentDarkColors(i) = colorIndex;        
                
    end
    %Update data structures
    %LastDark = Dark;
    LastDarkEllipses = [DarkEllipses,LastDarkEllipses(:,1:cast(ceil(size(LastDarkEllipses,2))/memoryLoss,'uint8'))];
    LastDarkColors = [currentDarkColors,LastDarkColors(:,1:cast(ceil(size(LastDarkColors,2))/memoryLoss,'uint8'))];   
    %disp(size(LastDarkColors));
    %% 2 quadrant video display
    %Quadrant 1: Original grayscale
    subplot(2,1,1);
    imshow(I); %original grayscale
    title('Original Grayscale Video');
    set(gca,'FontSize',16,'fontWeight','bold')
    % Quadrant 2: MSER color
    subplot(2,1,2);
    imshow(Q)  
    title('Tracked MSER Regions in Color');
    set(gca,'FontSize',16,'fontWeight','bold')    
    %Plot ellipses
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    brightEllipsesTrans = vl_ertr(BrightEllipses);
    vl_plotframe(darkEllipsesTrans,'w-');
    vl_plotframe(brightEllipsesTrans, 'k-');
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    writeVideo(writer,frame);
    %pause
end
close(writer);


