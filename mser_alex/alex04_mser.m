clc
close all
run('vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose

format = 'Sequence2/frame%02d.jpg'; N = 50; % indoor

%% Set up video output
writer = VideoWriter('MSER_Crude_Tracking_Ellipses_0','MPEG-4');
writer.Quality = 100;
writer.FrameRate = 0.5;
open(writer);
frameTimes = zeros(N,1);
myFigure = figure(1);
set(myFigure, 'Position', [0,0,700,900]);

brightRcolorset = [255, 255, 255,   0,   0, 127, 255];
brightGcolorset = [  0, 128, 255, 255, 128,   0,   0];
brightBcolorset = [  0,   0,   0,   0, 255, 255, 255]; 
darkRcolorset = floor(brightRcolorset/2);
darkGcolorset = floor(brightGcolorset/2);
darkBcolorset = floor(brightBcolorset/2);

LastBrightEllipses = [0;0;0;0;0];
LastBrightColors = [1];
LastDarkEllipses = [0;0;0;0;0];
LastDarkColors = [1];
threshold = -2; 
memoryLoss = 1.3;

for f = 1:N
    filename = sprintf(format,f);
    C = imread(filename);
    %C = vidFrames(:,:,:,f); 
    %C = imresize(C,0.25,'bicubic'); 
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
            Q(mserC1) = floor(brightRcolorset(colorIndex)/2);
            Q(mserC2) = floor(brightGcolorset(colorIndex)/2);
            Q(mserC3) = floor(brightBcolorset(colorIndex)/2);
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
            Q(mserC1) = floor(darkRcolorset(colorIndex)/2);
            Q(mserC2) = floor(darkGcolorset(colorIndex)/2);
            Q(mserC3) = floor(darkBcolorset(colorIndex)/2);
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
    title('Tracked MSER Regions & Fitted Ellipses');
    set(gca,'FontSize',16,'fontWeight','bold')    
    %Plot ellipses
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    brightEllipsesTrans = vl_ertr(BrightEllipses);
    vl_plotframe(darkEllipsesTrans,'k-');
    vl_plotframe(brightEllipsesTrans, 'w-');
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    writeVideo(writer,frame);
    %pause
end
close(writer);