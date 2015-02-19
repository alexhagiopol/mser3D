% alex01_mser
% Uses code from frank01_mser

clc
close all
run('vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose


%% Import drone video (comment this section out after you import once and store video frames in workspace; this takes ~5.6GB of RAM)

disp('Starting Video Import');
readerobj = VideoReader('through_the_cracks_FPV.mp4', 'tag', 'myreader1');
vidFrames = read(readerobj);
N = get(readerobj, 'NumberOfFrames');
disp('Video Import Finished');


%% Start video writer
writer = VideoWriter('MSER_Video_0','MPEG-4');
writer.Quality = 100;
writer.FrameRate = 30;
open(writer);
frameTimes = zeros(N,1);
myFigure = figure(1);
set(myFigure, 'Position', [0,0,1200,600]);

%% Process each video frame 
for f=1:N 
    %% Read image from video and resize + grayscale
    C = vidFrames(:,:,:,f); 
    C = imresize(C,0.25,'bicubic');   
    I=rgb2gray(C);
    I=adapthisteq(I);
    
    %% Detect MSERs (needs vlfeat)
    Bright=vl_mser(I,'MinArea',0.005,'MaxArea',0.05,'BrightOnDark',1,'DarkOnBright',0);
    Dark=vl_mser(I,'MinArea',0.005,'MaxArea',0.05,'BrightOnDark',0,'DarkOnBright',1);
    
    %% Fill some mser regions
    S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result
    R = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result
      
    for i=size(Bright,1):-1:1
        mserS=vl_erfill(I,Bright(i));        
        S(mserS)=255-1*(size(Bright,1)-i); %set brightness of grayscale
        colorIndex = mod(i,3) + 1; %Ensure non-zero result less than 7
        %convert mser region location in grayscale to mser region location
        %in color
        [rS,cS] = ind2sub(size(S), mserS);
        mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
        mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
        mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));        
        %assign average color of original video location
        R(mserC1) = mean(mean(C(mserC1)));
        R(mserC2) = mean(mean(C(mserC2)));
        R(mserC3) = mean(mean(C(mserC3)));        
    end
    for i=1:size(Dark,1)
        mser=vl_erfill(I,Dark(i));
        S(mser)=i*1*(size(Dark,1)-i);  %set brightness of grayscale
        colorIndex = mod(i,length(Rcolorset)) + 1; %Ensure non-zero result less than 7
        %convert mser region location in grayscale to mser region location
        %in color
        [rS,cS] = ind2sub(size(S), mserS);
        mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
        mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
        mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1));
        %assign average color of original video location
        R(mserC1) = mean(mean(C(mserC1)));
        R(mserC2) = mean(mean(C(mserC2)));
        R(mserC3) = mean(mean(C(mserC3)));
    end    
    %% 4 quadrant video display
    subplot(2,2,1);
    imshow(I); %original grayscale
    title('Original Grayscale Video');
    set(gca,'FontSize',16,'fontWeight','bold')
    
    subplot(2,2,2);
    imshow(C)  %original color
    title('Original Color Video');
    set(gca,'FontSize',16,'fontWeight','bold')
    
    subplot(2,2,3);
    imshow(S); %mser grayscale  
    title('MSER Regions in Grayscale');
    set(gca,'FontSize',16,'fontWeight','bold')
    
    subplot(2,2,4);
    imshow(R); %mser color
    title('MSER Regions in Color');
    set(gca,'FontSize',16,'fontWeight','bold')
        
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    writeVideo(writer,frame);
    
end
close(writer);

