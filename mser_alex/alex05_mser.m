%alex05_mser
%Naive version that produces hijacking effect.
%MSER computation using VL Feat for images output by the PyBioSim
%simulator. MSER tagging for tracking purposes. Data association? Particle
%filter for tracking?

clear
clc
close all
run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose
format = '../PyBioSim-master/BioSim_Output_Images/%08d.jpg'; 
N = 180;

%% Set up video output
writer = VideoWriter('MSER_Tracking','MPEG-4');
writer.Quality = 100;
writer.FrameRate = 30;
open(writer);
myFigure = figure(1);
set(myFigure, 'Position', [0,0,700,900]);

%% Initialize with first frame
    data_assocs = [];
    f = 1;
    filename = sprintf(format, 100000000 + f);
    C = imread(filename);
    I = rgb2gray(C);
    I = adapthisteq(I);
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.001,'MaxArea',0.003,'BrightOnDark',0,'DarkOnBright',1);
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    data_assoc = darkEllipsesTrans; %initialize data associations
    
%% Initialize with second frame
    f = 2;
    filename = sprintf(format, 100000000 + f);
    C = imread(filename);
    I = rgb2gray(C);
    I = adapthisteq(I);
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.001,'MaxArea',0.003,'BrightOnDark',0,'DarkOnBright',1);
    darkEllipsesTrans = vl_ertr(DarkEllipses); 
    
    %Perform naive data association
    ellipsesScores = compareRegionEllipses(darkEllipsesTrans,data_assoc(:,:)); 
    temp = [];
    for j = 1:size(data_assoc,2)
        temp = [temp, darkEllipsesTrans(:,ellipsesScores(1,j))];
    end
    data_assoc = [data_assoc; temp];
    
%% Track with rest of frames
for f = 3:N
    %% Import and resize image
    filename = sprintf(format, 100000000 + f);
    C = imread(filename);
    I = rgb2gray(C);
    I = adapthisteq(I);
    %% Detect MSERs
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.001,'MaxArea',0.003,'BrightOnDark',0,'DarkOnBright',1);
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    
    %% Perform naive data association
    ellipsesScores = compareRegionEllipses(darkEllipsesTrans,data_assoc(5*(f-2)+1:5*(f-1),:)); 
    temp = [];
    for j = 1:size(data_assoc,2)
        temp = [temp, darkEllipsesTrans(:,ellipsesScores(1,j))];
    end
    data_assoc = [data_assoc; temp];
    
    %% Fill MSERs
    S=128*ones(size(I,1),size(I,2),'uint8');
    for i=1:size(Dark,1)
        mser=vl_erfill(I,Dark(i));
        S(mser)=i*1*(size(Dark,1)-i);
    end    
    %% Subplot 1
    subplot(2,1,1);
    imshow(C);
    
    title('Original Image With Superimposed Tracks');
    set(gca,'FontSize',16,'fontWeight','bold') 
    hold on
    plot(data_assoc(1:5:end,1),data_assoc(2:5:end,1),'r.');
    plot(data_assoc(1:5:end,2),data_assoc(2:5:end,2),'b.');
    
    
    hold off
    %% Subplot 2
    subplot(2,1,2);
    imshow(S)  
    title('MSER Output');
    set(gca,'FontSize',16,'fontWeight','bold') 
    %% Plot ellipses
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    vl_plotframe(darkEllipsesTrans,'r-');  
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    writeVideo(writer,frame);
    %pause
end
close(writer);
