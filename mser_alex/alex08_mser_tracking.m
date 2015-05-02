%{
alex08_mser_tracking
Next evolution of alex05_mser. Includes particle filter implementation for
robust tracking and resillience to occlusions. 
%}

clear
clc
close all
run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose
format = '../PyBioSim-master/BioSim_Output_Images/%08d.jpg'; 
N = 300; %Number of frames in video.
global C; %I'm making the original color image from PyBioSim globally available.

%% Set up video output
writer = VideoWriter('MSER_Tracking','MPEG-4');
writer.Quality = 100;
writer.FrameRate = 30;
open(writer);
myFigure = figure(1);
set(myFigure, 'Position', [0,0,1280,720]);

%% Set up data structures

tracks = {}; %use cell array to store multiple instances of track data structures

%% Initialize with first frame
    f = 1;
    filename = sprintf(format, 100000000 + f);
    C = imread(filename);
    I = rgb2gray(C);
    I = adapthisteq(I);
    %Compute MSER belw; tuning knobs included in function call.
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.001,'MaxArea',0.003,'BrightOnDark',0,'DarkOnBright',1);
    %Create tracks & initialize with arbitrary info
    for i = 1:size(DarkEllipses,2)
        tracks{i} = track;
        tracks{i}.add_msmt(DarkEllipses(:,i));
        row = round(DarkEllipses(1,i));
        col = round(DarkEllipses(2,i));
        r = C(row,col,1); 
        g = C(row,col,2); 
        b = C(row,col,3); 
        tracks{i}.set_color(r,g,b);
    end    
    %pause
%% Track with rest of frames
for f = 2:N
    %% Import and resize image
    filename = sprintf(format, 100000000 + f);
    C = imread(filename);
    I = rgb2gray(C);
    I = adapthisteq(I);
    %% Detect MSERs
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.001,'MaxArea',0.003,'BrightOnDark',0,'DarkOnBright',1);
            
    %% Subplot 1
    subplot(1,2,1);     
    imshow(C);
    title('Original Image With Particles & Tracks');
    set(gca,'FontSize',16,'fontWeight','bold')     
    hold on
    for i = 1:size(tracks,2)
        tracks{i}.assoc_msmt(DarkEllipses);
        tracks{i}.update_particles();
        tracks{i}.plot_stuff;
    end   
    hold off   
    
    %% Fill MSERs
    S=128*ones(size(I,1),size(I,2),'uint8');
    for i=1:size(Dark,1)
        mser=vl_erfill(I,Dark(i));
        S(mser)=i*1*(size(Dark,1)-i);
    end
    %% Subplot 2
    subplot(1,2,2);
    imshow(S)  
    title('MSER Output');
    set(gca,'FontSize',16,'fontWeight','bold') 
    %% Plot ellipses
    darkEllipsesTrans = vl_ertr(DarkEllipses); %convert from R,C coordinates to X,Y coordinates
    vl_plotframe(darkEllipsesTrans,'r-');  
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    writeVideo(writer,frame);
    %pause
end
close(writer);