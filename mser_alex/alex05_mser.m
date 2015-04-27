%alex05_mser
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
writer.FrameRate = 0.5;
open(writer);
myFigure = figure(1);
set(myFigure, 'Position', [0,0,700,900]);

for f = 1:N
    %% Import and resize image
    filename = sprintf(format, 100000000 + f);
    C = imread(filename);
    I = rgb2gray(C);
    %I = I(400:2000,400:2800);
    %I = imresize(I, 0.25, 'bicubic');
    I = adapthisteq(I);
    %% Detect MSERs
    %[Bright, BrightEllipses] = vl_mser(I,'MinDiversity',0.07,'MinArea',0.0005,'MaxArea',0.03,'BrightOnDark',1,'DarkOnBright',0,'Verbose');
    [Dark, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.001,'MaxArea',0.003,'BrightOnDark',0,'DarkOnBright',1,'Verbose');
    
    %% Fill MSERs
    S=128*ones(size(I,1),size(I,2),'uint8');
    %{
    for i=size(Bright,1):-1:1
        mser=vl_erfill(I,Bright(i));
        S(mser)=255-1*(size(Bright,1)-i);
    end
    %}
    for i=1:size(Dark,1)
        mser=vl_erfill(I,Dark(i));
        S(mser)=i*1*(size(Dark,1)-i);
    end
    
    %% Subplot 1
    subplot(2,1,1);
    imshow(C);
    title('Original Image');
    set(gca,'FontSize',16,'fontWeight','bold')
    
    %% Subplot 2
    subplot(2,1,2);
    imshow(S)  
    title('Tracked MSER Regions & Fitted Ellipses');
    set(gca,'FontSize',16,'fontWeight','bold') 
    %% Plot ellipses
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    %brightEllipsesTrans = vl_ertr(BrightEllipses);
    vl_plotframe(darkEllipsesTrans,'r-');
    %vl_plotframe(brightEllipsesTrans, 'w-');   
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    %writeVideo(writer,frame);
    %pause
end
close(writer);
