% alex03_mser
% Needs VL Feat library
% Uses code from frank01_mser

clc
close all
run('vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
vl_version verbose

%% Tuning parameters
startingFrame = 1;

%% Import video (comment this section out after you import once and store video frames in workspace; this takes ~5.6GB of RAM)
%{
disp('Starting Video Import');
readerobj = VideoReader('Drone_Video_Edited.mp4', 'tag', 'myreader1');
vidFrames = read(readerobj);
N = get(readerobj, 'NumberOfFrames');
disp('Video Import Finished');
%}

%% Set up video output
writer = VideoWriter('MSER_Video_Crude_Tracking_4','MPEG-4');
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

%% Process first video frame
f = startingFrame;
C = vidFrames(:,:,:,f); 
C = imresize(C,0.25,'bicubic');   
I = rgb2gray(C);
I = adapthisteq(I);    
S = 128*ones(size(I,1),size(I,2),'uint8'); %grayscale result; leave this the same.
Q = 128*ones(size(I,1),size(I,2),3,'uint8'); %color result; edit this with MSERs   
%Compute MSERs
[BrightMSERs, BrightEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.005,'MaxArea',0.03,'BrightOnDark',1,'DarkOnBright',0);
[DarkMSERs, DarkEllipses] = vl_mser(I,'MinDiversity',0.7,'MinArea',0.005,'MaxArea',0.03,'BrightOnDark',0,'DarkOnBright',1);
%{
Regions Matrix Format:
MSER number
ellipse param 1
ellipse param 2
ellipse param 3
ellipse param 4
ellipse param 5
color
votes
%}
TrackedRegionsMatrix = [];
for i = 1:size(BrightMSERs,1)                     %MSER num,      ellipse params      color index   votes  
    TrackedRegionsMatrix = [TrackedRegionsMatrix, [BrightMSERs(i);BrightEllipses(:,i); mod(i,7) + 1;0;]];
end
for i = 1:size(DarkMSERs,1)
    TrackedRegionsMatrix = [TrackedRegionsMatrix, [DarkMSERs(i);DarkEllipses(:,i); mod(i,7) + 1;0;]];
end
%Color in MSER regions
for i = 1:size(TrackedRegionsMatrix, 2)
    mserS = vl_erfill(I,TrackedRegionsMatrix(1,i));
    [rS,cS] = ind2sub(size(S), mserS);
    mserC1 = sub2ind(size(C), rS, cS, 1*ones(length(rS),1));
    mserC2 = sub2ind(size(C), rS, cS, 2*ones(length(rS),1));
    mserC3 = sub2ind(size(C), rS, cS, 3*ones(length(rS),1)); 
    Q(mserC1) = brightRcolorset(colorIndex);
    Q(mserC2) = brightGcolorset(colorIndex);
    Q(mserC3) = brightBcolorset(colorIndex);
end
videoOutput(I,Q,myFigure,writer);
%% Process each subsequent video frame 
close(writer);



