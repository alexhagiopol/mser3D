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
%Choose random starting colors for regions
numRegs = size(BrightEllipses,2);
%Create object farm!
myOF = objectFarm(numRegs,stop - start + 1); %tell the object data structure that the last #numRegs objects are associated with the last frame 
%Create objects!
for i = 1:numRegs
    myMSER = MSER(BrightEllipses(:,i),Bright(i),f); %store MSER info
    color = randi([0,255],3,1); %generate random color
    %Object ID: 1.012 = frame #1, region #12. That's why we divide by 1000.
    myObject = worldObject(myMSER,f+i/1000,f,color); %create new object for each mser
    myOF.addObject(myObject); %add to big data structure
end
%imshow(myOF.getImage(I,f));
