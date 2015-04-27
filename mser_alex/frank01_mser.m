% frank01_mser

clear
clc
close all

run('../vlfeat-0.9.19/toolbox/vl_setup')
vl_version verbose

% format = 'Sequence1/frame%02d.jpg'; N = 48; % framing
format = '../Image Storage/images_input/Sequence2/frame%02d.jpg'; N = 50; % indoor
% format = 'Sequence3/frame%02d.jpg'; N = 47; % hulk
% format = 'Sequence4/frame%03d.jpg'; N = 101; % trees
% format = 'Sequence5/frame%03d.jpg'; N = 193; % garage
h = fspecial('gaussian', [5 5], 1 );
for f=1:N
    %% Read image
    filename = sprintf(format,f);
    C = imread(filename);
    
    %% Convert to smaller grayscale
    S=imresize(C,0.25,'bicubic');
    subplot(2,1,1);
%     imshow(S);
    I=rgb2gray(S);
    I=adapthisteq(I);
    imshow(I);
%     I=imfilter(I,h);
    
    %% Detect MSER (needs vlfeat)
    Bright=vl_mser(I,'MinArea',0.005,'MaxArea',0.05,'BrightOnDark',1,'DarkOnBright',0);
    Dark=vl_mser(I,'MinArea',0.005,'MaxArea',0.05,'BrightOnDark',0,'DarkOnBright',1);
    
    %% Fill some mser regions
    S=128*ones(size(I,1),size(I,2),'uint8');
    for i=size(Bright,1):-1:1
        mser=vl_erfill(I,Bright(i));
        S(mser)=255-1*(size(Bright,1)-i);
    end
    for i=1:size(Dark,1)
        mser=vl_erfill(I,Dark(i));
        S(mser)=i*1*(size(Dark,1)-i);
    end
    subplot(2,1,2);
    imshow(S)
    pause 
end