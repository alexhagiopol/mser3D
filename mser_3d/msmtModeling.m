clear
clc
close all

run('../vlfeat-0.9.19/toolbox/vl_setup')
vl_version verbose
format = '/home/alex/mser/mser_3d/output2/img%010d.jpg';
N = 100;
three_pane_fig = figure(1);
set(three_pane_fig, 'Position', [0,0,2100,700]);
MSER_STATS = zeros(768,1024);
for f = 1:N
    rand_MinArea = 0.001 + 0.009*rand();
    rand_MaxArea = 0.1 + 0.9*rand();
    rand_MinDiversity = 0.1 + 0.9*rand();
    filterFactor = round(2+75*rand());
    rand_filter = fspecial('gaussian',[filterFactor,filterFactor],filterFactor);    
    filename = sprintf(format,f);
    C = imread(filename);
    C = imfilter(C,rand_filter,'same');
    I = rgb2gray(C);
    
    %subplot(1,3,1);
    %imshow(C);
    %subplot(1,3,2);
    %imshow(I);
    
    Bright=vl_mser(I,'MinArea',rand_MinArea,'MaxArea',rand_MaxArea,'MinDiversity',rand_MinDiversity,'BrightOnDark',1,'DarkOnBright',0);
    Dark=vl_mser(I,'MinArea',rand_MinArea,'MaxArea',rand_MaxArea,'MinDiversity',rand_MinDiversity,'BrightOnDark',0,'DarkOnBright',1);
    S=128*ones(size(I,1),size(I,2),'uint8');
    for i=size(Bright,1):-1:1
        mser=vl_erfill(I,Bright(i));
        S(mser)=255-1*(size(Bright,1)-i);
        MSER_STATS(mser) = MSER_STATS(mser) + 1;
    end
    for i=1:size(Dark,1)
        mser=vl_erfill(I,Dark(i));
        S(mser)=i*1*(size(Dark,1)-i);
        MSER_STATS(mser) = MSER_STATS(mser) + 1;
    end
    %subplot(1,3,3);
    %imshow(S);
    %pause
    disp(f);
end

figure;
MSER_STATS = MSER_STATS(50:end-50,50:end-50);
mesh(MSER_STATS'/N);