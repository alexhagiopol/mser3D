function similarity = compare_sequence_mser(input_format, output_format, N)
%{ 
This program assumes... 
    1. default tuning parameters of the VL Feat MSER implementation
    2. input and output images have the same dimensions and are grayscaled

N is the number of images in a sequence

input format is the filename format corresponding to the sequence of
input images. the %02d in the example gets replaced with the number of
the image in the sequences
example: '../images_input/Sequence1/frame%02d.jpg'
 
output format is the filename format corresponding to the sequence of
images created by the function to be tested. 
example: '../images_output/Reference_Standard/Sequence1/frame%02d.jpg'
 
note: in %02d, the % means escape, the 0 means add leading zeros, the 2
means convert to minimum of 2 digits, the d means convert to integer
see http://www.mathworks.com/help/matlab/ref/sprintf.html?refresh=true

example function call: compare_sequence_mser('../images_input/Sequence1/frame%02d.jpg','../images_output/frame%02d.jpg',48);

name = marilyn
%}
    run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
    vl_version verbose
    similarity = ones(1,N);
    for f = 1:N
        %% Generate Reference MSER image
        input_filename = sprintf(input_format,f);        
        I1 = rgb2gray(imread(input_filename));
        Bright = vl_mser(I1,'BrightOnDark',1,'DarkOnBright',0);
        Dark = vl_mser(I1,'BrightOnDark',0,'DarkOnBright',1); 
        S1 = 128*ones(size(I1,1),size(I1,2),'uint8');
        for i=size(Bright,1):-1:1
            mser=vl_erfill(I1,Bright(i));
            S1(mser)=255-1*(size(Bright,1)-i);
        end
        for i=1:size(Dark,1)
            mser=vl_erfill(I1,Dark(i));
            S1(mser)=i*1*(size(Dark,1)-i);
        end        
        %% Compare to output from other function
        output_filename = sprintf(output_format,f);
        S2 = imread(output_filename);
        diff = abs(S1 - S2);
        avg_diff = mean(diff(:));
        similarity(f) = avg_diff;
        %% Show Results
        subplot(2,1,1);
        imshow(S1);
        subplot(2,1,2);
        imshow(S2);
        pause;
    end
    similarity = 1 - mean(similarity)/255;
end