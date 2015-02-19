function reference_sequence_mser(input_format, output_format, N)
    run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
    vl_version verbose
    for f = 1:N
        input_filename = sprintf(input_format,f);        
        I1 = rgb2gray(imread(input_filename));
        Bright = vl_mser(I1,'BrightOnDark',1,'DarkOnBright',0);
        Dark = vl_mser(I1,'BrightOnDark',0,'DarkOnBright',1); 
        S = 128*ones(size(I1,1),size(I1,2),'uint8');
        for i=size(Bright,1):-1:1
            mser=vl_erfill(I1,Bright(i));
            S(mser)=255-1*(size(Bright,1)-i);
        end
        for i=1:size(Dark,1)
            mser=vl_erfill(I1,Dark(i));
            S(mser)=i*1*(size(Dark,1)-i);
        end
        output_filename = sprintf(output_format, f);
        imwrite(S,output_filename,'jpg');
        imshow(S);
        %pause;
    end
end