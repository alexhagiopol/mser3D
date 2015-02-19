function img = reference_single_mser(input_filename)
        %This program produces and returns a single reference MSER image
        run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
        %vl_version verbose
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
        img = S1;
end