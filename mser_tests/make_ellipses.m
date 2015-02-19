function make_ellipses(input_filename)
%example function call: make_ellipses('../images_input/Sequence1/frame01.jpg')
    run('../vlfeat-0.9.19/toolbox/vl_setup') % start up vl_feat
    I0 = imread(input_filename);
    I1 = rgb2gray(I0);
    t = cputime();
    [Bright, BrightEllipses] = vl_mser(I1,'Delta',2,'MinArea',0.0005,'MaxArea',0.1,'MaxVariation',0.5,'MinDiversity',0.5,'BrightOnDark',1,'DarkOnBright',0);
    [Dark, DarkEllipses] = vl_mser(I1,'Delta',2,'MinArea',0.0005,'MaxArea',0.1,'MaxVariation',0.5,'MinDiversity',0.5,'BrightOnDark',0,'DarkOnBright',1); 
    t = cputime() - t;
    disp(['Time elapsed = ',num2str(t), ' seconds.']);
    Ellipses = [BrightEllipses,DarkEllipses];
    fileID = fopen('../vlfeat_ellipses.txt','w');
    for i=1:size(Ellipses,2)
        for j=1:size(Ellipses,1)
            fprintf(fileID,' %f \n', Ellipses(j,i));
        end
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    darkEllipsesTrans = vl_ertr(DarkEllipses);
    brightEllipsesTrans = vl_ertr(BrightEllipses);
    my_figure = figure(1);
    set(my_figure, 'Position', [0,0,720,1280]);
    imshow(I0);
    vl_plotframe(darkEllipsesTrans,'k-');
    vl_plotframe(brightEllipsesTrans, 'w-');
    imwrite(frame2im(getframe(my_figure)),'../vlfeat_frame01.jpg','jpg');
end