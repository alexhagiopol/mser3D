function similarity = compare_ellipses(filename1, filename2)
    %example function call: compare_ellipses('../nister_ellipses.txt','../vlfeat_ellipses.txt');
    similarity = 0;
    fileID1 = fopen(filename1);
    fileID2 = fopen(filename2);
    data1 = textscan(fileID1,'%f32')
end