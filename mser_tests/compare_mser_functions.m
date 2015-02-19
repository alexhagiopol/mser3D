function similarity = compare_mser_functions(input_format, N, func1_handle, func2_handle)
    %example function call: compare_sequence_mser('../images_input/Sequence1/frame%02d.jpg','../images_output/frame%02d.jpg',48);
    %make sure that the current directory contains this function
    similarity = ones(1,N);
    for f = 1:N
        input_filename = sprintf(input_format,f);
        S1 = func1_handle(input_filename);
        S2 = func2_handle(input_filename);
        diff = abs(S1 - S2);
        avg_diff = mean(diff(:));
        similarity(f) = avg_diff;
    end
    similarity = 1 - mean(similarity)/255;
end