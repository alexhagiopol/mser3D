function videoOutput(I,Q,myFigure, writer)
    %% Video plumbing
    %Quadrant 1: Original grayscale
    subplot(2,1,1);
    imshow(I); %original grayscale
    title('Original Grayscale Video');
    set(gca,'FontSize',16,'fontWeight','bold')
    % Quadrant 2: MSER color
    subplot(2,1,2);
    imshow(Q)  
    title('Tracked MSER Regions in Color');
    set(gca,'FontSize',16,'fontWeight','bold')    
    %% Produce and write video frame 
    frame = frame2im(getframe(myFigure));
    writeVideo(writer,frame);
end