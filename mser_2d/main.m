%% Driver function with automatic or manual selection of MSER Measurement data associations

%% Initialize
clc
%clear
close all
run('../vlfeat-0.9.20/toolbox/vl_setup') % start up vl_feat
vl_version verbose

%% Set tuning constants 
%VL Feat tuning constants
MinDiversity = 0.7; 
MinArea = 0.005; 
MaxArea = 0.03; 
% Alex tuning constants
start = 1; %start at custom frame number. Default = 1.
stop = 10;  %end at custom frame number. Default = N.
format = '../datasets/StillImages/Frame%04d.bmp'; % framing
manualMatching = true;
visualization = false;
resize = false; %reduce image to speed up computation
threshold = -1; %-1 %Score threshold needed for two regions to be considered to come from the same object. A higher score indicates higher similarity.
measurementsOutputFileName = '../datatsets/tempManualMserMeasurements.csv';
videoOutputFileName = 'Alex_Tracking_Movie';

%% Set up video output
if visualization
    writer = VideoWriter(videoOutputFileName,'Uncompressed AVI'); %AVI required because mp4 doesnt work on Matlab Linux :(
    writer.FrameRate = 10;
    open(writer);
    three_pane_fig = figure(1);
    set(three_pane_fig, 'Position', [0,0,2100,700]); 
end

%% Process first video frame 
f = start;
filename = sprintf(format,f); %f-1 because images are 0 indexed
% Read image from video and resize + grayscale
C = imread(filename);
if resize
    C = imresize(C,0.5);
end
I=rgb2gray(C);
%Detect MSERs
[SeedValues, EllipsesValues] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0);
numRegions = size(EllipsesValues,2);
%Create object collection!
mainOC = ObjectCollection(size(I,1),size(I,2),stop - start + 1); %tell the object data structure that the last #numRegs objects are associated with the last frame 
mainOC.addImage(f,I);
%Create objects!
for i = 1:numRegions
    Rimg = C(:,:,1);
    Gimg = C(:,:,2);
    Bimg = C(:,:,3);
    mser_pixels = vl_erfill(I, SeedValues(i));
    avgR = mean(mean(Rimg(mser_pixels)));
    avgG = mean(mean(Gimg(mser_pixels)));
    avgB = mean(mean(Bimg(mser_pixels)));
    myMSER = MSER(EllipsesValues(:,i),SeedValues(i),f,avgR,avgG,avgB); %store MSER info
    color = [avgR;avgG;avgB];%randi([0,255],3,1); %generate random color
    %Object ID: 1.012 = frame #1, region #12. That's why we divide by 1000.
    myObject = Object(myMSER,f+i/1000,f,color); %create new object for each mser
    mainOC.addObject(myObject); %add to big data structure
end
prevIm = mainOC.getImage(I,f);

%% Process rest of frames + make video
for f=start + 1:stop
    %% Read image, resize, grayscale, make data structures, & detect MSERs
    filename = sprintf(format,f); %f-1 because images are 0 indexed
    C = imread(filename);
    if resize
        C = imresize(C,0.5);
    end
    I=rgb2gray(C);
    [SeedValues, EllipsesValues] = vl_mser(I,'MinDiversity',MinDiversity,'MinArea',MinArea,'MaxArea',MaxArea,'BrightOnDark',1,'DarkOnBright',0); %Set to compute dark on bright!!!
    mainOC.addImage(f,I);
    numRegions = size(SeedValues,1); %number of regions
    tempOC = ObjectCollection(size(I,1),size(I,2),1); %tell the object data structure that the last #numRegs objects are associated with the last frame 
    %Create objects!
    for i = 1:numRegions
        Rimg = C(:,:,1);
        Gimg = C(:,:,2);
        Bimg = C(:,:,3);
        mser_pixels = vl_erfill(I, SeedValues(i));
        avgR = mean(mean(Rimg(mser_pixels)));
        avgG = mean(mean(Gimg(mser_pixels)));
        avgB = mean(mean(Bimg(mser_pixels)));
        myMSER = MSER(EllipsesValues(:,i),SeedValues(i),f,avgR,avgG,avgB); %store MSER info
        color = [avgR;avgG;avgB];%randi([0,255],3,1); %generate random color
        %Object ID: 1.012 = frame #1, region #12. That's why we divide by 1000.
        myObject = Object(myMSER,f+i/1000,f,color); %create new object for each mser
        tempOC.addObject(myObject); %add to big data structure
    end
    newIm = tempOC.getImage(I,f);
    %Perform matching
    if manualMatching == true
        mainOC.manualMatchObjects(f-1,f,prevIm,newIm,threshold,tempOC);
    else %do standard automatic matching
        mainOC.matchObjects(f-1,f,threshold,tempOC);
    end    
    
    if visualization  && ~manualMatching
        %% Display results
        %Show raw image, prev frame, new frame
        %figure;
        %hold on;
        frames = [C,prevIm,newIm];
        imshow(frames)       
        hold on;
        title(['Original Frame # ',num2str(f), '                          MSER Previous Frame                           MSER Current Frame ']);
        set(gca,'FontSize',13);    
        %Get matches between current and past frame in matrix form
        matches = mainOC.getMatchCoords(f);
        
        if size(matches,2) > 0
            matchesTrans = [vl_ertr(matches(1:5,:));vl_ertr(matches(6:10,:))]; %Change from XY elipses to RC ellipses
            %Shift locations over because of image concatenation
            matchesTrans(1,:) = matchesTrans(1,:) + 2*size(C,2);
            matchesTrans(6,:) = matchesTrans(6,:) + size(C,2);    
            %Draw lines
            for i = 1:size(matchesTrans,2)
                if matchesTrans(6,i) > 0 && matchesTrans(7,i) > 0 %check if match actually exists 
                    line([matchesTrans(1,i),matchesTrans(6,i)],[matchesTrans(2,i),matchesTrans(7,i)],'Color',[1, 1, 1]);
                end
            end
        end         
        
        %%Plot MSER Measurements as unit test to ensure mserMeasurement objects make sense
        for i = 1:size(EllipsesValues,2)
            ellipse = vl_ertr(EllipsesValues(:,i)); %crucial to use vl_ertr
            mserMeasurement = mainOC.covarianceEllipseToMserMeasurement(ellipse);
            % Plot returned ellipse as test
            % Get the 95% confidence interval error ellipse
            chisquare_val = 2.4477;
            theta_grid = linspace(0,2*pi);
            phi = mserMeasurement(5);
            X0=mserMeasurement(1);
            Y0=mserMeasurement(2);
            a=mserMeasurement(3);
            b=mserMeasurement(4);

            % the ellipse in x and y coordinates 
            ellipse_x_r  = a*cos( theta_grid );
            ellipse_y_r  = b*sin( theta_grid );

            %Define a rotation matrix
            R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

            %let's rotate the ellipse to some angle phi
            r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;            
            
            % Draw the error ellipse            
            plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'r.')   
            text(X0,Y0,['MSMT', num2str(i)],'Color',[1.0,0,0]);            
        end
        
        %% Produce and write video frame 
        frame = frame2im(getframe(three_pane_fig));
        writeVideo(writer,frame); 
        %hold on;
         
    else
        disp(['Frame #',num2str(f),' done'])
    end
    prevIm = newIm;
    %pause
end
if visualization
    close(writer);
end
%Make CSV data file
mainOC.exportMserMeasurementsInGroups(1,measurementsOutputFileName);
if manualMatching == true
    mainOC.showTracks(3,size(I,1),size(I,2),2);
    mainOC.makeTrackVideo(5,size(I,1),size(I,2),2);
end

if visualization
    %% Show MSER tracks in stills and videos. WARNING: These two commands could take hours!
    %mainOC.showTracks(3,size(I,1),size(I,2),2);
    %mainOC.makeTrackVideo(5,size(I,1),size(I,2),2);
    
    %% Compute and display statistics
    mser_counts = mainOC.computeTrackLengths();
    stdev = std(mser_counts);
    avg = mean(mser_counts);
    maxi = max(mser_counts);
    mini = min(mser_counts);
    disp(['Avg length of tracks = ',num2str(avg)]);
    disp(['Max length of tracks = ',num2str(maxi)]);
    disp(['Min length of tracks = ',num2str(mini)]);
    disp(['STDEV length of tracks = ',num2str(stdev)]);
    fig = figure;
    set(fig, 'Position', [0,0,800,500]);
    edges = [round(mini):1:round(avg+stdev),50];
    histogram(mser_counts,edges);  
    ylim([0,450]);
    ylabel('Number of Tracks');
    xlabel('Number of MSERs per Track');
    title('MSER Track Statistics Using Multiple (6) Frame Matching for Dark on Bright MSERs');
    line([avg,avg],[0,450],'Color',[1,0,0],'LineStyle','-');
    line([avg + stdev,avg + stdev],[0,450],'Color',[1,0,0],'LineStyle','--');

    fig2 = figure;
    set(fig2, 'Position', [0,0,1500,500]);
    tracks_per_frame = mainOC.computeGoodTracksPerFrame(round(stdev + avg));
    plot(tracks_per_frame,'b*');
    ylim([0,18]);
    ylabel('Number of Good Tracks');
    xlabel('Frame #');
    title('Good Dark on Bright Tracks Visible in Each Video Frame');
end

close all;
    
    
    
    
    
    
    
    
    
    
    
    
    
    