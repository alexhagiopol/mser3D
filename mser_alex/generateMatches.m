%% Matches data format:
    %{
    ***Matches are above + below eachother. If the match values are all equal to 0, there is no match***
    current_center_X_1          current_center_X_2           ...
    current_center_Y_1          current_center_Y_2           ...
    current_cov_1_1             current_cov_1_2              ...
    current_cov_2_1             current_cov_2_2              ... 
    current_cov_3_1             current_cov_3_2              ... 
    prev_match_center_X_1       prev_match_center_X_2        ...
    prev_match_center_Y_1       prev_match_center_Y_2        ...
    prev_match_cov1_1           prev_match_cov_1_2           ...
    prev_match_cov2_1           prev_match_cov_2_2           ...
    prev_match_cov3_1           prev_match_cov_3_2           ...
    %}

function [matches, match_summary, newColors] = generateMatches(prevEllipses,newEllipses, prevColors, threshold)
    numPrevRegions = size(prevEllipses,2);
    numNewRegions = size(newEllipses,2);
    %matches is a matrix that contains the score
    match_scores = zeros(numNewRegions,numPrevRegions);    
    %consider each possible match and assign a score in match_scores matrix
    for p = 1:numPrevRegions
        for n = 1:numNewRegions
            differences = abs(newEllipses(:,n) - prevEllipses(:,p));
            differences(1:2,:) = differences(1:2,:)*20; %increase importance of location
            score = -1*mean(abs(differences./newEllipses(:,n)));
            match_scores(n,p) = score;
        end
    end
    
    match_summary = -1*ones(2,numNewRegions); %the indices of matches match the indices of numNewRegions. At each index, we assign the best unique match, if any. We assign -1 if no match
    for i = 1:numNewRegions
        [max_score, index] = max(match_scores(:));
        [n,p] = ind2sub(size(match_scores),index);
        match_scores(n,:) = -Inf; %ensure future match uniqueness
        match_scores(:,p) = -Inf; %ensure future match uniqueness
        if max_score > -Inf
            match_summary(1,n) = p;
            match_summary(2,n) = max_score;            
        end
    end
    
    newColors = zeros(3,numNewRegions);
    matches = [newEllipses;-1*ones(size(newEllipses))];
    for i = 1:numNewRegions
        if match_summary(1,i) > 0 && match_summary(2,i) > threshold
            matches(6:10,i) = prevEllipses(:,match_summary(1,i));
            newColors(:,i) = prevColors(:,match_summary(1,i));
        else
            newColors(:,i) = randi([0,255],3,1);
        end
    end
    
    disp(match_scores);
    disp(match_summary);
    disp(matches);
end