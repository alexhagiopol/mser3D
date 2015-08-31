%return match information based on ellipse matching
function match_summary = matchEllipses(prevEllipses, newEllipses)
    numPrevRegions = size(prevEllipses,2);
    numNewRegions = size(newEllipses,2);
    %matches is a matrix that contains the score
    match_scores = zeros(numNewRegions,numPrevRegions);    
    %consider each possible match and assign a score in match_scores matrix
    for p = 1:numPrevRegions
        for n = 1:numNewRegions
            differences = abs(newEllipses(1:5,n) - prevEllipses(1:5,p));
            differences(1:2,:) = differences(1:2,:)*20; %increase importance of location
            score = -1*mean(abs(differences./newEllipses(1:5,n)));
            match_scores(n,p) = score;
        end
    end

    %{
    match_summary explanation:
    prev regions object index:   8,   6,    9,   4,   3
    curr regions object index:   9,   5,    4,   8,   1
    match score:               2.1, 3.4, -5.6, 1.1, 1.4  
    %}           
    match_summary = -1*ones(3,numNewRegions); 
    x = 1;
    while x <= numNewRegions
        [max_score, index] = max(match_scores(:));
        [n,p] = ind2sub(size(match_scores),index);
        match_scores(n,:) = -Inf; %ensure future match uniqueness
        match_scores(:,p) = -Inf; %ensure future match uniqueness
        if max_score > -Inf
            match_summary(1,x) = prevEllipses(6,p);
            match_summary(2,x) = newEllipses(6,n);
            match_summary(3,x) = max_score;            
        end
        disp(['index = ',num2str(x)]);
        disp('array');
        disp(match_summary);
        x = x + 1;
    end
    

end