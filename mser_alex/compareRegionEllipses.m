%{
This function assumes that two MSER regions are similar if they
have similar ellipsis parameters as measured by vl_mser. 

Form of inputs and outputs:

prevEllipses is a matrix whose columns represent fitted ellipses as 
calculated by vl_mser. Each column will have 5 entries corresponding to the 
ellipsis parameters calculated by vl_mser. prevEllipses is calculated based
on the previous video frame.

newEllipses is the same as prevEllipses, except it is calculated from the
current video frame. *newEllipses might not have the same number of columns
as prevEllipses*.

ellipsesScores is a matrix with the same number of columns as
newEllipses. Each column in ellipsesScores will correspond to a column in newEllipses.
Each column of ellipsesScores will contiain two elements. The
first element will be the index of the column from prevEllipses that
best matches the corresponding column from newEllipses. The second element
will be a score that indicates the strength of the match. The best match of
a given column is the column that has the lowest sum of absolute differences.
The score is expressed as a percentage: 100 - 100*abs(column1 -
column2)/abs(column1). The score will allow the caller of this function to
decide whether or not to assign the same color 

%}
function ellipsesScores = compareRegionEllipses(prevEllipses, newEllipses)
    ellipsesScores = zeros(2,size(newEllipses,2));
    for i = 1:size(newEllipses,2)
        bestIndex = 1;
        bestScore = -Inf;
        prevEllipses;
        newEllipses;
        for j = 1:size(prevEllipses, 2)            
            differences = abs(newEllipses(1:2,i) - prevEllipses(1:2,j));
            differences(1:2,:) = differences(1:2,:)*5;
            score = -1*mean(differences./abs(newEllipses(1:2,i)));
            %disp(['score = ',num2str(score)]);
            if score > bestScore
                bestIndex = j;
                bestScore = score;
            end
        end
        ellipsesScores(1,i) = bestIndex;
        ellipsesScores(2,i) = bestScore;
        %disp(['BEST = ',num2str(bestScore)]);
    end
end