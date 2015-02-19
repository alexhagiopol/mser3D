%Algorithm 3 from Donoser07thesis
%Merges two disjoint sets, defined by their
%canonical elements x and y. The canonical element
%of the tree with the largest depth becomes the root
%of the union.
function pixel = mser_merge(par_set, rnk_set, x,y)
    if rnk_set(x) > rnk_set(y)
        par_set(y) = x;
    elseif rnk_set(x) == rnk_set(y)
        rnk_set(y) = rnk_set(y) + 1;
        par_set(x) = y;
    else
        par_set(x) = y;
    end
    pixel = y;
end