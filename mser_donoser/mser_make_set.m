%Algorithm 1 from Donoser07thesis
%Makes a new disjoint set X.
function mser_make_set(par_set, rnk_set, x)
    par_set(x) = x;
    rnk_set(x) = 0;
end