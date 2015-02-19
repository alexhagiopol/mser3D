%Algorithm 2 from Donoser07thesis
%Finds the canonical element of the disjoint set that pixel x belongs to.
function result = mser_find(x)
    if par_set(x) ~= x
        par_set = mser_find(Par_set(x));
    end
    result = par_set(x);
end