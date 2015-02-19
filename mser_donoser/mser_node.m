%Node class to be contained in component tree
classdef mser_node < handle
    properties
        id;
        level;
        children_ids;
    end    
    methods
        function MN = mser_node(new_id,new_level)
            MN.id = new_id;
            MN.level = new_level;
            MN.children_ids = [];
        end        
    end
end