classdef object < handle
    properties
        msers; %cell array containing all MSERs
        id; %object id
        last_seen; %frame in which the object was last seen
        color; %[R,G,B]
        recent_index;
    end
    methods
        %initialize
        function O = object(first_mser,id_,frame_num,color_)
            O.msers = [O.msers, first_mser];
            O.id = id_;
            O.last_seen = frame_num;
            O.color = color_;
            O.recent_index = 1; %store index of most recent mser;
        end
    end    
end