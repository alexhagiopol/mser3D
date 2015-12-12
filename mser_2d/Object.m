classdef Object < handle
    properties
        msers; %vector containing all MSERs
        id; %object id = equal to frame.regionNum
        last_seen; %frame in which the object was last seen
        color; %[R,G,B]
        recent_i; %index of most recent mser
    end
    methods
        %initialize
        function O = Object(first_mser,id_,frame_num,color_)
            O.msers = [first_mser, O.msers];
            O.id = id_;
            O.last_seen = frame_num;
            O.color = color_;
            O.recent_i = 1; %store index of most recent mser;
        end
        %get mser
        function mser = getLatestMSER(O)
            mser = O.msers(O.recent_i);
        end
        %add mser
        function addMSER(O, mser)
            O.recent_i = O.recent_i + 1;
            O.msers = [O.msers, mser];
        end
        %update most recent frame appearance
        function updateLastFrame(O, frame_num)
            O.last_seen = frame_num;
        end
    end    
end