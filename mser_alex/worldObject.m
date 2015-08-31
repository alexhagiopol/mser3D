classdef worldObject < handle
    properties
        msers; %vector containing all MSERs
        id; %object id = equal to frame.regionNum
        last_seen; %frame in which the object was last seen
        color; %[R,G,B]
        recent_i; %index of most recent mser
    end
    methods
        %initialize
        function WO = worldObject(first_mser,id_,frame_num,color_)
            WO.msers = [first_mser, WO.msers];
            WO.id = id_;
            WO.last_seen = frame_num;
            WO.color = color_;
            WO.recent_i = 1; %store index of most recent mser;
        end
        %get mser
        function mser = getLatestMSER(WO)
            mser = WO.msers(WO.recent_i);
        end
        %add mser
        function addMSER(WO, mser)
            WO.recent_i = WO.recent_i + 1;
            WO.msers = [WO.msers, mser];
        end
        %update most recent frame appearance
        function updateLastFrame(WO, frame_num)
            WO.last_seen = frame_num;
        end
    end    
end