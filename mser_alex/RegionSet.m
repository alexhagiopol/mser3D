classdef RegionSet
    properties
            %{
            Regions Matrix Format:
            MSER number
            ellipse param 1
            ellipse param 2
            ellipse param 3
            ellipse param 4
            ellipse param 5
            color
            votes
            %}
        RegionMatrices = [];
    end
    
    methods
        function addRegion(RS, regionMatrix)
            RS.RegionMatrices = [RS.RegionMatrices, regionMatrix];
        end
        function ellipsesScores = compareRegionEllipsescompareRegionEllipses(newEllipses) 
            
        end
    
        function getRegion
    end
end