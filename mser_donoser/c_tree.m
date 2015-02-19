%Algorithms from Donoser07thesis
classdef c_tree < handle
    properties
        nodes; %make this into a 2xn cell array
        %{
        1       2       3       4
        {id1}   {id2}   {id3}   {id4}
        {node1} {node2} {node3} {node4}
        %}
    end
    methods
        function CT = c_tree  
            CT.nodes = {};
        end
        
        %Helper function: returns the cell array index of a node given its
        %ID
        function index = get_index(CT, id)
            index = 1;
            for i = CT.nodes{1,:}
                if i == id
                    break;
                end
                index = index + 1;
            end
        end
        
        %Algorithm 4: adds new node
        function make_node(CT, new_id, new_level)
            global nodes_made;
            new_node = mser_node(new_id, new_level);
            CT.nodes = [CT.nodes, {new_id; new_node}];
            %disp('made a node');
            nodes_made = nodes_made + 1;            
        end      
        
        %Algorithm 5: makes the child connection
        function add_child(CT, parent_id, child_id)
            CT.nodes{1,CT.get_index(parent_id)}.children_ids = [CT.nodes{1,CT.get_index(parent_id)}.children_ids, child_id];
        end
        
        %Algorithm 6: returns a node given an id
        function node = get_node(CT, query_id)
            node = CT.nodes{2,CT.get_index(query_id)};
        end
        
        %Helper function: deletes a node. 
        function remove_node(CT, id)
            CT.nodes(:,CT.get_index(id)) = [];
        end
        
        %Algorithm 7: merges 2 nodes into 1
        %id1 or id2 must be equal to id3
        function merge_nodes(CT, id1, id2, id3)
            node1 = CT.get_node(id1);
            node2 = CT.get_node(id2);
            if id2 == id3
                for c = node1.children_ids
                    CT.add_child(id2,c);
                end
                CT.remove_node(id1);
            else
                for c = node2.children_ids
                    CT.add_child(id1,c);
                end
                CT.remove_node(id2);                
            end
        end
    end
end