%Name:        Isaac Nakone, Alex Welke
%Date:        09/07/2024 - 29/07/2024
%Description: This is a generic tree class.

classdef tree < handle &...              %Inherits handle properties.
                matlab.mixin.SetGet &... %Inherits setters and getters.
                matlab.mixin.Copyable    %Is copyable. 

    properties 
        nodes               = cell(1);
        connectivity_matrix = [];

        %Stores the number of stages in the tree structure
        num_layers;

        %Stores the indices of the nodes in each layer
        tree_matrix = {[]};
    end

    methods


        function obj = tree(seed_str)
            obj.nodes               =  {struct('name', seed_str, 'node_layer', 1)};  
            obj.connectivity_matrix = 0; 
            obj.num_layers = 1;
            obj.tree_matrix{1} = 1;
        end


        function node_index = search_name_in_nodes(obj, name)
            node_index = 0;
            for tree_index = 1:size(obj.connectivity_matrix,1)
                if strcmp(obj.nodes{tree_index}.name, name)
                    node_index = tree_index;
                    break;
                end
            end        
            if (node_index == 0)
                disp("Error, parent name not found in aircraft tree!");
            end
        end


        function add_child_to_parent(obj,child_name,parent_name)
            parent_index = search_name_in_nodes(obj, parent_name);
            child = struct('name', child_name, 'node_layer', obj.nodes{parent_index}.node_layer+1);
            obj.nodes{end+1} = child;
            

            %Checks if the number of layers needs to be updated
            if obj.nodes{end}.node_layer > obj.num_layers
                obj.num_layers = obj.nodes{end}.node_layer;
                obj.tree_matrix = [obj.tree_matrix; {[]}];
            end

            num_nodes = size(obj.connectivity_matrix,1);

            obj.connectivity_matrix = [obj.connectivity_matrix;
                                            zeros(1,num_nodes)];
            obj.connectivity_matrix = [obj.connectivity_matrix,...
                                            zeros(num_nodes+1,1)];
            obj.connectivity_matrix(parent_index,num_nodes+1) = 1;
            %obj.connectivity_matrix(num_nodes+1,parent_index) = 2;

            obj.tree_matrix{obj.nodes{end}.node_layer} = ...
                [obj.tree_matrix{obj.nodes{end}.node_layer}, num_nodes+1];
        end
        
        %Finds the indices of all children of a given parent
        function child_indices = find_children(obj, parent_index)
            child_indices = [];

            %Stores indices of parents in previous layer
            previous_parent_indices = [];

            %Finds immediate children of parent
            for column = parent_index:size(obj.nodes, 2)
                %Checks whether a connection exists
                if obj.connectivity_matrix(parent_index, column) == 1
                    child_indices = [child_indices, column];
                    previous_parent_indices = [previous_parent_indices, column];
                end
            end

            %Loops through every layer underneath the given parent
            for layer = obj.nodes{parent_index}.node_layer:obj.num_layers

                %Loops through children of previous parents and find their
                %children
                previous_parent_indices_temp = previous_parent_indices;
                previous_parent_indices = [];
                for current_parent = previous_parent_indices_temp
                    for column = current_parent:size(obj.nodes, 2)
                        %Checks whether a connection exists
                        if obj.connectivity_matrix(current_parent, column) == 1
                            child_indices = [child_indices, column];
                            previous_parent_indices = [previous_parent_indices, column];
                        end
                    end

                end
            end

        end
        
        function parent_index = find_parent(obj, child_index)
            parent_index = find(obj.connectivity_matrix(:, child_index));
            
        end

        function node_name_list = gen_node_name_list(obj)
            num_nodes = size(obj.connectivity_matrix,1);
            node_name_list = [];
            
            for tree_index = 1:num_nodes
                node_name_list = [node_name_list; obj.nodes{tree_index}.name];                
            end  
        end
    

        function plot_tree_graph(obj)
            node_name_list = gen_node_name_list(obj);
            G              = digraph(obj.connectivity_matrix);
            h              = plot(G);
            labelnode(h,1:size(obj.connectivity_matrix,1),node_name_list);
            h.set('Interpreter', 'none')
        end

    end


end