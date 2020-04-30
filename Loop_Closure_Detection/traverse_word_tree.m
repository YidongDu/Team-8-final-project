function [word, index, weight] = traverse_word_tree(input,node)
    distance = [];
    i = 1;
    while(1)
        childname = strcat('child',num2str(i));
        if(isfield(node,childname))
            distance = [distance,pdist2(input,node.(childname).centroid)];
        else
            break
        end
        i = i+1;  
    end
    if(isempty(distance))
        word = node.centroid;
        index = node.lf_index;
        weight = node.weight;
    else
        [minimum, minIndex] = min(distance);
        minchild = strcat('child',num2str(minIndex));
        [word, index, weight] = traverse_word_tree(input,node.(minchild));
        %index = [index,minIndex];
    end
    
        
end