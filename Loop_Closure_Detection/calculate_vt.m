%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Owen Winship
% Date : 04/27/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vt] = calculate_vt(input_descriptors, word_tree)
    word_vector = word_tree.word_vector(:,1:(end-1));
    size_word = size(word_vector);
    occurence_vector = zeros(size_word(1),1);
    weight_vector = word_tree.word_vector(:,end);
    sizeinput = size(input_descriptors);
    for i = 1:sizeinput(1)
        [word, index, weight] = traverse_word_tree(input_descriptors(i,:),word_tree); 
        occurence_vector(index) = occurence_vector(index)+1;
    end
    vt = occurence_vector.*weight_vector;
        
end