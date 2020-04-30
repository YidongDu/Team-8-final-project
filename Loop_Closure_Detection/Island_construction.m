%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author : Roger (Aohan) Mei
% Date : 04/20/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function temp_group = Island_construction(candidates)
    saver = {}
    temp_saver = {}
    for i  = 1 : length(candidates)
        if i == 1
            cur_slot_num = candidates{i}.Slot_number;
            temp_saver{length(temp_saver)+1} = candidates{i};
            prev_slot_num = cur_slot_num;
        else
            cur_slot_num = candidates{i}.Slot_number;
            delta_slot_num = cur_slot_num - prev_slot_num;
            if delta_slot_num <= 4
                % The candidates belong to the same island
                temp_saver{length(temp_saver)+1} = candidates{i};
                prev_slot_num = cur_slot_num;
            else
                % Save the previous island and construct a new island
                saver{length(saver)+1} = temp_saver;
                temp_saver = {};
                temp_saver{length(temp_saver)+1} = candidates{i};
                prev_slot_num = cur_slot_num;
            end
        end
    end
    temp_group = saver;
end