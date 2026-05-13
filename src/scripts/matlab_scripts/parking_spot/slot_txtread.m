%% read txt to save slot position
files_list = dir;
%%
slot_list = [];
for i = 1:length(files_list)
    cur_file_name = files_list(i).name;
    if length(cur_file_name)>4 && cur_file_name(1)>='a' && cur_file_name(1)<='z' ...
            && cur_file_name(2)>='0' && cur_file_name(2)<='9' &&cur_file_name(end-2:end) == "txt"
        slot_position = readmatrix(cur_file_name);
        slot_list = [slot_list;slot_position];
    end
end

clearvars -except slot_list
save('../saved_slot_pos')