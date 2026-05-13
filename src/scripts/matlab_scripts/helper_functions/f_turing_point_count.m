function turing_point_count = f_turing_point_count(data,time,figure_number)
turing_point_count = 0;
turing_point_list = [];
data_length = length(data);
d_data = [0;diff(data)];

i = 1;
while i <data_length
    current_d_data = d_data(i);
    if abs(current_d_data)<0.0001
        i = i + 1;
        continue
    end
    for j = i+1:data_length-10
        if current_d_data * d_data(j)< 0
            turing_point_list = [turing_point_list;j-1];
            break;
        end
    end
    i = j+1;
end
turing_point_count = length(turing_point_list);
%%
% figure(figure_number)
% plot(time,data);hold on
% plot(time,data,"*","MarkerIndices",turing_point_list,"MarkerEdgeColor",'k');hold on 

end