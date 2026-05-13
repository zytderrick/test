%% calculate the steer wheel angle according to the reference kappa
%%
ref_kappa_list = [0.02;0.03;0.05;0.08;0.1;0.15;0.2;0.22;0.25];
ref_kappa_list_length = length(ref_kappa_list);
SWA_list = zeros(ref_kappa_list_length,1);
for i = 1:ref_kappa_list_length
    SWA_list(i) = f_kappa2SWAdeg(ref_kappa_list(i));
end
kappa2SWA_table = table(ref_kappa_list,SWA_list,'VariableNames',{'kappa','SWA(deg)'});
disp(kappa2SWA_table);
