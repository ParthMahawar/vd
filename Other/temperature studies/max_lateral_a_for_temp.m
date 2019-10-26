load('C:\Users\scruf\Dropbox\FSAE\vd\Magic Formula\TTC Documentation\Round 8 (16 in Hoosiers)\RawData_Cornering_Matlab_USCS_10inch_Round8\A1965raw15.mat')
raw_data = [TSTC, FY];
%sort all datapoints by temperature
raw_data = transpose(raw_data);
raw_data = sortrows(raw_data.',1).';

step = 3;
max_temp_for_current_set = raw_data(1,1);
i = 1;

matrix_of_max_lat_forces = [];

while i < size(raw_data,2)
    temp_vector = 0;
    while raw_data(1,i) < max_temp_for_current_set && i < size(raw_data,2)
        temp_vector = [temp_vector, abs(raw_data(2,i))];
        i = i + 1;
    end
    
    %90th percentile force for given temperature, average temp for set
    new_entry = [prctile(temp_vector,90), max_temp_for_current_set-step/2];
    
    matrix_of_max_lat_forces = [matrix_of_max_lat_forces, transpose(new_entry)];
    
    max_temp_for_current_set = max_temp_for_current_set + step;
end

plot(matrix_of_max_lat_forces(2,:), matrix_of_max_lat_forces(1,:));
hold off;