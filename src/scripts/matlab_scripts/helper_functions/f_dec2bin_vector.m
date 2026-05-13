%% dec2bin return vector_32
function [binary_vector_32] = f_dec2bin_vector(code)
    binary_vector_32 = zeros(32,1);
    if code < -2^31 || code > 2^31-1
        return
    end
    if code < 0
        binary_vector_32(32) = 1;
        code = abs(code);
    end
    for i = 1:31
        binary_vector_32(i) = mod(code,2);
        code = floor(code/2);
    end
end