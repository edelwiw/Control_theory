function print_matrix(A, precision)
    [m, n] = size(A);
    fprintf("\\begin{bmatrix}\n");
    for i = 1:m
        for j = 1:n

            if imag(A(i, j)) == 0
                fprintf('%.*f ', precision, A(i, j));
            else 
                if imag(A(i, j)) < 0
                    fprintf('%.*f - %.*fj ', precision, real(A(i, j)), precision, -imag(A(i, j)));
                else
                    fprintf('%.*f + %.*fj ', precision, real(A(i, j)), precision, imag(A(i, j)));
                end
            end
            
            if j < n
                fprintf(' & ');
             end
        end
        fprintf('\\\\ \n');
    end
    fprintf("\\end{bmatrix}\n");
end