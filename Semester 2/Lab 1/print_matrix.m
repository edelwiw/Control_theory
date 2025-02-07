function print_matrix(A, precision)
    [m, n] = size(A);
    fprintf("\\begin{bmatrix}\n");
    for i = 1:m
        for j = 1:n
            if j == n
                fprintf('%.*f', precision, A(i, j));
                break;
            end
            fprintf('%.*f & ', precision, A(i, j));
        end
        fprintf(' \\\\ \n');
    end
    fprintf("\\end{bmatrix}\n");
end