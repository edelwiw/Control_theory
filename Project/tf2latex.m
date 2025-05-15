function tf2latex(sys)
    [ny, nu] = size(sys);
    latex_str = ['\begin{bmatrix}', newline];
    for i = 1:ny
        for j = 1:nu
            [num, den] = tfdata(sys(i,j), 'v');
            if all(num == 0)
                element = '0';
            else
                num_str = polynom2tex(num, 's', 4);
                den_str = polynom2tex(den, 's', 4);
                element = sprintf('\\frac{%s}{%s}', num_str, den_str);
            end
            latex_str = [latex_str, element];
            if j < nu
                latex_str = [latex_str, ' & '];
            end
        end
        if i < ny
            latex_str = [latex_str, ' \\ ', newline];
        end
    end
    latex_str = [latex_str, newline, '\end{bmatrix}'];
    disp(latex_str);
end
