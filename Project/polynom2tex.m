function s = polynom2tex(coeffs, var, prec)
    idx = find(coeffs ~= 0, 1);
    if isempty(idx)
        s = '0';
        return;
    end
    coeffs = coeffs(idx:end);
    s = '';

    for n = 1:length(coeffs)
        coef = coeffs(n);
        power = length(coeffs) - n; 

        if coef == 0
            continue;
        end

        if coef > 0 
            s = [s, ' + ', sprintf('%.*f', prec, coef)];
        elseif coef < 0
            s = [s, ' - ', sprintf('%.*f', prec, abs(coef))];
        end

        if power > 0
            s = [s, var];
            if power > 1
                s = [s, '^', num2str(power)];
            end
        end

    end