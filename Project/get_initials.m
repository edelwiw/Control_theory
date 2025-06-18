function [M, m, l, g] = get_initials()
    rng(30, "philox");
    M = randi([100000 1000000]) / 1000 / sqrt(2);
    m = randi([1000 10000]) / 1000 * sqrt(3);
    l = randi([100 1000]) / 100 / sqrt(5);
    l = l / 2;
    g = 9.81;   
end