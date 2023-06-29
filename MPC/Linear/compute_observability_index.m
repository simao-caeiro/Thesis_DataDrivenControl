function observability_index = compute_observability_index(C, A)

n = size(A, 1);

observability_matrix = C;
observability_index = 1;

while rank(observability_matrix) < n
    observability_index = observability_index + 1;
    observability_matrix = [C; observability_matrix * A];
    if observability_index > n
        fprintf('WARNING: (A,C) is unobservable\n\n');
        observability_index = inf;
        break;
    end
end

end

