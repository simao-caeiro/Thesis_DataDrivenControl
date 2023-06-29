function ts = compute_settling_time(signal, ref, percent, Ts)

% percent = percent/100;
aux = 0;
ts_sample = 0;
ts = 0;

for i=1:length(signal)
    diff = abs(signal(i) - ref);
    diff_p = diff/ref*100;
    if(diff_p > percent && aux == 1)
        aux = 0;
        ts_sample = 0;
    elseif(diff_p<= percent && aux == 0)
        ts_sample = i;
        aux = 1;
    end
end

if ts_sample == 0
    ts_sample = length(signal);
end

ts = ts_sample*Ts;

end