function e_track = compute_tracking_error(signal, ref, n_samples)

signal = signal(end-n_samples:end);
avg_signal = mean(signal);

e_track = abs(avg_signal - ref);

end