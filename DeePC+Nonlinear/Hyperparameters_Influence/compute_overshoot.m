function S = compute_overshoot(signal, ref)

max_value = max(signal);
diff = abs(max_value-ref);
S = diff/ref *100;

end