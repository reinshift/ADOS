function h = H(x)
    h = 0.5 + 16*x/(2*log(exp(-16*x)+exp(16*x)));
end