function dx = flow(x,wc, Ac, Bwc)
x = x(1:end-1);
dx = [Ac*x + Bwc*wc;1];
end