fs = 0.5e6;

h_len = 4096;

pass_f = 77.5e3;

w1 = 2 * (pass_f / fs - 1/h_len);
w2 = 2 * (pass_f / fs + 1/h_len);

t = -(h_len - 1) / 2:1:(h_len - 1) / 2;

a = w1*sinc(w1*t);
b = w2*sinc(w2*t);

c = a - b;

plot(c);
fvtool(c);