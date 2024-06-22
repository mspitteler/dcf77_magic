% Plot after executing `cat /dev/ttyACM0 > /tmp/dcf77.csv'.

dcf77_tab = readtable('/tmp/dcf77.csv');
dcf77 = table2array(dcf77_tab(:, 1));
% dcf77_avg = dcf77(:, 2);
dcf77 = dcf77(:, 1);

fs = 0.5e6 / 5000;

dcf77_fft = abs(fft((dcf77 - mean(dcf77))/4096));
figure();
plot(0:1/length(dcf77_fft)*fs:fs-1/length(dcf77_fft), 20*log10(dcf77_fft));

kernel = [zeros(1, round(fs*0.4))+7/4 zeros(1, round(fs*0.1))+3/4 zeros(1, round(fs*0.1)) zeros(1, round(fs*0.4))+7/4];
figure();
plot(kernel);
offs=0.71*fs;
dcf77 = dcf77(end-13*fs+offs:end-3*fs+offs);
% dcf77_avg = dcf77_avg(end-9*fs:end);
figure();
hold('on');
dcf77_conv = conv(dcf77, kernel);
dcf77_conv_wrapped = zeros(1, 100);
for i = 1:100
    dcf77_conv_wrapped(i) = sum(dcf77_conv(i:100:end));
end
plot(0:1/fs:(length(dcf77)-1)/fs, dcf77);
% plot(0:1/fs:(length(dcf77_avg)-1)/fs, dcf77_avg);
hold('off');

figure();
hold('on');
interpolated = spline(1:length(dcf77_conv_wrapped), dcf77_conv_wrapped, 1:1/20:101-1/20);
plot(0:1/fs:(length(dcf77_conv_wrapped)-1)/fs, dcf77_conv_wrapped);
dcf77_cconv = cconv(dcf77, kernel, 100);
plot(0:1/fs:(length(dcf77_cconv)-1)/fs, dcf77_cconv);
plot(0:1/2000:(length(interpolated)-1)/2000, interpolated);

[a, b] = max(dcf77_conv_wrapped);
b
y = [dcf77_conv_wrapped(mod(b-1-1+100, 100)+1) dcf77_conv_wrapped(b) dcf77_conv_wrapped(mod(b+1-1, 100)+1)];
k = 2;
[a2, b2] = max(interpolated);
b2 = b2/20-1/20+1;
b2

lagrange_interpolated = zeros(1, length(0:0.05:k));
for x = 0:0.05:k
    L = zeros(1, k+1)+1;
    sum_ = 0;
    for j = 0:k
        for m = 0:k
            if m ~= j
                L(j+1) = L(j+1) * (x - m) / (j - m);
            end
        end
        sum_ = sum_ + y(j+1)*L(j+1);
    end
    lagrange_interpolated(int32(x/0.05+1)) = sum_;
end
plot(((b-1:0.05:b+1)-1)/fs, lagrange_interpolated);

[a3, b3] = max(lagrange_interpolated);
b3 = b3/20-1/20+b-1;
b3
hold('off');