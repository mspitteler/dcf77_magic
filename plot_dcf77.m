% Plot after executing `cat /dev/ttyACM0 > /tmp/dcf77.csv'.

dcf77_tab = readtable('/tmp/dcf77.csv');
dcf77 = table2array(dcf77_tab(:, 1));
% dcf77_bin = dcf77(:, 2) * 5000;
dcf77 = dcf77(:, 1);

fs = 0.5e6 / 5000;

dcf77_fft = abs(fft((dcf77 - mean(dcf77))/4096));
figure();
plot(0:1/length(dcf77_fft)*fs:fs-1/length(dcf77_fft), 20*log10(dcf77_fft));

kernel = [zeros(1, round(fs*0.4))+7/4 zeros(1, round(fs*0.1))+3/4 zeros(1, round(fs*0.1)) zeros(1, round(fs*0.4))+7/4];
figure();
plot(kernel);

offs=0.2*fs;
dcf77 = dcf77(1+offs:10*fs+offs);

% dcf77_bin = dcf77_bin(end-(fs*10):end);
figure();
hold('on');
dcf77_wrapped = zeros(1, 100);
tmp=45;
for i = 1+tmp:length(dcf77)+tmp
    dcf77_wrapped(mod(i-1,100)+1) = dcf77_wrapped(mod(i-1,100)+1) + dcf77(mod(i-1,length(dcf77))+1);
end

dcf77_wrapped_conv = conv(dcf77_wrapped, kernel);
dcf77_conv = conv(dcf77, kernel);
dcf77_conv_wrapped = zeros(1, 100);
dcf77_wrapped_conv_wrapped = zeros(1, 100);
for i = 1:100
    dcf77_wrapped_conv_wrapped(i) = sum(dcf77_wrapped_conv(i:100:end));
    dcf77_conv_wrapped(i) = sum(dcf77_conv(i:100:end));
end
plot(0:1/fs:(length(dcf77)-1)/fs, dcf77);
% plot(0:1/fs:(length(dcf77_bin)-1)/fs, dcf77_bin);
hold('off');

figure();
hold('on');
% plot(0:1/fs:(length(dcf77_wrapped)-1)/fs, dcf77_wrapped);
plot(0:1/fs:(length(dcf77_conv_wrapped)-1)/fs, dcf77_conv_wrapped);
plot(0:1/fs:(length(dcf77_wrapped_conv_wrapped)-1)/fs, dcf77_wrapped_conv_wrapped);
hold('off');
