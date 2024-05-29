% Plot after executing `cat /dev/ttyACM0 > /tmp/dcf77.csv'.

dcf77_tab = readtable('/tmp/dcf77.csv');
dcf77 = table2array(dcf77_tab(:, 1:2));
dcf77_avg = dcf77(:, 2);
dcf77 = dcf77(:, 1);

fs = 0.5e6 / 5000;

dcf77_fft = abs(fft((dcf77 - mean(dcf77))/4096));
figure();
plot(0:1/length(dcf77_fft)*fs:fs-1/length(dcf77_fft), 20*log10(dcf77_fft));

% kernel = [zeros(1, round(fs*0.4))+7/4 zeros(1, round(fs*0.1))+3/4 zeros(1, round(fs*0.1)) zeros(1, round(fs*0.4))+7/4];
% figure();
% plot(kernel);
% offs=0.5*fs;
dcf77 = dcf77(end-10*fs:end-1*fs);
dcf77_avg = dcf77_avg(end-9*fs:end);
figure();
hold('on');
% dcf77_conv = conv(dcf77, kernel);
% dcf77_conv_wrapped = zeros(1, 100);
% for i = 1:100
%     dcf77_conv_wrapped(i) = sum(dcf77_conv(i:100:end));
% end
plot(0:1/fs:(length(dcf77)-1)/fs, dcf77);
plot(0:1/fs:(length(dcf77_avg)-1)/fs, dcf77_avg);
hold('off');

% figure();
% plot(0:1/fs:(length(dcf77_conv_wrapped)-1)/fs, dcf77_conv_wrapped);
