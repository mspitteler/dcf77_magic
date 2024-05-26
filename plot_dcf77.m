% Plot after executing `cat /dev/ttyACM0 > /tmp/dcf77.csv'.

dcf77_tab = readtable('/tmp/dcf77.csv');
dcf77 = table2array(dcf77_tab);
dcf77_thresh = dcf77(:, 2);
dcf77 = dcf77(:, 1);

fs = 0.5e6 / 16384;

dcf77_fft = abs(fft((dcf77 - mean(dcf77))/4096));
figure();
plot(0:1/length(dcf77_fft)*fs:fs-1/length(dcf77_fft), 20*log10(dcf77_fft));

dcf77 = dcf77(end-(fs*10):end);
dcf77_thresh = dcf77_thresh(end-(fs*10):end);
figure();
hold('on');
plot(0:1/fs:(length(dcf77)-1)/fs, dcf77);
plot(0:1/fs:(length(dcf77_thresh)-1)/fs, dcf77_thresh);
hold('off');
