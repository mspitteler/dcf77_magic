drift = [(0:(1/32768):(0.5 - (1/32768))) (0.5:(1/16384):(1.5 - (1/16384))) zeros(1, 32768) + 1.5 zeros(1, 65536)] + 2;
adc = round(([wgn(1, 65536+32768, 0) wgn(1, 32768, -60)] + drift) * 1024);

hold('on');
plot(adc);
plot(drift * 1024);

avg = 0;
averages = zeros(1, 65536*2);

for i=1:length(adc)
    avg = floor((avg * 2047 + adc(i) * 512 + 1023) / 2048);
    averages(i) = avg / 512;
end

plot(averages);

hold('off');