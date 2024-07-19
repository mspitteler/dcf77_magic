# Homodyne DCF77 receiver for the Raspberry Pi Pico

This library contains the code for a DCF77 receiver for the Raspberry Pi Pico, it requires very little external 
electronics.

A minimal design would just consist of a resistive divider from the 3.3V supply where the lower resistor 
is connected in parallel to a smoothing capacitor of let's say 100nF. The midpoint of this resistive divider would be 
connected to one side of a loopstick antenna with resonant capacitor. The other side is to be connected to pin 26 (or 
analog input 0) of the Raspberry Pi Pico. This design is probably not usable more than 500km away from the transmitter 
in Mainflingen.

A more complex design could include a simple 2 stage amplifier circuit for example to use a larger part of the ADC 
range which results in less quantization noise.

## How it works

At the heart of this receiver is a very high Q (around 15000) biquad digital bandpass filter which filters out 
everything but the desired 77.5kHz carrier. This works down to amazingly low input amplitudes. It is possible to decode 
the correct time (although it could take a large number of minutes) from an input signal to the ADC that has an 
amplitude of 200uV (as long as there is noise in other parts of the frequency spectrum with much higher amplitude, as 
the step size of the ADC is 800uV (since it is a 12-bit ADC).

The reason that the signal can still be recovered even though it is well below the noise floor of an ideal 12-bit ADC 
(to make things worse, the ENOB is actually only about 8.7 bit) is that we only want a very specific part of the 
frequency spectrum: 77.5kHz, and because the noise is more or less equally distributed over the frequency spectrum, we 
can still pick up the signal.

The downside of this digital oriented direct conversion approach is that we ofcourse also lose in bandwidth due to the 
high Q. With a Q of 15000 we have a bandwidth of about 5Hz, which is both high enough for deviations in the clock 
frequency causing a slightly different center frequency and high enough for decoding the carrier modulation but also 
low enough that we don't get too much noise.

## Error resilience of the time decoder

The demodulated signal (after envelope detection) can still have a large amount of noise on it. This is why a matched 
filter is used to detect phase of the (almost) periodic demodulated signal. On top of this the time decoder is also 
made as robust as possible without having to do smart tricks with what we know about time. The way it basically works 
now is that the BCD re-encoded/synthesized bits of the RTC are compared to the BCD bits of the received time every 
minute and if a specific bit (let's say the 10s bit of the minute for example) is wrong for more than 10 minutes, it is 
flipped (so we would flip the RTC from 10:20 to 10:30 in case of the example). This works surprisingly well (even for 
the fast changing minute bits) since the bits are in BCD encoding. This algorithm works very close to a 50% bit error 
rate, (I haven't really done measurements yet on how close we can get reliably) although in some cases (when it has a 
really bad error rate) it can take almost a day to obtain the correct time, and probability of losing time at some 
point is high ofcourse.

## Usage

I used this code to make a reference clock (compatible with the ELV DCF 7000 driver) for NTPsec to make my own stratum 
1 time server. It is still far from complete though. Correctly outputting the timezone and whether we have reliable 
sync has yet to be done.

I'm also planning to make a digital clock based on this receiver, but I haven't started on that yet.
