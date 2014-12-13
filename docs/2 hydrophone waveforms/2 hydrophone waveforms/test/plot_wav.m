function plot_wav(filename1, filename2)
	
mic1 = wavread(filename1);	%"mic1_med_gain_aligned.wav"
mic2 = wavread(filename2);	%"mic2_med_gain_aligned.wav"

plot(mic1, 'k');
hold on;
plot(mic2, 'b');

end
