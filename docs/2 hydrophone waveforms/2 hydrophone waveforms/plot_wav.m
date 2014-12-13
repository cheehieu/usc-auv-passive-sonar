function plot_wav(filename1, filename2)
	
mic1 = wavread(filename1);
mic2 = wavread(filename2);

%subplot(2,2,1);
%plot(mic1, 'k');
%subplot(2,2,2);
%plot(mic2, 'b');
%subplot(2,2,3:4);
plot(mic1, 'k');
hold on;
plot(mic2, 'b');

end