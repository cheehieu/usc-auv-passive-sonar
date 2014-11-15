data1 = load("1_adc_samples.txt");
data2 = load("2_adc_samples.txt");
data3 = load("3_adc_samples.txt");
figure
hold on
%subplot(2,3,1:3);
plot(data1)
%subplot(2,3,1:3);
plot(data2, 'r')
%subplot(2,3,1:3);
plot(data3, 'y')

%subplot(2,3,4);
%plot(data1)
%subplot(2,3,5);
%plot(data2, 'r')
%subplot(2,3,6);
%plot(data3, 'y')

