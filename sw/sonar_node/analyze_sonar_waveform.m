function [ analysis ] = analyze_sonar_waveform( filename, plots_on )
%analyze_sonar_waveform Provides an analysis of the inputted waveform
%(assumes waveform contains only one channel).
%   Return analysis is structured as follows:
%       analysis.waveform    : the time domain waveform
%       analysis.fs          : the sample rate
%       analysis.bd          : the bit-depth
%       analysis.dt          : the time spacing between samples
%       analysis.times       : the time sequence for the waveform
%       analysis.fft         : the Fourier transform of the waveform
%       analysis.frequencies : the frequencies for the Fourier sequence
%       analysis.global_snr  : the SNR calculated using the entire spectrum
%       analysis.local_snr   : the SNR calculated using only the spectrum
%                               from f_start to f_stop
%
%   If plots_on = true, plots for the time and frequency domains will be
%   shown, otherwise, they will be turned off.



%% Initialize
[analysis.waveform, analysis.fs, analysis.bd] = wavread(filename);
analysis.dt = 1/analysis.fs;
analysis.times = 0:analysis.dt:length(analysis.waveform-1)/analysis.fs;
analysis.times = analysis.times(1:length(analysis.times)-1);
analysis.times = analysis.times';

%% Frequency domain
NFFT = 2^nextpow2(length(analysis.waveform));
analysis.fft = fft(analysis.waveform,NFFT)/length(analysis.waveform);
analysis.frequencies = analysis.fs/2*linspace(0,1,NFFT/2+1);
analysis.frequencies = analysis.frequencies';

%% Calculate SNRs
f_start = 29600;
f_stop = 30400;
f_delta = 100;
peak_signal = abs(max(analysis.fft(round((length(analysis.frequencies)/(analysis.fs/2))*f_start):round((length(analysis.frequencies)/(analysis.fs/2))*f_stop))));
local_peak_left = abs(mean(analysis.fft(round((length(analysis.frequencies)/(analysis.fs/2))*(f_start-f_delta)):round((length(analysis.frequencies)/(analysis.fs/2))*(f_start+f_delta)))));
local_peak_right = abs(mean(analysis.fft(round((length(analysis.frequencies)/(analysis.fs/2))*(f_stop-f_delta)):round((length(analysis.frequencies)/(analysis.fs/2))*(f_stop+f_delta)))));
local_peak = (local_peak_left + local_peak_right)/2;
global_peak = abs(max(analysis.fft));

analysis.global_snr = 10*log10(peak_signal/global_peak);
analysis.local_snr = 10*log10(peak_signal/local_peak);

%% Plot
if (plots_on == true)
    subplot(2,1,1);
    plot(analysis.times, analysis.waveform);
    xlabel('Time (seconds)');
    subplot(2,1,2);
    plot(analysis.frequencies, 2*abs(analysis.fft(1:NFFT/2+1)));
    xlabel('Frequency (Hertz)');
end

end
