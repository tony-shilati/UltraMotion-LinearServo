close all, clear, clc

% Load data from a CSV file
filename = "Undamped_Oscillator_ADS1220_0.1_to_40Hz_5kgLC_20251215_133907.csv";
path = "outputs/" + filename;
data = readmatrix(path); % Replace 'your_file.csv' with your actual file name

ticks_font_size = 20; % Font size of the axes
labels_font_size = 20; % Font size of the axes


%% Load the data - GS

%Command
cmd_signal = rmmissing(data(:, 1)); cmd_signal = cmd_signal - mean(cmd_signal);
cmd_time = rmmissing(data(:, 2));
%cmd_signal=tukeywin(length(cmd_signal), 0.25).*cmd_signal;

% Load Cell
tel_signal = rmmissing(data(2:end, 3)); tel_signal = tel_signal - mean(tel_signal);
tel_time = rmmissing(data(2:end, 5));
tel_signal = lowpass(tel_signal, 50, 1/mean(diff(tel_time)));
% tel_signal = highpass(tel_signal, 7, 1/mean(diff(tel_time)));

% Encoder
enc_signal = rmmissing(data(2:end, 4));
enc_time = rmmissing(data(2:end, 5));
% enc_signal = lowpass(enc_signal, 30, 1/mean(diff(enc_time)));

%% Load the data - GTE

% % Command
% cmd_signal = rmmissing(data(:, 1)); cmd_signal = cmd_signal - mean(cmd_signal);
% cmd_time = rmmissing(data(:, 2));
% %cmd_signal=tukeywin(length(cmd_signal), 0.25).*cmd_signal;
% 
% % Load Cell
% tel_signal = rmmissing(data(:, 3)); tel_signal = tel_signal - mean(tel_signal);
% tel_time = rmmissing(data(:, 4));
% %tel_signal = lowpass(tel_signal, 30, 1/mean(diff(tel_time)));
% 
% % Encoder
% enc_signal = rmmissing(data(:, 5));
% enc_time = rmmissing(data(:, 6));

%% Plot the sampling distribution
cmd_sample_dist = diff(cmd_time);
tel_sample_dist = diff(tel_time);
enc_sample_dist = diff(enc_time);

figure(1)
subplot(3,1,1)
histogram(cmd_sample_dist * 1000, 10)
ax = gca;ax.FontSize = ticks_font_size;
title("Command sample period distribution")
ylabel("Num samples")

subplot(3,1,2)
histogram(tel_sample_dist * 1000, 10)
ax = gca;ax.FontSize = ticks_font_size;
title("Load Cell sample period distribution")
ylabel("Num samples")

subplot(3,1,3)
histogram(enc_sample_dist * 1000, 10)
ax = gca;ax.FontSize = ticks_font_size;
title("Encoder sample period distribution")
ylabel("Num samples")
xlabel("Sample Period (ms)")



%% Plot the three signals together

figure(2)
% yyaxis left
plot(cmd_time, cmd_signal, '-','Color', [0, 0.4470, 0.7410], "LineWidth", 2), hold on
plot(enc_time, enc_signal, '-', 'Color', [0, 0.5, 0], "LineWidth", 2)
ax = gca; ax.FontSize = ticks_font_size;
ylabel("Encoder Position (mm)", FontSize=labels_font_size)

yyaxis right
plot(tel_time, tel_signal, '-', 'Color', [0.8500, 0.3250, 0.0980], "LineWidth", 1.5)
ylabel("Force (N)", FontSize=labels_font_size)
% ylim([-50, 50])

grid on
ax = gca;

legend("Commanded", "Linear Encoder", "Load Cell", FontSize=labels_font_size)
xlabel("Time (s)", FontSize=labels_font_size)

%% Plot the three signals seperate

figure(3)
subplot(3,1,1)
plot(cmd_time, cmd_signal, '-','Color', [0.8500, 0.3250, 0.0980], "LineWidth", 1.5), hold on
ax = gca; ax.FontSize = ticks_font_size;
ylabel("Commanded Pos (mm)", FontSize=labels_font_size)
xlim([min(cmd_time), max(cmd_time)])
grid on

subplot(3,1,2)
plot(enc_time, enc_signal, '-', 'Color', [0, 0.5, 0], "LineWidth", 1.5)
ax = gca; ax.FontSize = ticks_font_size;
ylabel("Encoder Pos. (mm)", FontSize=labels_font_size)
xlim([min(enc_time), max(enc_time)])
grid on

subplot(3,1,3)
plot(tel_time, tel_signal, '-', 'Color', [0, 0.4470, 0.7410], "LineWidth", 1.5)
ax = gca; ax.FontSize = ticks_font_size;
ylabel("Force (N)", FontSize=labels_font_size)
xlim([min(tel_time), max(tel_time)])
grid on

xlabel("Time (s)", FontSize=labels_font_size)



%% Plot the fft of all three together

[P1_cmd, P1_cmd_phase, f_cmd, fs_cmd] = normalized_fft(cmd_signal, cmd_time);
[P1_tel, P1_tel_phase, f_tel, fs_tel] = normalized_fft(tel_signal, tel_time);
[P1_enc, P1_enc_phase, f_enc, fs_enc] = normalized_fft(enc_signal, enc_time);

P1_cmd = abs(P1_cmd);
P1_tel = abs(P1_tel);
P1_enc = abs(P1_enc);

ws_cmd = 2*pi*fs_cmd;
ws_tel = 2*pi*fs_tel;
ws_enc = 2*pi*fs_enc;

xlims = [0.1 40];

z_cmd = fftshift(P1_cmd);

figure(4)
subplot(2,1,1)
loglog(f_cmd, P1_cmd, "LineWidth", 1.5), hold on
loglog(f_enc, P1_enc, "LineWidth", 1.5)
loglog(f_tel, P1_tel, "LineWidth", 1.5)
ax = gca; ax.FontSize=ticks_font_size;
ylabel("Amplitude", FontSize=labels_font_size)
legend("Commanded", "Linear Encoder", "Load Cell")

grid on
xlim(xlims)
% ylim([10^-6 5*10^-1])

subplot(2,1,2)
semilogx(f_cmd, P1_cmd_phase, "LineWidth", 1.5), hold on
semilogx(f_enc, P1_enc_phase, "LineWidth", 1.5), hold on
semilogx(f_tel, P1_tel_phase, "LineWidth", 1.5), hold on
xlim(xlims)
grid on

xlabel("Frequency (Hz)", FontSize=labels_font_size)


%% Command spectrogram

% Plot the spectrogram of the signal
figure(5)
subplot(3,1,1)
cs_len = length(cmd_signal);
s = spectrogram(cmd_signal, ceil(cs_len/29), ...
    ceil(cs_len/30), ceil(cs_len/29), fs_cmd, 'yaxis');
spectrogram(cmd_signal, ceil(cs_len/29), ceil(cs_len/30), ceil(cs_len/29), fs_cmd, 'yaxis');

title('Spectrogram of Command');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, 50])


%% Load Cell Spectrogram

% Plot the spectrogram of the signal
subplot(3,1,2)
ts_len = length(tel_signal);
s = spectrogram(tel_signal, ceil(ts_len/29), ...
    ceil(ts_len/30), ceil(ts_len/29), fs_tel, 'yaxis');
spectrogram(tel_signal, ceil(ts_len/29), ceil(ts_len/30), ceil(ts_len/29), fs_tel, 'yaxis');

title('Spectrogram of Load Cell');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, 50])




%% Encoder spectrogram

% Plot the spectrogram of the signal
subplot(3,1,3)
es_len = length(enc_signal);
s = spectrogram(enc_signal, ceil(es_len/29), ...
    ceil(es_len/30), ceil(es_len/29), 2*pi*fs_enc, 'yaxis');

spectrogram(enc_signal, ceil(es_len/29), ceil(es_len/30), ceil(es_len/29), fs_enc, 'yaxis');

title('Spectrogram of Encoder');
xlabel('Time (min)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, 70])



%% Define functions
function [P1_mag, P1_phase, f, Fs] = normalized_fft(signal, time)
    Fs = 1 / mean(diff(time)); % Sampling frequency
    L = length(signal); % Length of the signal
    Y = fft(signal); % Compute the FFT
    P2 = Y/L; % Two-sided spectrum
    P1 = P2(1:L/2+1); % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1); % Correct the amplitude
    P1_phase = unwrap(angle(P1))*(180/pi);
    P1_mag = abs(P1); % Magnitude of the FFT
    f = Fs*(0:(L/2))/L; % Frequency vector

end