close all, clear, clc

% Load data from a CSV file
filename = "MoveMotorTelemetry_20251107_105828.csv";
path = "outputs/" + filename;
data = readmatrix(path); % Replace 'your_file.csv' with your actual file name

%% Load the data

% Command
cmd_signal = rmmissing(data(:, 1)); cmd_signal = cmd_signal - mean(cmd_signal);
cmd_time = rmmissing(data(:, 2));
%cmd_signal=tukeywin(length(cmd_signal), 0.25).*cmd_signal;

% Telemetry
tel_signal = rmmissing(data(:, 3)); tel_signal = tel_signal - mean(tel_signal);
tel_time = rmmissing(data(:, 4));

% Encoder
enc_signal = rmmissing(data(:, 5));
enc_time = rmmissing(data(:, 6));

%% Plot the sampling distribution
cmd_sample_dist = diff(cmd_time);
tel_sample_dist = diff(tel_time);
enc_sample_dist = diff(enc_time);

figure(1)
subplot(3,1,1)
histogram(cmd_sample_dist * 1000, 100)
title("Command sample period distribution")
ylabel("Num samples")

subplot(3,1,2)
histogram(tel_sample_dist * 1000, 100)
title("Telemetry sample period distribution")
ylabel("Num samples")

subplot(3,1,3)
histogram(enc_sample_dist * 1000, 100)
title("Encoder sample period distribution")
ylabel("Num samples")
xlabel("Sample Period (ms)")

%% Plot the three signals together

figure(2)
yyaxis left
plot(cmd_time, cmd_signal, "LineWidth", 1.5), hold on
plot(enc_time, enc_signal, '-r', "LineWidth", 1.5)
ylabel("Position (mm)")

yyaxis right
plot(tel_time, tel_signal, "LineWidth", 1.5)
ylabel("Force (N)")

legend("Commanded", "Linear Encoder", "Load Cell")
xlabel("Time (s)")


%% Plot the fft of all three together

[P1_cmd, f_cmd, fs_cmd] = normalized_fft(cmd_signal, cmd_time);
[P1_tel, f_tel, fs_tel] = normalized_fft(tel_signal, tel_time);
[P1_enc, f_enc, fs_enc] = normalized_fft(enc_signal, enc_time);

z_cmd = fftshift(P1_cmd);

figure(3)
subplot(2,1,1)
loglog(f_cmd, mag2db(P1_cmd), "LineWidth", 1.5), hold on
loglog(f_tel, mag2db(P1_tel), "LineWidth", 1.5)
loglog(f_enc, mag2db(P1_enc), "LineWidth", 1.5)
ylabel("Amplitude (dB)")
legend("Commanded", "Telemetry", "Linear Encoder")

grid on
xlim([1, max([0.5*fs_cmd, 0.5*fs_tel, 0.5*fs_enc])])

xlabel("Frequency (Hz)")


%% Command spectrogram

% Plot the spectrogram of the signal
figure(4)
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
ylim([0, fs_cmd/2])


%% Telemetry Spectrogram

% Plot the spectrogram of the signal
subplot(3,1,2)
ts_len = length(tel_signal);
s = spectrogram(tel_signal, ceil(ts_len/29), ...
    ceil(ts_len/30), ceil(ts_len/29), fs_tel, 'yaxis');
spectrogram(tel_signal, ceil(ts_len/29), ceil(ts_len/30), ceil(ts_len/29), fs_tel, 'yaxis');

title('Spectrogram of Telemetry');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, fs_tel/2])




%% Encoder spectrogram

% Plot the spectrogram of the signal
subplot(3,1,3)
es_len = length(enc_signal);
s = spectrogram(enc_signal, ceil(es_len/29), ...
    ceil(es_len/30), ceil(es_len/29), fs_enc, 'yaxis');
spectrogram(enc_signal, ceil(es_len/29), ceil(es_len/30), ceil(es_len/29), fs_enc, 'yaxis');

title('Spectrogram of Encoder');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, fs_enc/2])



%% Define functions
function [P1, f, Fs] = normalized_fft(signal, time)
    Fs = 1 / mean(diff(time)); % Sampling frequency
    L = length(signal); % Length of the signal
    Y = fft(signal); % Compute the FFT
    P2 = abs(Y/L); % Two-sided spectrum
    P1 = P2(1:L/2+1); % Single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1); % Correct the amplitude
    f = Fs*(0:(L/2))/L; % Frequency vector

end