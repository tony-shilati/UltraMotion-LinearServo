close, clear, clc

% Load data from a CSV file
data = readmatrix('MoveMotorTelemetry_20251105_164813.csv'); % Replace 'your_file.csv' with your actual file name

%% Load the data

% Command
cmd_signal = rmmissing(data(:, 1)); cmd_signal = cmd_signal - mean(cmd_signal);
cmd_time = rmmissing(data(:, 2));

% Telemetry
tel_signal = rmmissing(data(:, 3)); tel_signal = tel_signal - mean(tel_signal);
tel_time = rmmissing(data(:, 4));


% Encoder
enc_signal = rmmissing(data(:, 5));
enc_time = rmmissing(data(:, 6));

%% Plot the three signals together

figure(1)
plot(cmd_time, cmd_signal, "LineWidth", 1.5), hold on
plot(tel_time, tel_signal, "LineWidth", 1.5)
plot(enc_time, enc_signal, "LineWidth", 1.5)

legend("Commanded", "Telemetry", "Linear Encoder")
xlabel("Time (s)")
ylabel("Position (mm)")

%% Plot the fft of all three together

[P1_cmd, f_cmd, fs_cmd] = normalized_fft(cmd_signal, cmd_time);
[P1_tel, f_tel, fs_tel] = normalized_fft(tel_signal, tel_time);
[P1_enc, f_enc, fs_enc] = normalized_fft(enc_signal, enc_time);

figure(2)
loglog(f_cmd, P1_cmd, "LineWidth", 1.5), hold on
loglog(f_tel, P1_tel, "LineWidth", 1.5)
loglog(f_enc, P1_enc, "LineWidth", 1.5)

grid on
xlim([1, max([0.5*fs_cmd, 0.5*fs_tel, 0.5*fs_enc])])
legend("Commanded", "Telemetry", "Linear Encoder")
xlabel("Frequency (Hz)")
ylabel("Amplitude (mm)")

%% Command spectrogram

% Plot the spectrogram of the signal
figure(3)
subplot(3,1,1)
s = spectrogram(cmd_signal, ceil(length(cmd_signal)/29), ...
    ceil(length(cmd_signal)/30), ceil(length(cmd_signal)/29), fs_cmd, 'yaxis');
spectrogram(cmd_signal, 1536, 1500, 1536, fs_cmd, 'yaxis');

title('Spectrogram of Command');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, 50])


%% Telemetry Spectrogram

% Plot the spectrogram of the signal
subplot(3,1,2)
s = spectrogram(tel_signal, ceil(length(tel_signal)/29), ...
    ceil(length(tel_signal)/30), ceil(length(tel_signal)/29), fs_tel, 'yaxis');
spectrogram(tel_signal, 1536, 1500, 1536, fs_tel, 'yaxis');

title('Spectrogram of Telemetry');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, 50])




%% Encoder spectrogram

% Plot the spectrogram of the signal
subplot(3,1,3)
s = spectrogram(enc_signal, ceil(length(enc_signal)/29), ...
    ceil(length(enc_signal)/30), ceil(length(enc_signal)/29), fs_enc, 'yaxis');
spectrogram(enc_signal, 1536, 1500, 1536, fs_enc, 'yaxis');

title('Spectrogram of Encoder');
xlabel('Time (s)');
ylabel('Frequency (Hz)');
colorbar;
colors = clim;
clim([colors(2) - 60, colors(2)])
ylim([0, 50])



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