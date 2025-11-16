close all, clear, clc

% Load data from a CSV file
filename = "Resonance_MoveMotorTelemetry_20251111_144443.csv";
path = "outputs/" + filename;
data = readmatrix(path); % Replace 'your_file.csv' with your actual file name

ticks_font_size = 20; % Font size of the axes
labels_font_size = 20; % Font size of the axes


%% Load the data - GS

% %Command
% cmd_signal = rmmissing(data(:, 1)); cmd_signal = cmd_signal - mean(cmd_signal);
% cmd_time = rmmissing(data(:, 2));
% %cmd_signal=tukeywin(length(cmd_signal), 0.25).*cmd_signal;
% 
% % Load Cell
% tel_signal = 100*rmmissing(data(:, 3)); tel_signal = tel_signal - mean(tel_signal);
% tel_time = rmmissing(data(:, 5));
% %tel_signal = lowpass(tel_signal, 30, 1/mean(diff(tel_time)));
% 
% % Encoder
% enc_signal = rmmissing(data(:, 4));
% enc_time = rmmissing(data(:, 5));

%% Load the data - GTE

% Command
cmd_signal = rmmissing(data(:, 1)); cmd_signal = cmd_signal - mean(cmd_signal);
cmd_time = rmmissing(data(:, 2));
%cmd_signal=tukeywin(length(cmd_signal), 0.25).*cmd_signal;

% Load Cell
tel_signal = rmmissing(data(:, 3)); tel_signal = tel_signal - mean(tel_signal);
tel_time = rmmissing(data(:, 4));
%tel_signal = lowpass(tel_signal, 30, 1/mean(diff(tel_time)));

% Encoder
enc_signal = rmmissing(data(:, 5));
enc_time = rmmissing(data(:, 6));

%% Plot the sampling distribution
cmd_sample_dist = diff(cmd_time);
tel_sample_dist = diff(tel_time);
enc_sample_dist = diff(enc_time);

figure(1)
subplot(3,1,1)
histogram(cmd_sample_dist * 1000, 5)
ax = gca;ax.FontSize = ticks_font_size;
title("Command sample period distribution")
ylabel("Num samples")

subplot(3,1,2)
histogram(tel_sample_dist * 1000, 5)
ax = gca;ax.FontSize = ticks_font_size;
title("Load Cell sample period distribution")
ylabel("Num samples")

subplot(3,1,3)
histogram(enc_sample_dist * 1000, 5)
ax = gca;ax.FontSize = ticks_font_size;
title("Encoder sample period distribution")
ylabel("Num samples")
xlabel("Sample Period (ms)")



%% Plot the three signals together

figure(2)
yyaxis left
plot(cmd_time, cmd_signal, '-','Color', [0, 0.4470, 0.7410], "LineWidth", 2), hold on
plot(enc_time, enc_signal, '-', 'Color', [0, 0.5, 0], "LineWidth", 2)
ax = gca; ax.FontSize = ticks_font_size;
ylabel("Position (mm)", FontSize=labels_font_size)

yyaxis right
plot(tel_time, tel_signal, '-', 'Color', [0.8500, 0.3250, 0.0980], "LineWidth", 1.5)
ylabel("Force (N)", FontSize=labels_font_size)
ylim([-0.8, 0.8])

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

[P1_cmd, f_cmd, fs_cmd] = normalized_fft(cmd_signal, cmd_time);
[P1_tel, f_tel, fs_tel] = normalized_fft(tel_signal, tel_time);
[P1_enc, f_enc, fs_enc] = normalized_fft(enc_signal, enc_time);

z_cmd = fftshift(P1_cmd);

figure(4)
% subplot(2,1,1)
loglog(f_cmd, P1_cmd, "LineWidth", 1.5), hold on
%loglog(f_tel, P1_tel, "LineWidth", 1.5)
%loglog(f_enc, P1_enc, "LineWidth", 1.5)
ax = gca; ax.FontSize=ticks_font_size;
ylabel("Amplitude", FontSize=labels_font_size)
%legend("Commanded", "Load Cell", "Linear Encoder")

grid on
xlim([0.05, min([0.5*fs_cmd, 0.5*fs_tel, 0.5*fs_enc])])
ylim([10^-6 5*10^-1])

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
    ceil(es_len/30), ceil(es_len/29), fs_enc, 'yaxis');
spectrogram(enc_signal, ceil(es_len/29), ceil(es_len/30), ceil(es_len/29), fs_enc, 'yaxis');

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

function alignTicks(ax)
    % --- Alignment Logic ---
    % Get the left y-axis tick locations and limits
    left_yticks = ax.YTick;
    left_ylim = ax.YLim;
    
    % Get the right y-axis limits
    right_ylim = ax.YAxis(2).Limits;
    
    % Check if left_ylim is not equal to right_ylim to avoid division by zero
    if left_ylim(1) ~= left_ylim(2) && right_ylim(1) ~= right_ylim(2)
        % Calculate the proportional positions of the left ticks within their limits
        % Formula: (tick_value - lower_limit) / (upper_limit - lower_limit)
        tick_proportions = (left_yticks - left_ylim(1)) ./ (left_ylim(2) - left_ylim(1));
        
        % Calculate the corresponding tick values for the right axis 
        % using the same proportions within its limits
        right_yticks = tick_proportions .* (right_ylim(2) - right_ylim(1)) + right_ylim(1);
        
        % Set the calculated tick values for the right y-axis
        ax.YAxis(2).Ticks = right_yticks;
    else
        warning('Y-axis limits are equal, tick alignment cannot be performed.');
    end
end