clear, clc, close all

%% Chirp Paramters

f1 = 0.1;           % Initial frequency
f2 = 40.0;          % Frequency at sweep length

A1 = 1.75;          % Initial amplitude in mm
A2 = 0.2;           % Final amplitude in mm

chirp_length = 380; % Chirp length in seconds

% Exponential decay parameter
tau = -chirp_length/log(A2/A1);

%% Calculate Chirp and Spectrum
t = linspace(0, 420, 420*1000);

x = chirp(t, 0.1, 420, 40, "logarithmic");
x1 = exp(-t./tau).*x;
subplot(3, 1, 1)
plot(t, x), hold on
plot(t, x1)

[X_mag, X_phase, f, Fs] = normalized_fft(x, t);
[X1_mag, X1_phase, f1, Fs1] = normalized_fft(x1, t);

%% 
subplot(3, 1, 2)
loglog(f, X_mag), hold on
loglog(f1, X1_mag)

subplot(3, 1, 3)
semilogx(f, X_phase), hold on
semilogx(f1, X1_phase)


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