% clc,clear

%% Reading in the time domain data and plotting it
filename = "LukesFinger_0.1-20Hz_Fingernail_Attachment_20251121_124718.csv";
path = "outputs/" + filename;
D = readmatrix(path);

time = rmmissing(D(:,5));
dt = diff(time);

pos = rmmissing(D(:,4)) * 1e-3; %pos = lowpass(pos, 30, 1/mean(diff(time)));
force = rmmissing(D(:,3))*-1; %force = lowpass(force, 30, 1/mean(diff(time)));

%% Compute the fft of the input and output
N = length(time);
X = fft(pos); X = X(1:floor(N/2) + 1);
F = fft(force); F = F(1:floor(N/2) + 1);

Fs = mean(1/diff(time));
fs = Fs*(0:(N/2))'; % Frequency vector
ws = 2*pi*fs; % Angular frequency vector

alpha = 0.0; % s time dealy due to adc comms
jw = (1i*ws);
dly = exp(jw*alpha);

% Compute the coherence of the raw signals
% cxy = mscohere(pos, force);

%% Plot impedance direct from FFTs
% 
% Z_direct = F./X;
% Z_direct_mag_db = mag2db(abs(Z_direct./(j*ws)));
% Z_direct_ang  = angle(Z_direct./(j*ws))*(180/pi);
% 
% figure(2)
% subplot(2,1,1)
% semilogx(fs, Z_direct_mag_db)
% xlim([0.1, 25])
% grid on
% 
% subplot(2,1,2)
% semilogx(fs, Z_direct_ang)
% xlim([0.1, 25])
% ylim([-200, 200])
% yticks([-180, -90, 0, 90, 180])
% grid on


%% Compute the H1 estimator
H1 = (F .* conj(X)) ./ (X .* conj(X));

H1_mag_db = mag2db(abs(dly.*H1./jw));
H1_ang = unwrap(angle(dly.*H1./jw)*(180/pi));

%% Plot the H1 estimator

xlims = [0.1, 40];

figure(7)
subplot(3,1,1)
semilogx(fs, H1_mag_db)
xlim(xlims)
grid on

subplot(3,1,2)
semilogx(fs, H1_ang)
xlim(xlims)
ylim([-200, 200])
yticks([-180, -90, 0, 90, 180])
grid on

subplot(3,1,3)
semilogx(fs, cxy)
xlim(xlims)


%% Function definitions