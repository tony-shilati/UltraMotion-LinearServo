%% reading in the time domain data and plotting it
clc,clear

filename = "Undamped_Oscillator_ADS1220_0.1_to_40Hz_5kgLC_20251215_133907.csv";
path = "outputs/" + filename;
D = readmatrix(path);

% time = rmmissing(D(:,6));
% dt = diff(time);
% 
% pos = rmmissing(D(:,5))*1e-3;
% force = rmmissing(D(:,3));

time = rmmissing(D(:,5));
dt = diff(time);

pos = rmmissing(D(:,4)) * 1e-3;
% pos = lowpass(pos, 70, 1/mean(diff(time)));

force = rmmissing(D(:,3))*-1;
% force = lowpass(force, 70, 1/mean(diff(time)));


%% Use modal frf to get the impedance frf
% win_len = 2^12;
win_len = 2^10;
% win_len = 10000;
fs = 1/mean(diff(time));

[frf, f, coh] = modalfrf(pos, force, fs, hann(win_len), 0.5*win_len, 'Sensor', 'dis');

%%%%%%%%%% Subtract out the unloaded response
% frf_unloaded = H1_unloaded(win_len);
% frf = frf - frf_unloaded;
%%%%%%%%%%

ws = f*2*pi;
jw = (1i*ws);

frf_mag = mag2db(abs(frf./jw));
frf_ang = angle(frf./jw)*(180/pi);

%% Plotting parameters


ax_fs = 16;         % Font size of ticks
lbl_fs = 18;        % Label font size

f_l = 1;   % Lower index limit to be plotted
f_u = length(frf_mag);  % Upper index limit to be plotted

fm_l = 0.1; % Lower measured frequency
fm_u = 55;  % Upper measured frequency

xlims = [fm_l, fm_u];

%% Theoroetical bode plot
% m = 0.0825;        % Lukes robot finger 
% k = 320;
% b = 1.8;

m = 0.015;        % Lukes robot finger on new system
k = 423;
b = 0.8;

% m = 0.065;      % Lukes actual finger
% k = 185;
% b = 1.025;

%
% m = 0.060;        % Unloaded mass 
% k = 0;
% b = 0;

% m = 0.05197;        % Mass-Spring System 
% k = 215;
% b = 0;

% m = 0.02724;        % Ball joint
% k = 0;
% b = 0;

sys = tf([m, b, k], [1, 0]);
[mag, phase] = bode(sys, ws);
mag_db = mag2db(squeeze(mag)); phase = squeeze(phase);

%% Plot the impedance FRFs

figure(7);%clf
subplot(3,1,1)
semilogx(f(f_l:f_u),frf_mag(f_l:f_u),'LineWidth',2); hold on
semilogx(f,mag_db,'k--','LineWidth',1.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims), ylim([-30, 50])
ylabel("|F(\itf\rm)/V(\itf\rm)|", 'FontSize', lbl_fs)
title("System Impedance", 'FontSize', 20)
grid on; 

subplot(3,1,2)
semilogx(f(f_l:f_u),frf_ang(f_l:f_u),'LineWidth',2); hold on
semilogx(f,phase,'k--','LineWidth',1.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims), ylim([-200, 200])
ylabel("Phase (°)", 'FontSize', lbl_fs)
xlabel("Frequency (Hz)", "FontSize", 17)
yticks([-180, -90, 0, 90, 180])
grid on; 

subplot(3,1,3)
semilogx(f(f_l:f_u), coh(f_l:f_u), 'LineWidth',2);grid on; hold on
ax = gca; ax.FontSize = ax_fs;
xlim(xlims), ylim([0, 1])
ylabel("Coherence", 'FontSize', lbl_fs)
xlabel("Frequency (Hz)", "FontSize", 17)


%% Effective Stiffness, Damping, and Mass

frf_real = real(frf./jw);
frf_imag = imag(frf./jw);
frf_ang = angle(frf./jw)*(180/pi);

K_eff = zeros(1, length(frf_ang));
B_eff = zeros(1, length(frf_ang));
M_eff = zeros(1, length(frf_ang));

% Effective stiffness
for i = 1:length(frf_ang)
    if (-180 < frf_ang(i) && frf_ang(i) < 0) && frf_imag(i) <= 0
        K_eff(i) = abs(frf_imag(i)) * ws(i);
    else
        K_eff(i) = 0;
    end
end

% Effective damping
for i = 1:length(frf_ang)
    if (-90 < frf_ang(i) && frf_ang(i) < 90) && frf_real(i) >= 0
        B_eff(i) = abs(frf_real(i));
    else
        B_eff(i) = 0;
    end
end

% Effective mass
for i = 1:length(frf_ang)
    if (0 < frf_ang(i) && frf_ang(i) < 180) && frf_imag(i) >= 0
        M_eff(i) = abs(frf_imag(i)) / ws(i);
    else
        M_eff(i) = 0;
    end
end

% Plots
figure(9)
subplot(3,1,1), hold on
semilogx(f(f_l:f_u), K_eff(f_l:f_u), 'LineWidth', 1.5)
ylabel("Effective Stiffness (N/m)")
grid on
xlim(xlims);

subplot(3,1,2), hold on
semilogx(f(f_l:f_u), B_eff(f_l:f_u), 'LineWidth', 1.5)
ylabel("Effective Damping (Ns/m)")
grid on
xlim(xlims);

subplot(3,1,3), hold on
semilogx(f(f_l:f_u), M_eff(f_l:f_u), 'LineWidth', 1.5)
ylabel("Effective Mass (kg)")
xlabel("Frequency (Hz)")
grid on
xlim(xlims);


%% Nyquist plot
figure(10)
plot(frf_real(f_l:f_u), frf_imag(f_l:f_u), LineWidth=1.5)
xlabel("Re\{H1\}"), ylabel("Im\{H1\}")
ax = gca; ax.FontSize = 20;
title("Nyquist Plot of Impedance FRF", FontSize=22)
grid on
axis equal
% a = 0.4;
% axis([-a, a, -a, a])



%% Function for subtracting out unloaded response
function frf = H1_unloaded(win_len)
path = "outputs/Unloaded_balljoint_0.1_to_50hz_5kgLC_20251216_125025.csv";
D = readmatrix(path);

time = rmmissing(D(:,5));

pos = rmmissing(D(:,4)) * 10^-3;
force = -1*rmmissing(D(:,3));
force = lowpass(force, 30, 1/mean(diff(time)));

%% Use modal frf to get the impedance frf
fs = 1/mean(diff(time));

[frf, f, coh] = modalfrf(pos, force, fs, hann(win_len), 0.5*win_len, 'Sensor', 'dis');

ws = f*2*pi;
jw = (1i*ws);

frf_mag = mag2db(abs(frf./jw));
frf_ang = angle(frf./jw)*(180/pi);
end