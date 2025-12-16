function pltFRF(filename)

path = "outputs/" + filename;
D = readmatrix(path);

% time = rmmissing(D(:,6));
% dt = diff(time);
% 
% pos = rmmissing(D(:,5));
% force = rmmissing(D(:,3));

time = rmmissing(D(:,5));
dt = diff(time);

pos = rmmissing(D(:,4)) * 10^-3;
force = -1*rmmissing(D(:,3));
force = lowpass(force, 30, 1/mean(diff(time)));

%% Use modal frf to get the impedance frf
% win_len = 2^12;
win_len = 2^10;
% win_len = 10000;
fs = 1/mean(diff(time));

[frf, f, coh] = modalfrf(pos, force, fs, hann(win_len), 0.5*win_len, 'Sensor', 'dis');

frf_unloaded = H1_unloaded(win_len);
frf = frf - frf_unloaded;

ws = f*2*pi;
jw = (1i*ws);

frf_mag = mag2db(abs(frf./jw));
frf_ang = angle(frf./jw)*(180/pi);


%%


ax_fs = 16;         % Font size of ticks
lbl_fs = 18;        % Label font size

alpha = 0.0071; % s time dealy of the 

% Data formatted for plotting
H1_mag = frf_mag;
H1_ang = frf_ang;
MCOH1 = coh;

f_l = 1;   % Lower index limit to be plotted
f_u = 150;  % Upper index limit to be plotted

fm_l = 0.075; % Lower measured frequency
fm_u = 55;  % Upper measured frequency
xlims = [fm_l, fm_u];

figure(2);%clf
subplot(3,1,1)
semilogx(f(f_l:f_u),H1_mag(f_l:f_u),'LineWidth',2);grid on; hold on;
% loglog([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
% loglog([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims)
ylim([-10, 50])
ylabel("|F(\itf\rm)/V(\itf\rm)|", 'FontSize', lbl_fs)
title("Mass-Spring System Impedance", 'FontSize', 20)

subplot(3,1,2)
semilogx(f(f_l:f_u),H1_ang(f_l:f_u),'LineWidth',2);grid on; hold on
% semilogx([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
% semilogx([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims)
ylabel("Phase (°)", 'FontSize', lbl_fs)
ylim([-200, 200])
yticks([-180, -90, 0, 90, 180])

subplot(3,1,3)
semilogx(f(f_l:f_u),MCOH1(f_l:f_u),'LineWidth',2);grid on; hold on
%semilogx([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
%semilogx([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims)
ylim([0, 1])
ylabel("Coherence", 'FontSize', lbl_fs)
xlabel("Frequency (Hz)", "FontSize", 17)




end

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