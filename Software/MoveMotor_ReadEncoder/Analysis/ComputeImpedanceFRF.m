%% reading in the time domain data and plotting it
clc,clear

filename = "UndampedOscillator_0_to_20Hz_ADS1256Direct_10kgLC_20251204_141322.csv";
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
% pos = lowpass(pos, 100, 1/mean(diff(time)));
force = rmmissing(D(:,3))*-1;
% force = lowpass(force, 100, 1/mean(diff(time)));
% force = highpass(force, 7, 1/mean(diff(time)));

% figure(6);
% subplot(2,1,1);
% plot(time,pos,'linewidth',2);grid on; hold on;
% subplot(2,1,2)
% plot(time,force,'LineWidth',2);grid on; hold on;


%% Selecting the "good" data off the initial plot

%  G = ginput(2)
% I1 = floor(G(1,1)/dt(1))
% I2 = floor(G(2,1)/dt(2))

I1 = 1;
I2 = length(time);

figure(6);clf
subplot(2,1,1)
plot(time(I1:I2),pos(I1:I2),'linewidth',2);grid on; hold on;
subplot(2,1,2)
plot(time(I1:I2),force(I1:I2),'LineWidth',2);grid on; hold on;


%%

%   y - output (Nt x No)
%   u - input (Nt x Ni)
%   ts - time vector (Nt x 1)
%   Nblk = number of samples per block - see mimocsd.m
%   window_name - 'rect', 'hanning' see mimocsd.m
%   NoPCToverlap - number of samples to overlap or percent of records to
%       overlap - see mimocsd.m
%   
%   H1, H2, Hv - FRF estimate using the named methods
%   MCOH - Coherence:  ordinary coherence is returned for SISO/SIMO data,
%       multiple coherence for MIMO or MISO data.
%   Guu, Gyu, Gyy - Auto/Cross Spectrum matrices returned if desired.

ts = time(I1:I2);
y = force(I1:I2);
u = pos(I1:I2);

Nblk = 2048;
window_name = 'hanning';
NoPCToverlap = 0.5;
fs = 1/dt(1);

% figure(8)
% % [s, f, t] = spectrogram(y, Nblk, floor(Nblk*NoPCToverlap));
% spectrogram(y, Nblk, floor(Nblk*NoPCToverlap),Nblk*2,fs)


[H1,H2,Hv,ws,MCOH1,MCOH2,MCOHv,Guu,Gyu,Gyy] = frfmest(y,u,ts,Nblk,window_name,NoPCToverlap);

%%

xlims = [0.5, 40];
ax_fs = 16;         % Font size of ticks
lbl_fs = 18;        % Label font size

alpha = 0.007071; % s time dealy of the 
jw = (1i*ws);

% Data formatted for plotting
f = ws/(2*pi); % convert frequency to radians
H1_mag = squeeze(mag2db(abs(exp(jw*alpha).*H1(1,:)./jw)));
% H1_mag = squeeze(abs(exp(jw*alpha).*H1(1,:)./jw));
H1_ang = squeeze(angle(exp(jw*alpha).*H1(1,:)./jw)*(180/pi));

f_l = 1;   % Lower index limit to be plotted
f_u = length(H1_mag);  % Upper index limit to be plotted

fm_l = 0.; % Lower measured frequency
fm_u = 20;  % Upper measured frequency

% % % Theoroetical bode plot
% m = 0.0825;        % Lukes robot finger 
% k = 320;
% b = 1.8;

% m = 0.065;      % Lukes actual finger
% k = 185;
% b = 1.025;

%
% m = 0.060;        % Unloaded mass 
% k = 0;
% b = 0;

m = 0.06;        % Mass-Spring System 
k = 250;
b = 0;



sys = tf([m, b, k], [1, 0]);
[mag, phase] = bode(sys, ws);
mag_db = mag2db(squeeze(mag)); phase = squeeze(phase);


figure(7);%clf
subplot(3,1,1)
semilogx(f(f_l:f_u),H1_mag(f_l:f_u),'LineWidth',2); hold on
semilogx(f,mag_db,'k--','LineWidth',1.5)
% loglog([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
% loglog([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims), ylim([-30, 50])
ylabel("|F(\itf\rm)/V(\itf\rm)|", 'FontSize', lbl_fs)
title("System Impedance", 'FontSize', 20)
grid on; 

subplot(3,1,2)
semilogx(f(f_l:f_u),H1_ang(f_l:f_u),'LineWidth',2); hold on
semilogx(f,phase,'k--','LineWidth',1.5)
% semilogx([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
% semilogx([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims), ylim([-200, 200])
ylabel("Phase (°)", 'FontSize', lbl_fs)
xlabel("Frequency (Hz)", "FontSize", 17)
yticks([-180, -90, 0, 90, 180])
grid on; 

subplot(3,1,3)
semilogx(f(f_l:f_u),MCOH1(f_l:f_u),'LineWidth',2);grid on; hold on
%semilogx([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
%semilogx([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims), ylim([0, 1])
ylabel("Coherence", 'FontSize', lbl_fs)
xlabel("Frequency (Hz)", "FontSize", 17)


%% Effective Stiffness, Damping, and Mass

H1_real = squeeze(real(exp(jw*alpha).*H1(1,:)./jw));
H1_imag = squeeze(imag(exp(jw*alpha).*H1(1,:)./jw));
H1_ang = squeeze(angle(exp(jw*alpha).*H1(1,:)./jw)*(180/pi));

K_eff = zeros(1, length(H1_ang));
B_eff = zeros(1, length(H1_ang));
M_eff = zeros(1, length(H1_ang));

% Effective stiffness
for i = 1:length(H1_ang)
    if (-180 < H1_ang(i) && H1_ang(i) < 0) && H1_imag(i) <= 0
        K_eff(i) = abs(H1_imag(i)) * ws(i);
    else
        K_eff(i) = 0;
    end
end

% Effective damping
for i = 1:length(H1_ang)
    if (-90 < H1_ang(i) && H1_ang(i) < 90) && H1_real(i) >= 0
        B_eff(i) = abs(H1_real(i));
    else
        B_eff(i) = 0;
    end
end

% Effective mass
for i = 1:length(H1_ang)
    if (0 < H1_ang(i) && H1_ang(i) < 180) && H1_imag(i) >= 0
        M_eff(i) = abs(H1_imag(i)) / ws(i);
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
xlim([10, 20]);

subplot(3,1,2), hold on
semilogx(f(f_l:f_u), B_eff(f_l:f_u), 'LineWidth', 1.5)
ylabel("Effective Damping (Ns/m)")
grid on
xlim([10, 20]);

subplot(3,1,3), hold on
semilogx(f(f_l:f_u), M_eff(f_l:f_u), 'LineWidth', 1.5)
ylabel("Effective Mass (kg)")
xlabel("Frequency (Hz)")
grid on
xlim([10, 20]);


%% Nyquist plot
figure(10)
plot(H1_real(f_l:f_u), H1_imag(f_l:f_u), LineWidth=1.5)
xlabel("Re\{H1\}"), ylabel("Im\{H1\}")
ax = gca; ax.FontSize = 20;
title("Nyquist Plot of Impedance FRF", FontSize=22)
grid on
axis equal
% a = 0.4;
% axis([-a, a, -a, a])


