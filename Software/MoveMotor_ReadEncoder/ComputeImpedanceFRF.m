%% reading in the time domain data and plotting it
clc,clear

filename = "Resonance_MoveMotorTelemetry_20251111_144443.csv";
path = "outputs/" + filename;
D = readmatrix(path);

time = rmmissing(D(:,6));
dt = diff(time);

pos = rmmissing(D(:,5));
force = rmmissing(D(:,3));

% time = rmmissing(D(:,5));
% dt = diff(time);
% 
% pos = rmmissing(D(:,4));
% force = -1*rmmissing(D(:,3));
% force = lowpass(force, 30, 1/mean(diff(time)));

figure(6);
subplot(2,1,1);
plot(time,pos,'linewidth',2);grid on; hold on;
subplot(2,1,2)
plot(time,force,'LineWidth',2);grid on; hold on;


%% Selecting the "good" data off the initial plot

%  G = ginput(2)
% I1 = floor(G(1,1)/dt(1))
% I2 = floor(G(2,1)/dt(2))

I1 = 1;
I2 = length(time);

figure(7);clf
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

Nblk = 1000;
window_name = 'hanning';
NoPCToverlap = 0.5;
fs = 1/dt(1);

figure(8)
% [s, f, t] = spectrogram(y, Nblk, floor(Nblk*NoPCToverlap));
spectrogram(y, Nblk, floor(Nblk*NoPCToverlap),Nblk*2,fs)


[H1,H2,Hv,ws,MCOH1,MCOH2,MCOHv,Guu,Gyu,Gyy] = frfmest(y,u,ts,Nblk,window_name,NoPCToverlap);

%%

xlims = [0.1, 50];
ax_fs = 16;         % Font size of ticks
lbl_fs = 18;        % Label font size

alpha = 0.0071; % s time dealy of the 
jw = (1i*ws);

% Data formatted for plotting
f = ws/(2*pi); % convert frequency to radians
H1_mag = squeeze(abs(exp(jw*alpha).*H1(1,:)./jw));
H1_ang = squeeze(angle(exp(jw*alpha).*H1(1,:)./jw)*(180/pi));

f_l = 1;   % Lower index limit to be plotted
f_u = 64;  % Upper index limit to be plotted

fm_l = 0.5; % Lower measured frequency
fm_u = 20;  % Upper measured frequency

figure(7);clf
subplot(3,1,1)
semilogx(f(f_l:f_u),mag2db(H1_mag(f_l:f_u)),'LineWidth',2);grid on; hold on;
loglog([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
loglog([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims)
ylim([-10^2, 0])
ylabel("|F(\itf\rm)/V(\itf\rm)| (db)", 'FontSize', lbl_fs)
title("Mass-Spring System Impedance", 'FontSize', 20)

subplot(3,1,2)
semilogx(f(f_l:f_u),H1_ang(f_l:f_u),'LineWidth',2);grid on; hold on
semilogx([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
semilogx([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims)
ylabel("Phase (°)", 'FontSize', lbl_fs)
ylim([-200, 200])
yticks([-180, -90, 0, 90, 180])

subplot(3,1,3)
semilogx(f(f_l:f_u),MCOH1(f_l:f_u),'LineWidth',2);grid on; hold on
semilogx([fm_l, fm_l], [-200, 200], '--k', 'LineWidth', 0.5)
semilogx([fm_u, fm_u], [-200, 200], '--k', 'LineWidth', 0.5)
ax = gca; ax.FontSize = ax_fs;
xlim(xlims)
ylim([0, 1])
ylabel("Coherence", 'FontSize', lbl_fs)
xlabel("Frequency (Hz)", "FontSize", 17)


