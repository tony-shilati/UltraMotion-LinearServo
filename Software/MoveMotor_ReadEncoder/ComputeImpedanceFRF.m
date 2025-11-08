%% reading in the time domain data and plotting it
clc,clear

filename = "MoveMotorTelemetry_20251107_105828.csv";
path = "outputs/" + filename;
D = readmatrix(path);

time = rmmissing(D(:,6));
dt = diff(time);

pos = rmmissing(D(:,5));
force = rmmissing(D(:,3));
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
% waterplot(s,f,t)
spectrogram(y, Nblk, floor(Nblk*NoPCToverlap),Nblk*2,fs)

[H1,H2,Hv,ws,MCOH1,MCOH2,MCOHv,Guu,Gyu,Gyy] = frfmest(y,u,ts,Nblk,window_name,NoPCToverlap);

%%

xlims = [0.1, 30];

figure(7);clf
subplot(3,1,1)
loglog(ws/(2*pi),squeeze(abs(H1(1,:)./(1i*ws))),'LineWidth',2);grid on; hold on
xlim(xlims)
ylabel("|F(w)/V(w)|")

subplot(3,1,2)
semilogx(ws/(2*pi),squeeze(angle(H1(1,:)./(1i*ws))*(180/pi)),'LineWidth',2);grid on; hold on
xlim(xlims)
ylabel("Phase (°)")
ylim([-200, 200])
yticks([-180, -90, 0, 90, 180])

subplot(3,1,3)
semilogx(ws/(2*pi),MCOH1,'LineWidth',2);grid on; hold on
xlim(xlims)
ylabel("H2 Coherence")
xlabel("Frequency (Hz)")