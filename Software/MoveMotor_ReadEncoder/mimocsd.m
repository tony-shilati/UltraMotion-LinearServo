function [Guu, Gyu, ws, Gyy] = mimocsd(y,u,ts,Nblk,window_name,NoPCToverlap,varargin)
% 
% Function which finds Spectral Density Matrices for MIMO (or SIMO / SISO)
% transfer function estimation.  Use psdest.m for standard psd estimation.
%
% [Guu, Gyu, ws, Gyy] = mimocsd(y,u,ts,Nblk,window_name,NoPCToverlap)
% [Guu, Gyu, ws, Gyy] = mimocsd(y,u,ts,Nblk,window_name,NoPCToverlap,'rs','peak')
%
% size(y) = Nt x No
% size(u) = Nt x Ni
%
% ts = time vector corresponding to y or u, or sample time
%
% Nblk         = Number of samples in each block.  The resulting spectra
%               have Nblk/2+1 lines.
% window_name  = 'rect'
%                'hanning' (default)
%                this may also be an Nblk length vector containing the window.                 
% NoPCToverlap = if > 1, number of samples to overlap in records
%              = if < 1, percent of individual records to overlap
%
% ## SCALING ###
%
% [Guu, Gyu, ws, Gyy] = mimocsd(y,u,ts,Nblk,window_name,NoPCToverlap,'rs','peak')
%
% The final input argument tells the algorithm how to scale the PSD.
% 'peak' - [default] Scale the PSD such that the absolute value of the PSD 
%       is approximately equal to the peak amplitude of the harmonic 
%       component in the time domain.  Equivalent to (2/N) scaling on an
%       FFT or Fourier Coefficeint scaling (Ginsberg, eq. 3.7.36).
% 'mnsq' - Scale the PSD so that the integral of the PSD with the frequency
%       axis in Hz (area under the curve) is equal to the mean square of 
%       signal
% 'none' - Scale PSD by (1/Navg) only - no correction for window.
%
% M. S. Allen
% msallen@engr.wisc.edu - modified 2010-12-15 (added scaling options).
%

% Setup input parameters
%

if nargin > 6;
    fmt_flg = varargin{1};
else
    fmt_flg = 'none';
end

if nargin > 7;
    sc_meth = varargin{2};
    if ~any(strcmpi(sc_meth,{'peak','mnsq','none'}));
        error('Unrecognized Scale Method.  Choices are: peak, mnsq, none');
    end
else
    sc_meth = 'peak';
end

Nt = size(y,1); if size(u,1) ~= Nt; error('Dimensions of y and u do not agree'); end
No = size(y,2); Ni = size(u,2);

if length(ts) > 1;
    dt = ts(2)-ts(1);
else
    dt = ts;
end

if Nblk > Nt; % zero pad the response and input
    y(Nblk) = 0; 
    u(Nblk) = 0;
end

if isempty(window_name)
    window_name = 'hanning';
end

if strcmpi(window_name,'hanning') || strcmpi(window_name,'hann')
    % Do not include zero samples at beginning and end. Symmetric Hanning Window
    %window = 0.5*(1-cos([1:Nblk].'*2*pi/(Nblk+1)));
    % Periodic Hanning Window - include zero sample at the beginning.
    window = 0.5*(1-cos([0:(Nblk-1)].'*2*pi/(Nblk)));
elseif strcmpi(window_name,'rectangular') || strcmpi(window_name,'rect')
    window = ones(Nblk,1);
elseif strcmpi(window_name,'test')
    % Do not include zero samples at beginning and end. (Standard Practice)
    window = 0.5*(1-cos([1:Nblk].'*2*pi/(Nblk+1)));
elseif ~ischar(window_name)
    window = window_name;
    % if an actual window is passed in, e.g. from the signal processing toolbox.
else
    error('Unsupported Window');
end

% if strcmp(lower(window_name),'hanning') | strcmp(lower(window_name),'hann')
%     % Do not include zero samples at beginning and end. Symmetric Hanning Window
%     %window = 0.5*(1-cos([1:Nblk].'*2*pi/(Nblk+1)));
%     % Periodic Hanning Window - include zero sample at the beginning.
%     window = 0.5*(1-cos([0:(Nblk-1)].'*2*pi/(Nblk)));
% elseif strcmp(lower(window_name),'rectangular') | strcmp(lower(window_name),'rect')
%     window = ones(Nblk,1);
% elseif strcmp(lower(window_name),'test')
%     % Do not include zero samples at beginning and end. (Standard Practice)
%     window = 0.5*(1-cos([1:Nblk].'*2*pi/(Nblk+1)));
% else
%     error('Unsupported Window');
% end

if isempty(NoPCToverlap) | NoPCToverlap == 0
    % No overlap
    Novlp = 0;
elseif NoPCToverlap > 1
    % Number of points to overlap specified
    Novlp = NoPCToverlap;
elseif NoPCToverlap < 1;
    % Percent to overlap specified
    Novlp = floor(NoPCToverlap*Nblk);
else
    error('NoPCToverlap not recognized');
    NoPCToverlap
end

Navg = floor((Nt-Nblk)/(Nblk-Novlp)) + 1
    % Subtract off block size, the number of times the results is divisible
    % by (Nblk-Novlp) gives Navg - 1
Nex = Nt - (Nblk-Novlp)*Navg;
if Nex ~= 0;
    disp([num2str(Nex),' Samples disregarded in spectrum estimation']);
end

% save mimocsd_debug.mat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Guu = zeros(Ni,Ni,(Nblk/2+1));
Gyu = zeros(No,Ni,(Nblk/2+1));
yw = zeros(Nblk,No);
uw = zeros(Nblk,Ni);
if nargout > 3; Gyy = zeros(No,No,(Nblk/2+1)); end

for m = 1:Navg % Change to parfor - parallelize
    blk = (Nblk-Novlp)*(m-1) + [1:Nblk];
    % Create block of windowed data.
    for k = 1:No
        yw(:,k) = y(blk,k).*window;
    end
    for k = 1:Ni
        uw(:,k) = u(blk,k).*window;
    end
    U = fft(uw);
    Y = fft(yw);
    for k = 1:(Nblk/2+1) % real IO data only
        Guu(:,:,k) = Guu(:,:,k) + (U(k,:).')*conj(U(k,:));
        Gyu(:,:,k) = Gyu(:,:,k) + (Y(k,:).')*conj(U(k,:));
    end
    if nargout > 3; 
        for k = 1:(Nblk/2+1)
            Gyy(:,:,k) = Gyy(:,:,k) + (Y(k,:).')*conj(Y(k,:));
        end
    end
end

if strcmpi(sc_meth,'mnsq');
    % Scale: Gives area under PSD = mean square of signal when the x-axis
    % is frequency in Hz.
    sc_fact = (2*dt)/(Navg*norm(window)^2);

    % Scaling from PSD in Matlab - not sure when one would want this.
    % sc_fact = 1/(Navg*norm(window)^2);
elseif strcmpi(sc_meth,'peak');
    % Derived by MSA:
    % This scaling gives height of peaks = amplitude of the corresponding
    % harmonic in the time domain. (Equivalent to 2/N scaling for FFT)
    sc_fact = (1/Navg)*(4/sum(window)^2);
else % none
    sc_fact = (1/Navg); % always divide by number of averages
end

if nargout > 3; 
    Gyy = Gyy*sc_fact;
end
Guu = Guu*sc_fact;
Gyu = Gyu*sc_fact;

ws = [0:1:(Nblk/2)]*2*pi/(dt*Nblk);

if strcmpi(fmt_flg,'rs');
    if nargout > 3; 
        Gyy = permute(Gyy,[3,1,2]);
    end
    Guu = permute(Guu,[3,1,2]);
    Gyu = permute(Gyu,[3,1,2]);
end

% figure(100);
% subplot(2,1,1)
% plot(ws,abs(squeeze(Gyy(1,1,:)))); grid on;
% ylabel('Gyy'); xlabel('\omega');
% subplot(2,1,2)
% plot(ws,abs(squeeze(Guu(1,1,:)))); grid on;
% ylabel('Guu'); xlabel('\omega');