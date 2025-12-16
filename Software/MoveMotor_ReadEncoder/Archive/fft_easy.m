function [Xfft,ws] = fft_easy(xt,ts,varargin)
% [Xfft ws] = fft_easy(xt,ts,DIM);
% 
% Function which takes the FFT of the real time sequence "xt" and
% returns the FFT for positive frequencies and the corresponding
% frequency vector in radians per second. (rad/s).
%   xt - sampled time response or array of time responses.  If xt is a matrix,
%        the FFT operates on the larger dimension of xt, unless the
%        optional argument DIM is given.
%   ts - Time between samples or vector of equally spaced time samples.
% Optional Arguments:
%   DIM - (optional) the dimension over which the FFT acts (1 or 2D only)
% Output Arguments:
%   Xfft - complex FFT coefficients of xt.
%   ws - frequency vector corresponding to Xfft in rad/s.
% If no output arguments are requested, the magnitude and phase of Xfft
%   are plotted in a new figure window.
% 
% Note:  To obtain an approximation of the Fourier series coefficients,
%        this fft should be multiplied by (2/N) where N is the length
%        of the time sequence.  No Scaling is applied by fft_easy.
%
% Matt Allen Spring 2005
% msallen@engr.wisc.edu
%
% TO DO:  Modify code to allow for arrays of > 3 dimensions
%

if length(ts) > 1;
    dt = ts(2)-ts(1);
    if abs(min(diff(ts)) - max(diff(ts)))/abs(max(diff(ts))) > 1e-6;
        error(['Time samples are not equally spaced, ',...
            'min(diff(ts))=',num2str(min(diff(ts))),',  max(diff(ts))=',num2str(max(diff(ts)))]);
    end
else
    dt = ts;
end

if nargin > 2
    DIM = varargin{1};
    if DIM == 2;
        xt = xt.';
    end
else
    DIM = [];
end
if (isempty(DIM) || nargin < 3) && size(xt,3) == 1;
    [a,b] = size(xt);
    if b > a;
        xt = xt.';
    end
end

% The time sequences are now the columns of xt.
N = size(xt,1);

Xfft = fft(xt,[],1);
Xfft = Xfft([1:(N/2+1)],:,:);
% Create Frequency vector in rad/s
w1 = 2*pi/(N*dt);
ws = [0:w1:w1*N/2].';

if ~isempty(varargin)
    DIM = varargin{1};
    if DIM == 2;
        Xfft = Xfft.';
    end
end

if nargout < 1;
    if nargin > 3
        figure(varargin{2})
    else
        figure
    end
    ht = subplot(3,1,1:2)
    semilogy(ws/2/pi,abs(Xfft)); grid on;
    set(get(gca,'Children'),'LineWidth',2);
    title('FFT(x)');
    xlabel('Frequency (Hz)'); ylabel('|FFT(x)|');
    subplot(3,1,3)
    plot(ws/2/pi,angle(Xfft)*180/pi); grid on;
    set(get(gca,'Children'),'LineWidth',2);
    xlabel('Frequency (Hz)'); ylabel('Phase (FFT(x)) (^o)');
    axes(ht);
end
    