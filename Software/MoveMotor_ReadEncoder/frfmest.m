function [H1,H2,Hv,ws,MCOH1,MCOH2,MCOHv,Guu,Gyu,Gyy] = frfmest(y,u,ts,Nblk,window_name,NoPCToverlap)
%
% MIMO Frequency Response Functin Estimate from input-output data.
%
% [H1,H2,Hv,ws,MCOH1,MCOH2,MCOHv,Guu,Gyu,Gyy] = frfmest(y,u,ts,Nblk,window_name,NoPCToverlap)
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
%
% Matt Allen, Spring 2005
% msalle@sandia.gov
%
% This is a work in progress.  Basic functionality has been verified, yet
% the documentation is incomplete and the options are limited.
%

% Find Spectral Density Matrices for use my algorithms.
[Guu Gyu ws Gyy] = mimocsd(y,u,ts,Nblk,window_name,NoPCToverlap);

No = size(Gyu,1); Ni = size(Gyu,2); Nw = size(Gyu,3);
H1 = zeros(No,Ni,Nw); H2 = zeros(No,Ni,Nw); Hv = zeros(No,Ni,Nw);

% Estimate FRFs using H1, H2 and Hv estimators
% Turn off warnings ??? 
% warning('OFF','MATLAB:nearlySingularMatrix');
for k = 1:Nw
    % H1 Method
    H1(:,:,k) = Gyu(:,:,k)/Guu(:,:,k);
    % H1(:,:,k) = Guu(:,:,k)\(Gyu(:,:,k)'); % Are These Different?
    Fvirt(k,:) = eig(Guu(:,:,k)).';
    
    % H2 Method
    H2(:,:,k) = Gyy(:,:,k)/(Gyu(:,:,k)');
    RGyu(k,:) = svd(Gyu(:,:,k)).';
    
    % Hv Method Allemang Course Notes v3_5.pdf eq (5.36)
    for p = 1:No
        GFFXp = [Guu(:,:,k), Gyu(p,:,k)';
                Gyu(p,:,k), Gyy(p,p,k)];
        [V,lam] = eig(GFFXp);
        [las,lsind] = sort(abs(diag(lam)));
        Hrow = (-V(:,lsind(1))/V(end,lsind(1)))';
        Hv(p,:,k) = Hrow(1,1:Ni);
    end
    
end

% Turn warnings back on
% warning('ON','MATLAB:nearlySingularMatrix');

if Ni > 1;
    figure(2000);
    subplot(2,1,1)
    semilogy(ws, Fvirt);
    title('Virtual force.  Low values might signal ill-conditioning in H_1 estimate');
        for k = 1:Ni; vflab{k} = ['Virtual Force #',num2str(k),' (H1)']; end
        legend(vflab);
    subplot(2,1,2);
    semilogy(ws, RGyu,'-.'); 
    title('Singluar Values of Gyu, relevant for H_2 estimate');
    xlabel('Frequency (rad/s)');
        for k = 1:Ni; vflab2{k} = ['S.V. (Gyu) #',num2str(k),' (H2)']; end
        legend(vflab2);
end


MCOH1 = zeros(Nw,No);
MCOH2 = zeros(Nw,No);
MCOHv = zeros(Nw,No);
if nargout > 4 & Ni == 1; % Compute Ordinary Coherence - Allemang v3_5.pdf page (5-24)
    for k = 1:Nw
        for p = 1:No
            MCOH1(k,p) = abs(Gyu(p,1,k))^2/(Gyy(p,p,k)*Guu(1,1,k));
        end
    end
    MCOH2 = MCOH1; % No formulas for these - use 1st
    MCOHv = MCOH1; % No formulas for these - use 1st
    % gam = sqrt(MCOH); % Note - COH = gam^2
    
% Compute Multiple Coherence - Allemang v3_5.pdf page (5-25), eq. 5-43 -
% this gives the linear relationship between the output and all of the
% inputs.
elseif nargout > 4 % Error in the formula?  Which H to use?
    for k = 1:Nw
        for p = 1:No
            for q = 1:Ni
                for t = 1:Ni
                    MCOH1(k,p) = MCOH1(k,p) + H1(p,q,k)*Guu(q,t,k)*conj(H1(p,t,k))/Gyy(p,p,k);
                    MCOH2(k,p) = MCOH2(k,p) + H2(p,q,k)*Guu(q,t,k)*conj(H2(p,t,k))/Gyy(p,p,k);
                    MCOHv(k,p) = MCOHv(k,p) + Hv(p,q,k)*Guu(q,t,k)*conj(Hv(p,t,k))/Gyy(p,p,k);
                end
            end
        end
    end
end

% Make sure that no spurrious imaginary parts have crept in.
if max(imag(MCOH1)./real(MCOH1)) > 1e-5; warning('Coherence is not real'); end
    MCOH1 = real(MCOH1);
if max(imag(MCOH2)./real(MCOH2)) > 1e-5; warning('Coherence is not real'); end
    MCOH2 = real(MCOH2);
if max(imag(MCOHv)./real(MCOHv)) > 1e-5; warning('Coherence is not real'); end
    MCOHv = real(MCOHv);
