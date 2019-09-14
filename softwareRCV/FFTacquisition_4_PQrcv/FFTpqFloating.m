function X_K = FFTpqFloating(xk)
%function X_K = FFTpqFloating(xk,k_factor)
tic;
N = length(xk);
X_K = zeros(1,N);
h=waitbar(0,'Pls wait');
W=-i*2*pi/N;
for index_XK=1:N
    waitbar(index_XK/N,h);
    for index_xk = 1:N
        a(index_xk) = xk(index_xk)*exp((index_xk-1)*(index_XK-1)*W);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % This approach is about twice slower
        %        a(index_xk) = xk(index_xk)*W^((index_xk-1)*(index_XK-1));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    X_K(index_XK) = sum(a);
end
close(h);

toc;