% function restored_x = inverseFilter(y,h,gamma)
% %% 广义逆滤波
% N = size(y,1);  %大小
% Yf = fft2(y);   %频域变换
% Hf = fft2(h,N,N); %点扩散函数
% sHf = Hf.*(abs(Hf)>0)+1/gamma*(abs(Hf)==0);  %处理为零的情况
% iHf = 1./sHf;     %求逆
% iHf = iHf.*(abs(Hf)*gamma>1)+gamma*abs(sHf).*iHf.*(abs(sHf)*gamma<=1);
% restored_x = realifft2(iHf.*Yf));   %复原图像
function Z=fftfilter1(X,H)
s1= size(H);
s2 = size(X);
F=fft2(X, s1(1), s1(2));
% F=fft2(X, size(H,1), size(H, 2));
Z=H.*F;
Z=ifftshift(Z);
Z=abs(ifft2(Z));
Z=Z(1:s2(1),1: s2(2));
end
