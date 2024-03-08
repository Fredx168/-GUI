% function restored_x = inverseFilter(y,h,gamma)
% %% �������˲�
% N = size(y,1);  %��С
% Yf = fft2(y);   %Ƶ��任
% Hf = fft2(h,N,N); %����ɢ����
% sHf = Hf.*(abs(Hf)>0)+1/gamma*(abs(Hf)==0);  %����Ϊ������
% iHf = 1./sHf;     %����
% iHf = iHf.*(abs(Hf)*gamma>1)+gamma*abs(sHf).*iHf.*(abs(sHf)*gamma<=1);
% restored_x = realifft2(iHf.*Yf));   %��ԭͼ��
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
