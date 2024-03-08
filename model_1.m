function varargout = model_1(varargin)
%MODEL_1 MATLAB code file for model_1.fig
%      MODEL_1, by itself, creates a new MODEL_1 or raises the existing
%      singleton*.
%
%      H = MODEL_1 returns the handle to a new MODEL_1 or the handle to
%      the existing singleton*.
%
%      MODEL_1('Property','Value',...) creates a new MODEL_1 using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to model_1_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      MODEL_1('CALLBACK') and MODEL_1('CALLBACK',hObject,...) call the
%      local function named CALLBACK in MODEL_1.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help model_1

% Last Modified by GUIDE v2.5 10-Nov-2022 00:27:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @model_1_OpeningFcn, ...
                   'gui_OutputFcn',  @model_1_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before model_1 is made visible.
function model_1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for model_1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes model_1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = model_1_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[imgfilename,imgpathname]=uigetfile({'*.jpg;*.png;*.bmp;*.tif'},'Select a image');
if imgfilename
    imgdata=imread([imgpathname '\' imgfilename]);
    axes(handles.p1);
    imshow(imgdata);
    %image(handles.p1,imgdata);
    handles.imgfilename=imgfilename;
    handles.imgdata=imgdata;
end
handles.outputimg = handles.imgdata;
guidata(hObject,handles);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close model_1;


% --- Executes on selection change in save.
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns save contents as cell array
%        contents{get(hObject,'Value')} returns selected item from save
v = get(handles.save,'value');
switch v
    case 1
        frame = getframe(handles.p1);
        img=frame2im(frame);
        [filename, pathname, FileIndex] = uiputfile('{*.jpg;*.png;*.tif;*.bmp}','Save file as');
        if FileIndex == 0  % ���ѡ���ˡ�cancel��
            return;
        else
            file_path = [pathname,filename];
            set(handles.edit12, 'String', file_path );
            imwrite(img,[pathname '/',filename]);
        end
    case 2
        %saveas(gcf,'Barchart.png');
        frame = getframe(handles.p2);
        img=frame2im(frame);
        [filename, pathname, FileIndex] = uiputfile('{*.jpg;*.png;*.tif;*.bmp}','Save file as');
        if FileIndex == 0  % ���ѡ���ˡ�cancel��
            return;
        else
            file_path = [pathname,filename];
            set(handles.edit12, 'String', file_path );
            %saveas(gcf,filename);
            imwrite(img,[pathname '/',filename]);
        end
    case 3
        frame = getframe(handles.p3);
        img=frame2im(frame);
        %imwrite(img,'extra.png');
        [filename, pathname, FileIndex] = uiputfile('{*.jpg;*.png;*.tif;*.bmp}','Save file as');
        if FileIndex == 0  % ���ѡ���ˡ�cancel��
            return;
        else
            file_path = [pathname,filename];
            set(handles.edit12, 'String', file_path );
            imwrite(img,[pathname  '/',filename]);
        end
        case 4
        frame = getframe(handles.p4);
        img=frame2im(frame);
        %imwrite(img,'extra.png');
        [filename, pathname, FileIndex] = uiputfile('{*.jpg;*.png;*.tif;*.bmp}','Save file as');
        if FileIndex == 0  % ���ѡ���ˡ�cancel��
            return;
        else
            file_path = [pathname,filename];
            set(handles.edit12, 'String', file_path );
            imwrite(img,[pathname  '/',filename]);
        end
end

% --- Executes on selection change in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns clear contents as cell array
%        contents{get(hObject,'Value')} returns selected item from clear
var = get(handles.clear,'value');
switch var
    case 1
        cla(handles.p1,'reset');
    case 2
        cla(handles.p2,'reset');
    case 3
        cla(handles.p3,'reset');
    case 4
        cla(handles.p4,'reset');
    case 5
        cla(handles.p1,'reset');
        cla(handles.p2,'reset');
        cla(handles.p3,'reset');
        cla(handles.p4,'reset');
end

% --- Executes during object creation, after setting all properties.
function clear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    num = size(I);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal��Ϊģ̬
%         errordlg('��ѡ��Ҷ�ͼ����иò���','����','modal');%ǰ�������ݣ�����������
%         return;
%     end
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal��Ϊģ̬
%         errordlg('��ѡ��Ҷ�ͼ����иò���','����','modal');%ǰ�������ݣ�����������
%         return;
        answer = questdlg('��Ҫ������Ӧ���ǻҶ�ͼ���Ƿ�����ѡ��?', 'Dessert Menu', '��','��','��');
        switch answer
        case '��'
            pushbutton1_Callback(hObject, eventdata, handles);
%             I = handles.imgdata;
            return;
        case '��'
            return;
        end
    end
    [m, n]=size(I);
    M=2*m; n=2*n;
    u=-m/2:m/2-1;
    v=-n/2:n/2-1;
    [U, V]=meshgrid(u, v);
    D=sqrt(U.^2+V.^2);
    D0=130;
    H=exp(-(D.^2)./(2*(D0^2)));
    N=0.01*ones(size(I,1), size(I,2));
%     N=0.01*ones(m, n);
    N=imnoise(N, 'gaussian', 0, 0.001);%��Ӹ�˹����
    J=fftfilter1(I, H)+N;
    HC=zeros(m, n);
    M1=H>0.1;           %Ƶ�ʷ�Χ
    HC(M1)=1./H(M1);
    K=fftfilter1(J, HC); %���˲�
    HC=zeros(m, n);
    M2=H>0.01;
    HC(M2)=1./H(M2);
    L=fftfilter1(J, HC);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    cla(handles.p4,'reset');
    axes(handles.p3);
    imshow(J);
    title('ͼ���˻�');
    axes(handles.p2);
    imshow(K);
    title('���˲�ʱƵ�ʷ�Χ�Ƚϴ�');
    axes(handles.p4);
    imshow(L);
    title('���˲�ʱƵ�ʷ�Χ�Ƚ�С');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%����ȴ�ͼ����͸ɾ���������ʲô�벨���ţ�����Ϊ���������͡�ѩ������
%����ȹ���С�˶�ѩ��״�Ϳ���
%��������������������С�����ڻ����ϣ���������ѩ��������Ӱ��ͼ���档
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
%     num = size(I);
% %     if numel(num)>2
% %         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal��Ϊģ̬
% %         errordlg('��ѡ��Ҷ�ͼ����иò���','����','modal');%ǰ�������ݣ�����������
% %         return;
% %     end
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal��Ϊģ̬
% %         errordlg('��ѡ��Ҷ�ͼ����иò���','����','modal');%ǰ�������ݣ�����������
% %         return;
%         answer = questdlg('��Ҫ������Ӧ���ǻҶ�ͼ���Ƿ�����ѡ��?', 'Dessert Menu', '��','��','��');
%         switch answer
%         case '��'
%             pushbutton1_Callback(hObject, eventdata, handles);
%             I = handles.imgdata;
%         case '��'
%             return;
%         end
%     end
    LEN=25;
    THETA=45;
    PSF=fspecial('motion', LEN, THETA);%��������չ����
    J=imfilter(I, PSF, 'conv', 'circular');%�����˶�ģ��
    noise_mean=0;
    NSR1 = 0;
    noise_var=0.0001;
    K=imnoise(J, 'gaussian', noise_mean, noise_var);%��Ӹ�˹����
    L1=deconvwnr(K, PSF, NSR1);%NSR�����Ϊ0ʱ�ĸ�ԭͼ��
%noise_var�������ܶ�
    NSR2=noise_var/var(I(:));%V = var(A) ���� A ���ش�С������ 1 �ĵ�һ������ά�ȵ�Ԫ�صķ��
    L2=deconvwnr(K, PSF, NSR2);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    cla(handles.p4,'reset');
    axes(handles.p3);
    imshow(J);
    title('�˻�ͼ��');
    axes(handles.p2);
    imshow(L1);
    title('NSR = 0');
    axes(handles.p4);
    imshow(L2);
    title('NSRΪ��ʵֵ');
end




% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
%     num = size(I);
%     if numel(num)>2
%         answer = questdlg('��Ҫ������Ӧ���ǻҶ�ͼ���Ƿ�����ѡ��?', 'Dessert Menu', '��','��','��');
%         switch answer
%         case '��'
%             pushbutton1_Callback(hObject, eventdata, handles);
%             I = handles.imgdata;
%         case '��'
%             return;
%         end
%     end
    M = get(handles.Aa,'string');
    if isempty(M)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal��Ϊģ̬
        errordlg('���ڿ�������noisepower����','����','modal');
        return;
    end
    Aa = str2double(M);%�������
    PSF=fspecial('gaussian', 10, 5);
    J=imfilter(I, PSF, 'conv');%ͼ���˻�
    v=0.02;
    K=imnoise(J, 'gaussian', 0, v);
    %�������
    NP=v*numel(I);
    [L, LAGRA]=deconvreg(K, PSF, NP);
%     LAGRA=deconvreg(K, PSF, NP);
    %�����������ӽ��и�ԭ
    edged=edgetaper(K, PSF);
    M1=deconvreg(edged, PSF, [], LAGRA*Aa);%�޸������������ӵõ���ͼ��
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    cla(handles.p4,'reset');
    axes(handles.p3);
    imshow(K);
    title('ͼ���˻�');
    axes(handles.p2);
    imshow(edged);
    title('ͼ���Ե');
    axes(handles.p4);
    imshow(M1);
    title('���������������Ӹ�ԭ');
%     figure;
%     imshow(L);
%     title('Ĭ�������������Ӹ�ԭ');
end





% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% I=handles.imgdata;
% cla(handles.p4);
% I=im2double(I);
% LEN=30;
% THETA=20;
% PSF=fspecial('motion', LEN, THETA);
% J=imfilter(I, PSF, 'circular', 'conv');
% axes(handles.p2);
% imshow(J);
% NUMIT = 300;%��������
% L=deconvlucy(J, PSF, NUMIT);
% %ͼ��ԭ
% axes(handles.p3);
% imshow(L);
% handles.outputimg = L;
% guidata(hObject,handles);

I=handles.imgdata;
I=im2double(I);
PSF=fspecial('gaussian', 7, 10);
v=0.0001;
J=imnoise(imfilter(I, PSF), 'gaussian',0, v);%�������

axes(handles.p3);
imshow(J);
title('�˻�ͼ��');
WT=zeros(size(I));
WT(5:end-4, 5:end-4)=1;
K=deconvlucy(J, PSF, 20, sqrt(v));%Ĭ��WT����
axes(handles.p2);
imshow(K);
title('Ĭ��WT����');
L=deconvlucy(J, PSF, 20, sqrt(v), WT);%������WT
axes(handles.p4);
imshow(L);
title('������WT����');

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
I=im2double(I);
PSF=fspecial('gaussian', 7, 10);
v=0.001;
J=imnoise(imfilter(I, PSF), 'gaussian',0, v);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
cla(handles.p4,'reset');
INITPSF=ones(size(PSF));%��ʼPSF��С
WT=zeros(size(I));
WT(5:end-4, 5:end-4)=1;
[K, PSF2]=deconvblind(J, INITPSF, 30,10*sqrt(v),WT);
axes(handles.p3);
imshow(J);
title('�˻�ͼ��');
axes(handles.p2);
imshow(K);
title('ä������ԭ');
handles.outputimg = K;
guidata(hObject,handles);



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
%     num = size(I);
%     if numel(num)>2
%         answer = questdlg('��Ҫ������Ӧ���ǻҶ�ͼ���Ƿ�����ѡ��?', 'Dessert Menu', '��','��','��');
%         switch answer
%         case '��'
%             pushbutton1_Callback(hObject, eventdata, handles);
%             I = handles.imgdata;
%         case '��'
%             return;
%         end
%     end
    LEN=25;
    THETA=45;
    PSF=fspecial('motion', LEN, THETA);
    J=imfilter(I, PSF, 'conv', 'circular');%�˶�ģ��
    noise=0.03*randn(size(I));
    K=imadd(J, noise);
    NP=abs(fft2(noise)).^2;
    % NPower=sum(NP(:))/prod(size(noise));
    NCORR=fftshift(real(ifft2(NP)));%�����������ϵ��
    IP=abs(fft2(I)).^2;
    % IPower=sum(IP(:))/prod(size(I));
    ICORR=fftshift(real(ifft2(IP)));%ͼ��������ϵ��
    L=deconvwnr(K, PSF, NCORR, ICORR);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    cla(handles.p4,'reset');
    axes(handles.p3);
    imshow(J);
    title('�˶�ģ��');
    axes(handles.p2);
    imshow(K);
    title('�˶�ģ��+����');
    axes(handles.p4);
    imshow(L);
    title('�����ϵ����ԭ');
end


function Aa_Callback(hObject, eventdata, handles)
% hObject    handle to Aa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Aa as text
%        str2double(get(hObject,'String')) returns contents of Aa as a double


% --- Executes during object creation, after setting all properties.
function Aa_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Aa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function pushbutton6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
