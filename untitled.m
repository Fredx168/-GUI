function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 16-Dec-2022 01:19:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
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

% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
if ~isempty(handles.imgfilename)
    I = handles.proj;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否转化?', 'Dessert Menu', '是','否','重新选择灰度图像','No thank you');
        switch answer
        case '是'
            I = rgb2gray(I);
        case '否'
            return;
        case '重新选择灰度图像'
            pushbutton1_Callback(hObject, eventdata, handles);
            I = handles.imgdata;
        end
    end
    cla(handles.p3,'reset');
    I = im2double(I);
%     I = imcrop(I,[100,100,1024,1024]);
    J = imnoise(I,'gaussian',0,0.03);
    K = wiener2(J,[5,5]);
    axes(handles.p3);
    imshow(K);
    title('自适应滤波');
end

% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
%     num = size(I);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
    I = handles.proj;
    cla(handles.p3,'reset');
    I = im2double(I);
    PSF = fspecial('average',3);
    J = imfilter(I,PSF);
    K = exp(imfilter(log(I),PSF));
    axes(handles.p3);
    imshow(J);%算术均值滤波
    title('算术均值滤波');
    figure;
    imshow(K);%几何均值滤波
    title('几何均值滤波');
end

% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
%     num = size(I);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
    I = handles.proj;
    cla(handles.p3,'reset');
    I = im2double(I);
    PSF = fspecial('average',3);
    Q1 = 1.6;
    Q2 = -1.6;
    J1 = imfilter(I.^(Q1+1),PSF);
    J2 = imfilter(I.^Q1,PSF);
    J = J1./J2;
    K1 = imfilter(I.^(Q2+1),PSF);
    K2 = imfilter(I.^Q2,PSF);
    K = K1./K2;
    axes(handles.p3);
    %n逆谐波均值滤波，Q为正
    imshow(J);
    title('逆谐波均值，Q值为正');
    figure;
    imshow(K);
    %n逆谐波均值滤波，Q为负
    title('逆谐波均值，Q值为负');
end
% --------------------------------------------------------------------
function Untitled_8_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.proj;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
        answer = questdlg('需要的输入应该是灰度图像，是否转化?', 'Dessert Menu', '是','否','重新选择灰度图像','No thank you');
        switch answer
        case '是'
            I = rgb2gray(I);
        case '否'
            return;
        case '重新选择灰度图像'
            pushbutton1_Callback(hObject, eventdata, handles);
            I = handles.imgdata;
        end
    end
    cla(handles.p3,'reset');
    I = im2double(I);
    J = medfilt2(I,[3,3]);
    axes(handles.p3);
    imshow(J);
    title('二维中值滤波');
end


% --------------------------------------------------------------------
function Untitled_9_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_9 (see GCBO)
if ~isempty(handles.imgfilename)
    I = handles.proj;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否转化?', 'Dessert Menu', '是','否','重新选择灰度图像','No thank you');
        switch answer
        case '是'
            I = rgb2gray(I);
        case '否'
            return;
        case '重新选择灰度图像'
            pushbutton1_Callback(hObject, eventdata, handles);
            I = handles.imgdata;
        end
    end
    cla(handles.p3,'reset');
    I = im2double(I);
    domain = [1 1 1 1;1 1 1 1;1 1 1 1;1 1 1 1];
    J = ordfilt2(I, 8, domain);
    axes(handles.p3);
    imshow(J);
    title('二维排序滤波');
end
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_10_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_10 (see GCBO)
if ~isempty(handles.imgfilename)
    I = handles.proj;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否转化?', 'Dessert Menu', '是','否','重新选择灰度图像','No thank you');
        switch answer
        case '是'
            I = rgb2gray(I);
        case '否'
            return;
        case '重新选择灰度图像'
            pushbutton1_Callback(hObject, eventdata, handles);
            I = handles.imgdata;
        end
    end
    cla(handles.p3,'reset');
    I = im2double(I);
    J = ordfilt2(I,1,ones(4,4));
    K = ordfilt2(I,9,ones(3));
    axes(handles.p3);
    imshow(J);
    title('最大值滤波');
    figure;
    imshow(K);
    title('最小值滤波');
end
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_11_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_12_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_12 (see GCBO)
I = handles.imgdata;
I = im2double(I);
K = imnoise(I,'salt & pepper',0.03);
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(K);
title('椒盐噪声');
handles.proj = K;
guidata(hObject,handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_13_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_13 (see GCBO)
I = handles.imgdata;
I = im2double(I);
R = rand(size(I));
J = I;
J(R<=0.03)=0;
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(J);
title('椒噪声');
handles.proj = J;
guidata(hObject,handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_14_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_14 (see GCBO)
I = handles.imgdata;
I = im2double(I);
R = rand(size(I));
J = I;
J(R<=0.03)=1;
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(J);
title('盐噪声');
handles.proj = J;
guidata(hObject,handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_15_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_15 (see GCBO)
I = handles.imgdata;
I = im2double(I);
J = imnoise(I,'poisson');
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(J);
title('泊松噪声');
handles.proj = J;
guidata(hObject,handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_16_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_16 (see GCBO)
I = handles.imgdata;
I = im2double(I);
J = imnoise(I,'speckle');
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(J);
title('乘性噪声');
handles.proj = J;
guidata(hObject,handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_17_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_17 (see GCBO)
I = handles.imgdata;
I = im2double(I);
[m,n] = size(I);
a = 50;
b = 180;
J = a+(b-a)*rand(m,n);
num = size(I);
if numel(num)>2
    answer = questdlg('需要的输入应该是灰度图像，是否转化?', 'Dessert Menu', '是','否','重新选择灰度图像','No thank you');
    switch answer
        case '是'
            I = rgb2gray(I);
        case '否'
            return;
        case '重新选择灰度图像'
            pushbutton1_Callback(hObject, eventdata, handles);
            I = handles.imgdata;
    end
end
J =double(I)+J;
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(uint8(J));
title('均匀分布噪声');
handles.proj = uint8(J);
guidata(hObject,handles);
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function p1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to p1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate p1


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
        if FileIndex == 0  % 如果选择了‘cancel’
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
        if FileIndex == 0  % 如果选择了‘cancel’
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
        if FileIndex == 0  % 如果选择了‘cancel’
            return;
        else
            file_path = [pathname,filename];
            set(handles.edit12, 'String', file_path );
            imwrite(img,[pathname  '/',filename]);
        end
end

% --- Executes during object creation, after setting all properties.
function save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p1,'reset');
cla(handles.p2,'reset');
cla(handles.p3,'reset');


% --------------------------------------------------------------------
function Untitled_19_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
I = im2double(I);
J = imnoise(I,'gaussian',0,0.01);
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(J);
title('高斯噪声');
handles.proj = J;
guidata(hObject,handles);
