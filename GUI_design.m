function varargout = GUI_design(varargin)
% GUI_DESIGN MATLAB code for GUI_design.fig
%      GUI_DESIGN, by itself, creates a new GUI_DESIGN or raises the existing
%      singleton*.
%
%      H = GUI_DESIGN returns the handle to a new GUI_DESIGN or the handle to
%      the existing singleton*.
%
%      GUI_DESIGN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_DESIGN.M with the given input arguments.
%
%      GUI_DESIGN('Property','Value',...) creates a new GUI_DESIGN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_design_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_design_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_design

% Last Modified by GUIDE v2.5 16-Dec-2022 13:15:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
'gui_Singleton',  gui_Singleton, ...
'gui_OpeningFcn', @GUI_design_OpeningFcn, ...
'gui_OutputFcn',  @GUI_design_OutputFcn, ...
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


% --- Executes just before GUI_design is made visible.
function GUI_design_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_design (see VARARGIN)

% Choose default command line output for GUI_design
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_design wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_design_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)%导入图片
% hObject    handle to start (see GCBO)
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


% --- Executes on selection change in translate.
function translate_Callback(hObject, eventdata, handles)
% hObject    handle to translate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns translate contents as cell array
%        contents{get(hObject,'Value')} returns selected item from translate
var = get(handles.translate,'value');
i = handles.imgdata;
switch var
case 1
se = translate(strel(1),[-100,0]);
b = imdilate(i,se);
% b = move1(i1,-50,0);

case 2
se = translate(strel(1),[100,0]);
b = imdilate(i,se);
% b = move1(i1,100,1);
case 3
se = translate(strel(1),[0,100]);
b = imdilate(i,se);
%b = move1(i1,1,100);
case 4
se = translate(strel(1),[0,-100]);
b = imdilate(i,se);
%b = move1(i1,1,-100);
end
%image(handles.p2,b);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(b);
handles.outputimg = b;
guidata(hObject,handles);
% --- Executes during object creation, after setting all properties.


function translate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to translate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in mirror.
function mirror_Callback(hObject, eventdata, handles)
% hObject    handle to mirror (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns mirror contents as cell array
%        contents{get(hObject,'Value')} returns selected item from mirror
I = handles.imgdata;
[Height,Width]=size(I); 
T1=affine2d([-1 0 0;0 1 0;Width 0 1]);  %构造空间变换结构T1，这里为水平镜像变换矩阵
T2=affine2d([1 0 0;0 -1 0;0 Height 1]); %构造空间变换结构T2，这里为竖直镜像变换矩阵
var = get(handles.mirror,'value');
switch var
case 1
A=imwarp(I,T1);     %对原图像I进行水平镜像变换 
case 2
A=imwarp(I,T2);      %对原图像I进行竖直镜像变换
end
%image(handles.p2,A);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(A);
handles.outputimg = A;
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function mirror_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mirror (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function scale_Callback(hObject, eventdata, handles)
% hObject    handle to scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
img = handles.imgdata;
times = get(handles.scale,'value');
set(handles.times,'string',num2str(times));
change_i=imresize(img,times,'nearest'); %最近邻插值
image(handles.p2,change_i);
%axes(handles.p2);
%imshow(change_i);
handles.outputimg = change_i;
guidata(hObject,handles);
% --- Executes during object creation, after setting all properties.


function scale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function rotate_Callback(hObject, eventdata, handles)
% hObject    handle to rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
cla(handles.p3,'reset');
im = handles.imgdata;
x = get(handles.rotate,'value');
im2 = imrotate(im,x);
% image(handles.p2,im2);
axes(handles.p2);
imshow(im2);
handles.outputimg = im2;
set(handles.angle,'string',num2str(x));
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function rotate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function times_Callback(hObject, eventdata, handles)
% hObject    handle to times (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of times as text
%        str2double(get(hObject,'String')) returns contents of times as a double


% --- Executes during object creation, after setting all properties.
function times_CreateFcn(hObject, eventdata, handles)
% hObject    handle to times (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end



function angle_Callback(hObject, eventdata, handles)
% hObject    handle to angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of angle as text
%        str2double(get(hObject,'String')) returns contents of angle as a double


% --- Executes during object creation, after setting all properties.
function angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in transpose.
function transpose_Callback(hObject, eventdata, handles)
% hObject    handle to transpose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p3,'reset');
cla(handles.p2,'reset');
img = handles.imgdata;
T=affine2d([0 1 0;1 0 0;0 0 1]);%构造空间变换结构T.这里为转置变换矩阵
dst=imwarp(img,T);                %对原图像I进行转置变换
image(handles.p2,dst);
handles.outputimg = dst;
guidata(hObject,handles);


% --- Executes on button press in cut.
function cut_Callback(hObject, eventdata, handles)
% hObject    handle to cut (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
A=handles.imgdata;
% rect = [20,20,150,150];
questdlg('请在图二选择矩形截取区域并双击','图像截取提示','确定','确定');
%B = imcrop(A,rect);
[B,rect]=imcrop(A);
% h3 = msgbox('请在图二选择矩形截取区域', '图像截取提示', 'help');
rectangle('Position',rect,'LineWidth',2,'EdgeColor','b');
pause(1);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(B);
% image(handles.p2,B);
handles.outputimg = B;
guidata(hObject,handles);


% --- Executes on selection change in linear_gray.
function linear_gray_Callback(hObject, eventdata, handles)
% hObject    handle to linear_gray (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% if isempty(get(handles.gamma,'string'))
% % mode2=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
% errordlg('请先在<线性度取值>内输入线性度','警告','modal');%前者是内容，后者是名称
% %     errordlg('请在对应位置输入线性度取值','警告','modal');%前者是内容，后者是名称
% return;
% end
var = get(handles.linear_gray,'value');
num = size(handles.imgdata);
if numel(num)<=2
% mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
errordlg('请选择彩色图像进行该操作','警告','modal');%前者是内容，后者是名称
return;
%     errordlg('请选择彩色图像进行该操作','警告','modal');%前者是内容，后者是名称
end
gamma = get(handles.gamma,'string');
gamma = str2double(gamma);

% if numel(num)<=2
% % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
% errordlg('请选择彩色图像进行该操作','警告','modal');%前者是内容，后者是名称
% return;
% %     errordlg('请选择彩色图像进行该操作','警告','modal');%前者是内容，后者是名称
% end
if isempty(get(handles.gamma,'string'))
% mode2=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
errordlg('请先在<线性度取值>内输入线性度','警告','modal');%前者是内容，后者是名称
%     errordlg('请在对应位置输入线性度取值','警告','modal');%前者是内容，后者是名称
return;
end
switch var
case 2
i1 = handles.imgdata;
r = i1;
r(:,:,2) = 0;
r(:,:,3) = 0;
%         num = size(r);
%         if numel(num)>2
%             r = rgb2gray(r);
%         end
r1 = imadjust(r,[0.5,0.8],[0,1],gamma);
%image(handles.p2,r1);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(r1);
handles.outputimg = r1;
case 3
i1 = handles.imgdata;
g = i1;
g(:,:,1) = 0;
g(:,:,3) = 0;
%         num = size(g);
%         if numel(num)>2
%             g = rgb2gray(g);
%         end
g1 = imadjust(g,[0,0.3],[0,1],gamma);
%image(handles.p2,g1);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(g1);
handles.outputimg = g1;
case 4
i1 = handles.imgdata;
b = i1;
b(:,:,1) = 0;
b(:,:,2) = 0;
%         num = size(b);
%         if numel(num)>2
%             b = rgb2gray(b);
%         end
b1 = imadjust(b,[0,0.3],[0,1],gamma);
%image(handles.p2,b1);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(b1);
handles.outputimg = b1;
case 5
i1 = handles.imgdata;
r = i1;
r(:,:,2) = 0;
r(:,:,3) = 0;
r1 = imadjust(r,[0.5,0.8],[0,1],gamma);
g = i1;
g(:,:,1) = 0;
g(:,:,3) = 0;
g1 = imadjust(g,[0,0.3],[0,1],gamma);
b = i1;
b(:,:,1) = 0;
b(:,:,2) = 0;
b1 = imadjust(b,[0,0.3],[0,1],gamma);
i2 = r1+g1+b1;
%image(handles.p2,i2);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(i2);
handles.outputimg = i2;
end
guidata(hObject,handles);

% Hints: contents = cellstr(get(hObject,'String')) returns linear_gray contents as cell array
%        contents{get(hObject,'Value')} returns selected item from linear_gray


% --- Executes during object creation, after setting all properties.
function linear_gray_CreateFcn(hObject, eventdata, handles)
% hObject    handle to linear_gray (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end



function gamma_Callback(hObject, eventdata, handles)
% hObject    handle to gamma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of gamma as text
%        str2double(get(hObject,'String')) returns contents of gamma as a double


% --- Executes during object creation, after setting all properties.
function gamma_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gamma (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in piecewise_linear_gray.
function piecewise_linear_gray_Callback(hObject, eventdata, handles)
% hObject    handle to piecewise_linear_gray (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a = get(handles.parameter,'value');
m=a*55;
n=(a+0.13)*65+14;
%J=rgb2gray(handles.imgdata); 
J = handles.imgdata;
num = size(J);
if numel(num)>2
J = rgb2gray(J);
end
[M,N]=size(J);         
%x=1;y=1;               
for x=1:M
for y=1:N
if (J(x,y)<=m);     
H(x,y)=J(x,y)*10;
elseif(J(x,y)>m&&J(x,y)<=n);
H(x,y)=(10/7)*(J(x,y)-5)+50;
else if(J(x,y)>n);
H(x,y)=(105/180)*(J(x,y)-75)+150;
end
end
end
end
%image(handles.p2,H);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(H);
handles.outputimg=H;
guidata(hObject,handles);


% --- Executes on slider movement.
function parameter_Callback(hObject, eventdata, handles)
% hObject    handle to parameter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function parameter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to parameter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in notlinear1.


% --- Executes on button press in imgdata1.
function imgdata1_Callback(hObject, eventdata, handles)
% hObject    handle to imgdata1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[imgfilename1,imgpathname1]=uigetfile({'*.jpg;*.png;*.bmp;*.tif'},'Select a image');
if imgfilename1
imgdata1=imread([imgpathname1 '\' imgfilename1]);
axes(handles.p3);
imshow(imgdata1);
handles.imgfilename1=imgfilename1;
handles.imgdata1=imgdata1;
end
guidata(hObject,handles);


% --- Executes on selection change in algebra.
function algebra_Callback(hObject, eventdata, handles)
% hObject    handle to algebra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns algebra contents as cell array
%        contents{get(hObject,'Value')} returns selected item from algebra
var = get(handles.algebra,'value');
i1 = handles.imgdata;
i2 = handles.imgdata1;
num1 = size(i1);
num2 = size(i2);
switch var
    case 1
        return;
case 2
if num1(1)~=num2(1)||numel(num1)~=numel(num1)
% mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
errordlg('图一与图三的大小和类型不同，无法操作','警告','modal');%前者是内容，后者是名称
return;
end
k = imadd(i1,i2);
%k = imlincomb(0.5,i1,0.5,i2);
%图像叠加
case 3
%亮度调节
x = get(handles.light,'value');
if x==0
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('请在<亮度调节>滑动条选择亮度系数(0-2]','警告','modal');
    return;
end
k = immultiply(i1,x);
case 4
RGB=i1 ;          %读入eight图像，赋值给RGB
% A=imnoise(RGB,'gaussian',0,0.05);    %加入高斯白噪声
I=i1;                                %将A赋值给I
M=get(handles.frequency,'string');
if isempty(M)
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('请在<叠加次数>设置叠加次数(大于0的整数)','警告','modal');
    return;
end
M = round(str2double(M));
%设置叠叠加次数M
I=im2double(I);                     %将I数据类型转换成双精度
RGB=im2double(RGB);
for i=1:M
I=imadd(I,RGB);                  %对用原图像与带噪声图像进行多次叠加，结果返回给I
end
avg_A=I/(M+1);                      %求叠加的平均图像 
k = avg_A;
case 5
%dsa减影
a = i1;
b = i2;
as = size(a);
bs = size(b);
if  as(1)~=bs(1)||numel(as)~=numel(bs)
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('图一和图三的大小和类型应相同','警告','modal');%前者是内容，后者是名称
    return;
end
k = imsubtract(a,b);
% k = 255-k;
case 6
A=i1;        %读取图像tire，并赋值给A
% [m,n]=size(A);               %获取图像矩阵A的行列数m，n
B=i2;      %读取图像eight的值，并赋值给B
% C=B;     
%初始化矩阵C
as = size(A);
bs = size(B);
if as(1)~=bs(1)||numel(as)~=numel(bs)
    errordlg('图一应为混合图像，图三为其中一张图像,且格式、大小相同','警告','modal');%前者是内容，后者是名称
    return;
end
k = imsubtract(A,B);
disp(as(1));
case 7
A=i1;%读入原始图像赋值给A和B
B=i2;
as = size(A);
bs = size(B);
if as(1)~=bs(1)||numel(as)~=numel(bs);
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('图一与图三的类型与尺寸应该相同','警告','modal');%前者是内容，后者是名称
    return;
end
C=immultiply(A,B);
k = C;
case 8
A=i1;				%读入图像office_1和office_2，并赋值
B=i2;   
as = size(A);
bs = size(B);
if as(1)~=bs(1)||numel(as)~=numel(bs)
    errordlg('图一与图三的大小与类型应该相同','警告','modal');%前者是内容，后者是名称
    return;
end
k=imdivide(A,B);  
end
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(k);
handles.outputimg = k;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function algebra_CreateFcn(hObject, eventdata, handles)
% hObject    handle to algebra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in logic.
function logic_Callback(hObject, eventdata, handles)
% hObject    handle to logic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns logic contents as cell array
%        contents{get(hObject,'Value')} returns selected item from logic
I=handles.imgdata;%读入图像，赋值给I和J
J=handles.imgdata1;
is = size(I);
js = size(J);
if numel(is)==numel(js)&&is(1)==js(1)
I1=im2bw(I);                    %转化为二值图像
J1=im2bw(J);
K1=I1 & J1;                     %实现图像的逻辑“与”运算
K2=I1 | J1;                     %实现图像的逻辑“或”运算
K3=~I1;                         %实现逻辑“非”运算
K4=xor(I1,J1);                  %实现“异或”运算
H=~(I1|J1);%或非
G=~(I1&J1);%与非
var = get(handles.logic,'value');
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
switch var
case 2
imshow(K1);%与
handles.outputimg = K1;
case 3
imshow(K2);%或
handles.outputimg = K2;
case 4
imshow(K3);%非
handles.outputimg = K3;
case 5
imshow(K4);%异或
handles.outputimg = K4;
case 6
imshow(H);%或非
handles.outputimg = H;
case 7
imshow(G);%与非
handles.outputimg = G;
end
else
mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
errordlg('X and Y must have the same numel','警告',mode);%前者是内容，后者是名称
end
% --- Executes during object creation, after setting all properties.
function logic_CreateFcn(hObject, eventdata, handles)
% hObject    handle to logic (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function light_Callback(hObject, eventdata, handles)
% hObject    handle to light (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function light_CreateFcn(hObject, eventdata, handles)
% hObject    handle to light (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function frequency_Callback(hObject, eventdata, handles)
% hObject    handle to frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frequency as text
%        str2double(get(hObject,'String')) returns contents of frequency as a double


% --- Executes during object creation, after setting all properties.
function frequency_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frequency (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in shear.
function shear_Callback(hObject, eventdata, handles)
% hObject    handle to shear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I=handles.imgdata;  %读入图像
Td = maketform('affine',[1 4 0; 2 1 0; 0 0 1]');% 创建图像整体切变的参数结构体
Id = imtransform(I,Td,'FillValues', 255);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(Id);
handles.outputimg = Id;
guidata(hObject,handles);
%实现图像整体切变


% --- Executes on button press in func_nlfilter.
function func_nlfilter_Callback(hObject, eventdata, handles)
% hObject    handle to func_nlfilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    bulk = get(handles.bulk,'string');
    if isempty(bulk)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<邻域块大小>设置邻域块大小，推荐3或者5','警告','modal');
        return;
    end
    bulk = round(str2double(bulk));
    imgoutput1=nlfilter(imgdata,[bulk bulk],'std2');%方差运算
    fun = @(x) max(x(:));
    imgoutput2=nlfilter(imgdata,[bulk bulk],fun);%操作函数
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(imgoutput1);
    title('方差运算');
    cla(handles.p3,'reset');
    axes(handles.p3);
    imshow(imgoutput2);
    title('最大值运算');
    guidata(hObject,handles);
end


function bulk_Callback(hObject, eventdata, handles)
% hObject    handle to bulk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bulk as text
%        str2double(get(hObject,'String')) returns contents of bulk as a double


% --- Executes during object creation, after setting all properties.
function bulk_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bulk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in func_colfilt.
function func_colfilt_Callback(hObject, eventdata, handles)
% hObject    handle to func_colfilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    bulk = get(handles.bulk,'string');
    bulk = round(str2double(bulk));
    y = bulk;
    I1=handles.imgdata;           %输入图像
    %imgdata = rgb2gray(imgdata);
    num = size(I1);
    if numel(num)>2
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
    return;
    end
    I1 = im2double(I1);
    f=@(x) ones(y*y,1)*min(x); 
    %f=@(x) min(x);
    %I2 =colfilt(I1,[4 4],'sliding',f); %
    I3=colfilt(I1,[y y],'distinct',f);    %按照滑动邻域方式 进行对图像进行最小值邻域操作
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(I3);
    title('colfilt最小值邻域操作');
    handles.outputimg = I3;
    guidata(hObject,handles);
end
% handles    structure with handles and user data (see GUIDATA)





% --- Executes during object creation, after setting all properties.
function func_colfilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to func_colfilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in func_roipoly.
function func_roipoly_Callback(hObject, eventdata, handles)
% hObject    handle to func_roipoly (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    imgdata = im2double(imgdata);
    %c = [];%设置探索区域的向量参数
    %r = datad;
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    questdlg('请在图二选择若干点构成闭合的图形以创建ROI区域，并在闭合点上进行双击','图像截取提示','确定','确定');
    imgoutput=roipoly(imgdata);
    axes(handles.p2);
    imshow(imgoutput);
    title('ROI区域');
    handles.outputimg=imgoutput;
    guidata(hObject,handles);
end



% --- Executes on button press in func_roicolor.
function func_roicolor_Callback(hObject, eventdata, handles)
% hObject    handle to func_roicolor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%low = 20;
%high = 50;
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    low = get(handles.edit14,'string');
    high = get(handles.edit15,'string');
    if isempty(low)||isempty(high)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<灰度阈值>设置灰度的区间,不能为空','警告','modal');
        return;
    end
    low = round(str2double(low));
    high = round(str2double(high));
    imgoutput = roicolor(imgdata,low,high);%设置像素颜色映射范围
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(imgoutput);
    title('roifilt区域填充');
    handles.outputimg=imgoutput;
    guidata(hObject,handles);
end




% --- Executes on button press in func_roifill.
function func_roifill_Callback(hObject, eventdata, handles)
% hObject    handle to func_roifill (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    %imgdata = rgb2gray(imgdata);
    num = size(imgdata);
    if numel(num)>2
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
    return;
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    questdlg('请在图二用点绘制滤波区域，在闭合点时进行双击得到结果','图像截取提示','确定','确定');
    rp = roipoly(imgdata);
    imgoutput = regionfill(imgdata,rp);
    axes(handles.p2);
    imshow(imgoutput);
    title('roifilt区域填充');
    handles.outputimg=imgoutput;
    guidata(hObject,handles);
end


% --- Executes on button press in func_roifilt2.
function func_roifilt2_Callback(hObject, eventdata, handles)
% hObject    handle to func_roifilt2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    %imgdata = rgb2gray(imgdata);
    num = size(imgdata);
    
    if numel(num)>2
    % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
    errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
    return;
    end
    questdlg('请在图二用点绘制滤波区域，在闭合点时进行双击得到结果','图像截取提示','确定','确定');
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    rp = roipoly(imgdata);
    %imgdata = roifill(imgdata,rp);
    fun = fspecial('motion',20,45);
    imgoutput = roifilt2(fun,imgdata,rp);
    
    axes(handles.p2);
    imshow(imgoutput);
    title('roifilt2区域滤波');
    handles.outputimg=imgoutput;
    guidata(hObject,handles);
end



% --- Executes on button press in clear_original.
function clear_original_Callback(hObject, eventdata, handles)
% hObject    handle to clear_original (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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
cla(handles.p1,'reset');
cla(handles.p2,'reset');
cla(handles.p3,'reset');
end

% --- Executes during object creation, after setting all properties.
function clear_CreateFcn(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


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
frame = getframe(handles.p2);
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
[filename, pathname, FileIndex] = uiputfile('{*.jpg;*.png;*.tif;*.bmp}','Save file as');
if FileIndex == 0  % 如果选择了‘cancel’
return;
else
file_path = [pathname,filename];
set(handles.edit12, 'String', file_path );
%saveas(gcf,filename);
saveas(gcf,[pathname  '/',filename]);
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


% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close all;



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


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in blockproc.
function blockproc_Callback(hObject, eventdata, handles)
% hObject    handle to blockproc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    bulk = get(handles.bulk,'string');
    if isempty(bulk)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<邻域块大小>设置邻域块大小，推荐3或者5','警告','modal');
        return;
    end
    bulk = round(str2double(bulk));
    x = bulk;
    fun=@(block_struct) std2(block_struct.data); %创建图像块处理的函数句柄
    imgoutput=uint8(blockproc(imgdata,[x,x],fun));
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(imgoutput);
    title('blockproc图像的分离块操作');
    guidata(hObject,handles);
end





% --- Executes during object creation, after setting all properties.
function blockproc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to blockproc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --------------------------------------------------------------------

function enhancement_Callback(hObject, eventdata, handles)
% hObject    handle to enhancement (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%run('test1'); 
%set(GUI_design,'visible','off')

% --------------------------------------------------------------------
function hollow_Callback(hObject, eventdata, handles)
% hObject    handle to hollow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function linear_Callback(hObject, eventdata, handles)
% hObject    handle to linear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_8_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_10_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I= handles.imgdata;
row=size(I,1);
column=size(I,2);
N=zeros(1, 256);
for i=1:row
for j=1:column
k=I(i, j);
N(k+1)=N(k+1)+1;
end
end
cla(handles.p2,'reset');
cla(handles.p2,'reset');
axes(handles.p2);     
bar(N,'blue');
% N = imhist(I);
% cla(handles.p2,'reset');
% cla(handles.p2,'reset');
% axes(handles.p2);
% imshow(handles.p2);
% handles.outputimg = N;
% guidata(hObject,handles);
% --------------------------------------------------------------------
function Untitled_14_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%image(handles.p2,g1);
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    gamma = get(handles.gamma,'string');
    if isempty(gamma)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<线性度取值>设置合适的gamma参数,建议数值：100','警告','modal');
        return;
    end
    gamma = str2double(gamma);
    I=double(imgdata);
    J=(I-gamma)*255/(gamma-15);
    row=size(I,1);
    column=size(I,2);
    for i=1:row
        for j=1:column
            if J(i, j)<0
                J(i, j)=0;
            end
            if J(i, j)>255
                J(i, j)=255;
            end
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(J));
    handles.outputimg = uint8(J);
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_15_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%image(handles.p2,b1);
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    gamma = get(handles.gamma,'string');
    if isempty(gamma)||(str2double(gamma)<0)||(str2double(gamma)>0.65)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<线性度取值>设置合适的gamma参数,建议数值:[0,0.65]','警告','modal');
        return;
    end
    gamma = str2double(gamma);
    J=imadjust(imgdata, [gamma (gamma+0.35)], [0 1]);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(J));
    handles.outputimg = uint8(J);
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_16_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    gamma = get(handles.gamma,'string');
    if isempty(gamma)||(str2double(gamma)<0)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<线性度取值>设置合适的gamma参数,建议数值:0.4','警告','modal');
        return;
    end
    gamma = str2double(gamma);
    J=imadjust(imgdata, [0.1 0.5], [0, 1], gamma);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(J));
    handles.outputimg = uint8(J);
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_17_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
    num = size(imgdata);
    if numel(num)<=2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择彩色图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    gamma = get(handles.gamma,'string');
    if isempty(gamma)||(str2double(gamma)<=0)||(str2double(gamma)>0.41)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<线性度取值>设置合适的gamma（>0）参数,范围(0,0,41]','警告','modal');
        return;
    end
    gamma = str2double(gamma);
    J=imadjust(imgdata, [gamma 1.2*gamma 0; gamma*2 gamma*2.4 1], []);%
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(J));
    handles.outputimg = uint8(J);
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_18_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
%     I = imgdata;
%     num = size(imgdata);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
    gamma = get(handles.gamma,'string');
    if isempty(gamma)||(str2double(gamma)>1)||(str2double(gamma)<-1)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<线性度取值>设置合适的gamma参数,建议数值:(-1,1)','警告','modal');
        return;
    end
    gamma = str2double(gamma);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
%     imshow(imgdata);
    figure;
    imshow(handles.imgdata);
    title('亮度改变之后');
    brighten(gamma);
%     axes(handles.p2);
%     imshow(newi);
%     image(newi);
    
    
end
% --------------------------------------------------------------------
function Untitled_19_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
%     num = size(imgdata);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
    M=stretchlim(imgdata);
    J=imadjust(imgdata, M, [ ]);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(J)); 
    handles.outputimg = uint8(J);
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_20_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    imgdata = handles.imgdata;
%     num = size(imgdata);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
    J=imcomplement(imgdata);	
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(J)); 
    title('图像的反转变换');
    handles.outputimg = uint8(J);
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    J=imnoise(I, 'salt & pepper', 0.03);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    K=medfilt2(J);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(J);
    title('添加密度为0.03的高斯噪声');
    axes(handles.p3);
    imshow(K);
    title('medfilt2中值滤波');
    handles.outputimg= K;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    J=ordfilt2(I, 1, true(5));
    K=ordfilt2(I, 25, true(5));
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(J);
    title('排序后第一个作为输出的排序滤波结果');
    axes(handles.p3);
    imshow(K);
    title('排序后第一个作为输出的排序滤波结果');
    handles.outputimg= K;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_6_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    J=imnoise(I, 'gaussian', 0, 0.01);
    K=wiener2(J, [5 5]);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(J);
    title('添加密度为0.01的高斯噪声');
    axes(handles.p3);
    imshow(K);
    title('wiener2滤波结果');
    handles.outputimg= K;
    guidata(hObject,handles);
end


% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    h=[0,1,0; 1, -4, 1; 0, 1, 0];
    J=conv2(I, h, 'same');
    K=I-J;
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(K);
    title('拉普拉斯算子进行锐化滤波');
    handles.outputimg= K;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    J=imnoise(I, 'salt & pepper', 0.02);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    h=ones(3,3)/5;
    h(1,1)=0;   h(1,3)=0;
    h(3,1)=0;   h(1,3)=0;
    K=imfilter(J, h);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(J);
    title('添加密度为0.02的高斯噪声');
    axes(handles.p3);
    imshow(K);
    title('imfilter平滑');
    handles.outputimg= K;
    guidata(hObject,handles);
end


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    J=imnoise(I, 'gaussian', 0, 0.01);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    h=ones(3,3)/9;
    K=conv2(J, h);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(J);
    title('添加密度为0.01的高斯噪声');
    axes(handles.p3);
    imshow(K);
    title('conv2平滑');
    handles.outputimg= K;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    I=im2double(I);
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    bulk = get(handles.bulk,'string');
    if isempty(bulk)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<邻域块大小>设置邻域块大小，推荐3或者5','警告','modal');
        return;
    end
    bulk = round(str2double(bulk));
    I=im2double(I);
    J=imnoise(I, 'salt & pepper', 0.02);
    h1=fspecial('average', bulk);
    K=filter2(h1, J);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(J);
    title('添加密度为0.02的椒盐噪声');
    axes(handles.p3);
    imshow(K);
    title('fspecial + filter平滑');
    handles.outputimg= K;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_21_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function sum_Callback(hObject, eventdata, handles)
% hObject    handle to sum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function ave_Callback(hObject, eventdata, handles)
% hObject    handle to ave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)<=2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择彩色图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    clc;
    J = rgb2gray(I);
    gray = mean2(J);
    s1 = sprintf('彩图转灰度图像后的均值：%.2f\n',gray);
    disp(s1);
    rgb = mean2(I);
    s2 = sprintf('彩色图像的均值：%.2f\n',rgb);
    disp(s2);
    r = mean2(I(:,:,1));
    s3 = sprintf('红色的均值：%.2f\n',r);
    disp(s3);
    g = mean2(I(:,:,2));
    s4 = sprintf('红色的均值：%.2f\n',g);
    disp(s4);
    b = mean2(I(:,:,3));
    s5 = sprintf('红色的均值：%.2f\n',b);
    disp(s5);
end

% --------------------------------------------------------------------
function s_Callback(hObject, eventdata, handles)
% hObject    handle to s (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    clc;
    cla(handles.p2,'reset');
    cla(handles.p3,'reset');
    s1 = std2(I);
    g1 = sprintf('原图的标准差为：%.2f\n',s1);
    disp(g1);
    J = histeq(I);
    axes(handles.p2);
    imshow(J);
    title('图像histeq均衡化后');
    s2 = std2(J);
    g2 = sprintf('原图在均衡化后的标准差为：%.2f\n',s2);
    disp(g2);
    handles.outputimg = J;
end

% --------------------------------------------------------------------
function r_Callback(hObject, eventdata, handles)
% hObject    handle to r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    clc;
    cla(handles.p2,'reset');
    cla(handles.p3,'reset');
    J = medfilt2(I);
    r = corr2(I,J);
    axes(handles.p2);
    imshow(J);
    title('原图中值滤波后');
    g2 = sprintf('原图与中值滤波处理后的图片的相关系数为：%.4f\n',r);
    disp(g2);
    handles.outputimg = J;
end

% --------------------------------------------------------------------
function h_Callback(hObject, eventdata, handles)
% hObject    handle to h (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    clc;
    cla(handles.p2,'reset');
    cla(handles.p3,'reset');
    J = I;
    axes(handles.p2);
    imcontour(J,3);
    title('原图的等高线');
    handles.outputimg = J;
end

% --------------------------------------------------------------------


function Untitled_35_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_35 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
run('untitled.m');

% --------------------------------------------------------------------
function Untitled_36_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_36 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
run('model_1');
%D:\matlab_file\bin\GUI_files\


% --------------------------------------------------------------------
function Untitled_38_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_38 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_41_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_41 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_42_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_42 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_43_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_43 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_44_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_44 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_45_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_45 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    Image = handles.imgdata;
    [height,width,color]=size(Image);
    NewImage=zeros(height,width,color);
    sigma=1.414;  filtersize=[7 7];
    lowfilter=fspecial('gaussian',filtersize,sigma); 
    highfilter=zeros(filtersize);
    highpara=1; lowpara=0.6;            %控制滤波器幅度范围的系数  
    highfilter(ceil(filtersize(1,1)/2),ceil(filtersize(1,2)/2))=1;
    highfilter=highpara*highfilter-(highpara-lowpara)*lowfilter; 
    for i=1:color
        logI=log(double(Image(:,:,i)+1)); 
        highpart=imfilter(logI,highfilter,'replicate','conv'); 
        temp=exp(highpart);
        top=max(temp(:)); bottom=min(temp(:));
        temp=(temp-bottom)/(top-bottom);
        temp=temp*1.5;
        NewImage(:,:,i)=temp;
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(NewImage);
    title('同态滤波');
    handles.outputimg = NewImage;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_40_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_40 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_52_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_52 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 函数fft2()用于计算二维傅立叶变换
% 函数fftshift()是对函数fft2()作傅里叶变换后得到的频谱进行平移,将变换后的图像频谱中心从矩阵的原点移到矩阵的中心
% 作二维傅里叶变换前一定要用函数im2double()把原始图像的数据类型由uint8转化为double类型
% 否则会因为unit8数据类型只能表示0-255的整数而出现数据截断,进而出现错误结果
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(im2double(I)));
    [a,b]=size(s);
    a0=round(a/2);
    b0=round(b/2);
    W=30;  % 将理想带阻滤波器的带W设置为30
    d0=50; % 将理想带阻滤波器的截止频率D0设置为50
    for i=1:a %双重for循环计算频率点(i,j)与频域中心的距离D(i,j)=sqrt((i-round(a/2)^2+(j-round(b/2)^2))
        for j=1:b 
            distance=sqrt((i-a0)^2+(j-b0)^2);
            if distance<=d0-(W/2)  % 根据理想带阻滤波器产生公式分别进行三次if判断
                h=1;
            end
            if (distance>=d0-W/2)&&(distance<=d0+W/2)
                h=0;        
            end
            if distance>d0+W/2
                h=1;
            end
            s(i,j)=h*s(i,j);% 频域图像乘以滤波器的系数
        end
    end
    % real函数取元素的实部
    s=real(ifft2(ifftshift(s)));% 最后进行二维傅里叶反变换转换为时域图像
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('理想带阻滤波，带为30，截止频率为50Hz');
    handles.outputimg = s;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_53_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_53 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 函数fft2()用于计算二维傅立叶变换
% 函数fftshift()是对函数fft2()作傅里叶变换后得到的频谱进行平移,将变换后的图像频谱中心从矩阵的原点移到矩阵的中心
% 作二维傅里叶变换前一定要用函数im2double()把原始图像的数据类型由uint8转化为double类型
% 否则会因为unit8数据类型只能表示0-255的整数而出现数据截断,进而出现错误结果
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(im2double(I)));
    [N1,N2]=size(s);%求二维傅里叶变换后图像大小
    n=5;            % 将巴特沃斯带阻滤波器的阶数n设置为5
    W=30;           % 将巴特沃斯带阻滤波器的带宽W设置为30
    d0=50;          % 将巴特沃斯带阻滤波器的截止频率D0设置为50
    n1=round(N1/2);
    n2=round(N2/2);
    for i=1:N1      %双重for循环计算频率点(i,j)与频域中心的距离D(i,j)=sqrt((i-round(N1/2)^2+(j-round(N2/2)^2))
        for j=1:N2 
            distance=sqrt((i-n1)^2+(j-n2)^2);
            if distance==0 
                h=0; 
            else
                h=1/(1+((distance*W)/(distance*distance-d0*d0))^(2*n));% 根据巴特沃斯带阻滤波器公式为1/(1+[(D(i,j)*W)/(D^2(i,j)-D0^2)]^2n)
            end
            s(i,j)=h*s(i,j);% 频域图像乘以滤波器的系数
        end
    end
        % real函数取元素的实部
    s=real(ifft2(ifftshift(s)));% 最后进行二维傅里叶反变换转换为时域图像
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('巴特沃斯带阻滤波，带为30，D0为50Hz，阶数为2');
    handles.outputimg = s;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_54_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_54 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 函数fft2()用于计算二维傅立叶变换
% 函数fftshift()是对函数fft2()作傅里叶变换后得到的频谱进行平移,将变换后的图像频谱中心从矩阵的原点移到矩阵的中心
% 作二维傅里叶变换前一定要用函数im2double()把原始图像的数据类型由uint8转化为double类型
% 否则会因为unit8数据类型只能表示0-255的整数而出现数据截断,进而出现错误结果
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(im2double(I)));
    [a,b]=size(s);
    W=30;  % 将高斯带阻滤波器的带宽W设置为30
    d0=50; % 将高斯带阻滤波器的截止频率D0设置为50
    a0=round(a/2);
    b0=round(b/2);
    for i=1:a
        for j=1:b
            distance=sqrt((i-a0)^2+(j-b0)^2);    % 根据高斯带阻滤波器公式H(u,v)=1-e^-(1/2)[(D^2(u,v)-D^20)/D(u,v)*W] 
            h=1-exp(-0.5*((distance^2-d0^2)/(distance*W))^2); % exp表示以e为底的指数函数
            s(i,j)=h*s(i,j);% 频域图像乘以滤波器的系数
        end
    end
    s=real(ifft2(ifftshift(s)));% 最后进行二维傅里叶反变换转换为时域图像
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('高斯带阻滤波，带为30，D0为50Hz');
    handles.outputimg = s;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_49_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_49 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(I));
    [a,b]=size(s);
    a0=round(a/2);
    b0=round(b/2);
    d=50;
    for i=1:a 
        for j=1:b 
            distance=sqrt((i-a0)^2+(j-b0)^2);
            if distance<=d
                h=1;
            else
                h=0;
            end
            s(i,j)=h*s(i,j);
        end
    end
    s=uint8(real(ifft2(ifftshift(s))));
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('理想低通滤波');
    handles.outputimg = s;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_50_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_50 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    f=double(I);%数据类型转换
    g=fft2(f);%图像傅里叶转换 
    g=fftshift(g);%傅里叶变换平移
    % 	F2=log(abs(g));%对傅里叶变换结果取绝对值，然后取对数
    [N1,N2]=size(g);%傅里叶变换图像尺寸
    n=2;%参数赋初始值
    d0=30;
    n1=fix(N1/2);%数据圆整
    n2=fix(N2/2);%数据圆整
    for i=1:N1%遍历图像像素
        for j=1:N2 
            d=sqrt((i-n1)^2+(j-n2)^2);
            if d==0 
                h=0; 
            else
                h=1/(1+(d/d0)^(2*n));
            end
            result(i,j)=h*g(i,j);%?图像矩阵计算处理
        end
    end
    %F3=log(abs(result));%对傅里叶变换结果取绝对值，然后取对数
    result=ifftshift(result);
    X2=ifft2(result);
    X3=uint8(real(X2));
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(X3));
    title('巴特沃斯低通滤波，阶数为2，截止频率为30Hz');
    handles.outputimg = X3;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_51_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_51 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%IA=imnoise(I,'gaussian');%%加入高斯白噪声
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    IA = I;
    [f1,f2]=freqspace(size(IA),'meshgrid');
    D=100/size(IA,1);
    r=f1.^2+f2.^2;
    Hd=ones(size(IA));
    for i=1:size(IA,1)
        for j=1:size(IA,2)
            t=r(i,j)/(D*D);
            Hd(i,j)=exp(-t);
        end
    end
    Y=fft2(double(IA));
    Y=fftshift(Y);
    Ya=Y.*Hd;
    Ya=ifftshift(Ya);
    Ia=real(ifft2(Ya));
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(uint8(Ia));
    title('理想低通滤波');
    handles.outputimg = Ia;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_46_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_46 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 函数fft2()用于计算二维傅立叶变换
% 函数fftshift()是对函数fft2()作傅里叶变换后得到的频谱进行平移,
%将变换后的图像频谱中心从矩阵的原点移到矩阵的中心
% 作二维傅里叶变换前一定要用函数im2double()把原始图像的数据类型由uint8转化为double类型
% 否则会因为unit8数据类型只能表示0-255的整数而出现数据截断,进而出现错误结果
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(im2double(I)));
    [a,b]=size(s);
    a0=round(a/2);
    b0=round(b/2);
    d0=50; % 将理想高通滤波器的截止频率D0设置为50
    for i=1:a %双重for循环计算频率点(i,j)与频域中心的距离D(i,j)=sqrt((i-round(a/2)^2+(j-round(b/2)^2))
        for j=1:b 
            distance=sqrt((i-a0)^2+(j-b0)^2);
            if distance<=d0  % 根据理想高通滤波器产生公式,当D(i,j)<=D0,置为0
                 h=0;
            else
                 h=1;        % 根据理想高通滤波器产生公式,当D(i,j)>D0,置为1
            end
            s(i,j)=h*s(i,j);% 频域图像乘以滤波器的系数
        end
    end
    % real函数取元素的实部
    s=real(ifft2(ifftshift(s)));% 最后进行二维傅里叶反变换转换为时域图像
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('理想高通滤波，截止频率为50Hz');
    handles.outputimg = s;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_47_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_47 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 函数fft2()用于计算二维傅立叶变换
% 函数fftshift()是对函数fft2()作傅里叶变换后得到的频谱进行平移,将变换后的图像频谱中心从矩阵的原点移到矩阵的中心
% 作二维傅里叶变换前一定要用函数im2double()把原始图像的数据类型由uint8转化为double类型
% 否则会因为unit8数据类型只能表示0-255的整数而出现数据截断,进而出现错误结果
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(im2double(I)));
    [N1,N2]=size(s);%求二维傅里叶变换后图像大小
    n=2;            % 将巴特沃斯高通滤波器的阶数n设置为2
    d0=30;          % 将巴特沃斯高通滤波器的截止频率D0设置为30
    n1=round(N1/2);
    n2=round(N2/2);
    for i=1:N1      %双重for循环计算频率点(i,j)与频域中心的距离D(i,j)=sqrt((i-round(N1/2)^2+(j-round(N2/2)^2))
        for j=1:N2 
            distance=sqrt((i-n1)^2+(j-n2)^2);
            if distance==0 
                h=0; 
            else
                h=1/(1+(d0/distance)^(2*n));% 根据巴特沃斯高通滤波器公式为1/(1+[D0/D(i,j)]^2n)
            end
            s(i,j)=h*s(i,j);% 频域图像乘以滤波器的系数
        end
     end
% real函数取元素的实部
    s=real(ifft2(ifftshift(s)));% 最后进行二维傅里叶反变换转换为时域图像
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('巴特沃斯高通滤波，阶数为2，截止频率为30Hz');
    handles.outputimg = s;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_48_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_48 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 函数fft2()用于计算二维傅立叶变换
% 函数fftshift()是对函数fft2()作傅里叶变换后得到的频谱进行平移,将变换后的图像频谱中心从矩阵的原点移到矩阵的中心
% 作二维傅里叶变换前一定要用函数im2double()把原始图像的数据类型由uint8转化为double类型
% 否则会因为unit8数据类型只能表示0-255的整数而出现数据截断,进而出现错误结果
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
        return;
    end
    s=fftshift(fft2(im2double(I)));
    [a,b]=size(s);
    d0=10; % 将高斯高通滤波器的截止频率D0设置为10
    a0=round(a/2);
    b0=round(b/2);
    for i=1:a
        for j=1:b
            distance=sqrt((i-a0)^2+(j-b0)^2);    % 根据高斯高通滤波器公式H(u,v)=e^-[D^2(u,v)/2*D0^2] 
            h=1-(exp(-(distance^2)/(2*(d0^2)))); % exp表示以e为底的指数函数
            s(i,j)=h*s(i,j);% 频域图像乘以滤波器的系数
        end
    end
    s=real(ifft2(ifftshift(s)));% 最后进行二维傅里叶反变换转换为时域图像
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(s,[]);
    title('高斯高通滤波，截止频率为10Hz');
    handles.outputimg = s;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_55_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_55 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_56_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_56 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_57_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_57 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_58_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_58 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
%I = rgb2gray(I);
h1 = [-1,-1,-1;2,2,2;-1,-1,-1];
h2 = [-1,-1,2;-1,2,-1;2,-1,-1];
h3 = [-1,2,-1;-1,2,-1;-1,2,-1];
h4 = [2,-1,-1;-1,2,-1;-1,-1,2];
J1 = imfilter(I,h1);
J2 = imfilter(I,h2);
J3 = imfilter(I,h3);
J4 = imfilter(I,h4);
J = J1+J2+J3+J4;
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(J);
title('检测到的图像线段');
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% --------------------------------------------------------------------
function Untitled_59_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_59 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_61_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_61 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% I = handles.imgdata;
% I = im2double(I);
% J = imnoise(I,'gaussian',0,0.01);
% [K,thresh] = edge(J,'roberts');
% axes(handles.p2);
% imshow(J);
% axes(handles.p3);
% imshow(K);
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    J = edge(I,'canny');%同时检测水平和竖直方向
    axes(handles.p2);
    imshow(J);
    title('Canny算子');
    handles.outputimg= J;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_62_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_62 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% I = handles.imgdata;
% I = im2double(I);
% J = imnoise(I,'gaussian',0,0.005);
% [K,thresh] = edge(J,'log',[],2.3);
% axes(handles.p2);
% imshow(J);
% axes(handles.p3);
% imshow(K);
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    J = edge(I,'log',[],2.3);%同时检测水平和竖直方向
    axes(handles.p2);
    imshow(J);
    title('Log算子');
    handles.outputimg= J;
    guidata(hObject,handles);
end


% --------------------------------------------------------------------
function Untitled_63_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_63 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    J = edge(I,'roberts');
    axes(handles.p2);
    imshow(J);
    title('Roberts算子');
    handles.outputimg= J;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_64_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_64 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    J= edge(I,'prewitt');
    axes(handles.p2);
    imshow(J);
    title('prewitt算子');
    handles.outputimg= J;
    guidata(hObject,handles);
end
% --------------------------------------------------------------------
function Untitled_65_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_65 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
    num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    J = edge(I,'sobel');%同时检测水平和竖直方向
    axes(handles.p2);
    imshow(J);
    title('Sobel算子');
    handles.outputimg= J;
    guidata(hObject,handles);
end

% --------------------------------------------------------------------
function Untitled_66_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_66 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% key = get(handles.gamma,'String');
%out = I>key;
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
%     num = size(I);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
    key = get(handles.edit15,'String');
    if isempty(key)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<灰度阈值>的上框设置阈值,不能为空','警告','modal');
        return;
    end
    key = str2double(key);
    out = im2bw(I,key/256);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(out);
    title('全局阈值分割');
    handles.outputimg=out;
    guidata(hObject,handles);
end


% --------------------------------------------------------------------
function Untitled_67_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_67 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
%     num = size(I);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
%     key = get(handles.edit15,'String');
%     if isempty(key)
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请在<灰度阈值>的上框设置阈值,不能为空','警告','modal');
%         return;
%     end
    key = graythresh(I);
    out = im2bw(I,key);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(out);
    title('Otsu阈值分割');
    handles.outputimg=out;
    guidata(hObject,handles);
end


% --------------------------------------------------------------------
function Untitled_68_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_68 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
    I = handles.imgdata;
%     num = size(I);
%     if numel(num)>2
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请选择灰度图像进行该操作','警告','modal');%前者是内容，后者是名称
%         return;
%     end
%     key = get(handles.edit15,'String');
%     if isempty(key)
%         % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
%         errordlg('请在<灰度阈值>的上框设置阈值,不能为空','警告','modal');
%         return;
%     end
    t0 = 0.01;
    I = im2double(I);
    t1 = (min(I(:))+max(I(:)))/2;
    r1 = I>t1;
    r2 = I<=t1;
    t2 = (mean(I(r1))+mean(I(r2)))/2;
    while abs(t2-t1)<t0
        t1 = t2;
        r1 = I>t1;
        r2 = I<=t1;
        t2 = (mean(I(r1))+mean(I(r2)))/2;
    end
    out = im2bw(I,t2);
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
    axes(handles.p2);
    imshow(out);
    title('迭代式阈值分割');
    handles.outputimg=out;
    guidata(hObject,handles);
end


% --------------------------------------------------------------------
function Untitled_69_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_69 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function Untitled_70_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_70 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.imgfilename)
I=handles.imgdata;
I=double(I);              %转换为灰度值是0-1的双精度
num = size(I);
if numel(num)>2
    answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
    switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
    end
end
[M,N]=size(I);            %得到原图像的行列数
questdlg('请在图一选择一个生长点并双击','提示','确定','确定');
[y,x]=getpts;             %获得区域生长起始点
x2=abs(round(x));             %横坐标取整
y2=abs(round(y));            %纵坐标取整
seed=I(x2,y2);            %将生长起始点灰度值存入seed中
Y=zeros(M,N);             %作一个全零与原图像等大的图像矩阵Y，作为输出图像矩阵
Y(x2,y2)=1;               %将Y中与所取点相对应位置的点设置为白点
sum=seed;                 %储存符合区域生长条件的点的灰度值的总和
suit=1;                   %储存符合区域生长条件的点的总个数
count=1;                  %每次判断一点周围八点符合条件的新点的数目
threshold=10;             %域值，即某一点与周围八点的绝对差值要小于阈值
while count>0             %判断是否有新的符合生长条件的点，若没有，则结束
s=0;                      %判断一点周围八点时，符合条件的新点的灰度值之和
count=0;
for i=1:M
for j=1:N
if Y(i,j)==1
if (i-1)>0 & (i+1)<(M+1) & (j-1)>0 & (j+1)<(N+1) %判断此点是否为图像边界上的点
for u= -1:1                                        %判断点周围八点是否符合域值条件
for v= -1:1                                       %u,v为偏移量
if  Y(i+u,j+v)==0 & abs(I(i+u,j+v)-seed)<=threshold%判断是否未存在于输出矩阵Y，并且为符合域值条件的点
Y(i+u,j+v)=1;                                %符合以上两条件即将其在Y中与之位置对应的点设置为白点
count=count+1;                               %新的、符合生长条件的点的总个数
s=s+I(i+u,j+v);                              %新的、符合生长条件的点的总灰度数
end
end  
end
end
end
end
end
suit = suit+count;                                   %目前区域所有符合生长条件的点的总个数
sum  = sum+s;                                         %目前区域所有符合生长条件的点的总灰度值
seed = sum/suit;                                     %计算新的灰度平均值
end
axes(handles.p2);
imshow(Y);
title('区域生长法分离出的图像');
end



% --------------------------------------------------------------------
function Untitled_72_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_72 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% num = size(I);
% if numel(num)>2
%     k= watershed(I);
%     axes(handles.p2);
%     imshow(k);
%     j = rgb2gray(I);
% %     level=graythresh(j);
% %     g=im2bw(I,level); 
%     j = watershed(j);
%     j = label2rgb(j, 'jet', 'w', 'shuffle');
%     axes(handles.p3);
%     imshow(j);
% else
%     level=graythresh(I);
%     g=im2bw(I,level);  %用阈值法把平滑后的图像变为二值图像
% I = handles.imgdata;
% num = size(I);
% if numel(num)>2
%     answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
%     switch answer
%         case '是'
%             start_Callback(hObject, eventdata, handles);
%             return;
%         case '否'
%             return;
%     end
% end
% j = watershed(I,8);%对二维图像进行分割
% %j = label2rgb(j, 'jet', 'w', 'shuffle');
% j = histeq(j);
% axes(handles.p2);
% imshow(j);
% title('均衡化后的图像');
% % end

%新的算法
rgb = uint8(handles.imgdata);
num = size(rgb);
if numel(num)>2
    answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
    switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
    end
end
sz = size(rgb);
I = rgb;
if sz(1) ~= 256
    I = imresize(I, 256/sz(1));
%     rgb = imresize(rgb, 256/sz(1));
end      %调整图像尺寸
hy = fspecial('sobel');
hx = hy';
Iy = imfilter(double(I), hy, 'replicate');
Ix = imfilter(double(I), hx, 'replicate');
gradmag = sqrt(Ix.^2 + Iy.^2);
se = strel('disk', 3);
Io = imopen(I, se);
Ie = imerode(I, se);
Iobr = imreconstruct(Ie, I);
Ioc = imclose(Io, se);
Iobrd = imdilate(Iobr, se);
Iobrcbr = imreconstruct(imcomplement(Iobrd), imcomplement(Iobr));
Iobrcbr = imcomplement(Iobrcbr);
fgm = imregionalmax(Iobrcbr);
se2 = strel(ones(3,3));
fgm2 = imclose(fgm, se2);
fgm3 = imerode(fgm2, se2);
fgm4 = bwareaopen(fgm3, 15);
bw = im2bw(Iobrcbr, graythresh(Iobrcbr));
D = bwdist(bw);
DL = watershed(D);
bgm = DL == 0;
gradmag2 = imimposemin(gradmag, bgm | fgm4);
L = watershed(gradmag2);%分水岭
%Lrgb = label2rgb(L, 'jet', 'w', 'shuffle');
cla(handles.p2,'reset');
cla(handles.p3,'reset');
axes(handles.p2);
imshow(Iobrcbr); 
title('图像二值化')
axes(handles.p3);
imshow(gradmag,[]);
title('梯度图像')
%figure;
%imshow(rgb, []); 
%imshow(Lrgb);
figure;
imshow(histeq(L));
title('分水岭分割结果');


% --------------------------------------------------------------------
function Untitled_73_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_73 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_74_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_74 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_75_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_75 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_76_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_76 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_77_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_77 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_80_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_80 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
I = im2double(I);
h1 = size(I,1);
h2 = size(I,2);
H1 = hadamard(h1);
H2 = hadamard(h2);
J = H1*I*H2/sqrt(h1*h2);

axes(handles.p2);
imshow(J);
title('图像的Hadmard变换');

% --------------------------------------------------------------------
function Untitled_81_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_81 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_82_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_82 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
num = size(I);
if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
end
cla(handles.p2,'reset');
cla(handles.p3,'reset');
I = im2double(I);
J = dct2(I);%计算二维离散余弦变换
axes(handles.p2);
imshow(J);
title('二维离散余弦变换图像');
axes(handles.p3);
imshow(log(abs(J)),[]);
title('二维离散余弦变换系数的图像');

% --------------------------------------------------------------------
function Untitled_83_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_83 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
I = im2double(I);
num = size(I);
if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
end
cla(handles.p2,'reset');
cla(handles.p3,'reset');
s = size(I);
R = s(1);
S = s(2);
P = dctmtx(R);
Q = dctmtx(S);
J = P*I*Q';
K = dct2(I);
E = J-K;
find(abs(E)>0/0.000001);
axes(handles.p2);
imshow(J);
title('采用离散余弦变换得到的系数矩阵');
axes(handles.p3);
imshow(K);
title('采用dct2()得到的系数矩阵');
% --------------------------------------------------------------------
function Untitled_84_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_84 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
I = im2double(I);
num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
J= dct2(I);
J(abs(J)<0.1)=0;
K = idct2(J);
axes(handles.p2);
imshow(J);
title('离散余弦变换系数图像');
axes(handles.p3);
imshow(K);
title('离散余弦反变换图像');

% --------------------------------------------------------------------
function Untitled_85_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_85 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
fun1 = @dct2;
J1 = blkproc(I,[4 4],fun1);
fun2 = @(x) std2(x)*ones(size(x));
J2 = blkproc(I,[4 4],fun2);
axes(handles.p2);
imshow(J1);
title('图像分块后采用函数fun1处理');
axes(handles.p3);
imshow(J2);
title('图像分块后采用函数fun2处理');


% --------------------------------------------------------------------
function Untitled_78_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_78 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_79_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_79 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_86_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_86 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

I = handles.imgdata;
num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
% I = im2double(I);
BW = edge(I,'canny');
[H, Theta, Rho] = hough(BW,'RhoResolution',0.5,'ThetaResolution',0.5);
% disp('变化角度为：');
% disp(Theta);
% disp('变换半径为：');
% disp(Rho);
% set(0,'defaultFigurePosition',[100,100,1000,500]);
% set(0,'defaultFigureColor',[1 1 1]);
axes(handles.p2);
imshow(BW);
title('图像边缘信息的二值图像');
axes(handles.p3);
imshow(imadjust(mat2gray(H)));%mat2gray进行归一化操作，调整图像的对比度，
% imadjust分别对低强度和高强度部分 1% 的数据进行饱和处理，并显示它
axis normal;%自动调整轴的比值
hold on;
xlabel('Hough变换结果');
% colormap('autumn');
% colormap hot;

% --------------------------------------------------------------------
function Untitled_87_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_87 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p2,'reset');
cla(handles.p3,'reset');
I = handles.imgdata;
num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
    cla(handles.p3,'reset');
    cla(handles.p2,'reset');
BW = edge(I, 'canny'); 
% Create the hough transform using the binary image 
[H, T, R] = hough(BW); 
axes(handles.p2);
imshow(H, [], 'XData', T, 'YData', R, 'InitialMagnification', 'fit'); 
xlabel('图像的Hough变化：\theta'), ylabel('\rho'); 
axis on, axis normal, hold on;
% colormap(gca, hot);

% Find peaks in the hough transform of the image
P = houghpeaks(H, 5); 
x = T(P(:,2));
y = R(P(:,1)); 
plot(x, y, 's', 'color', 'green'); 

% Find lines and plot them
lines = houghlines(BW, T, R, P, 'FillGap', 5, 'MinLength', 7); 
axes(handles.p3);
imshow(I), hold on;
title('检测到的直线');
max_len = 0; 
for k = 1 : length(lines)  % here length(lines)=12
xy = [lines(k).point1; lines(k).point2]; 
plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');

% Plot beginnings and ends of lines
plot(xy(1,1), xy(1,2),  'LineWidth', 2, 'Color', 'yellow');
plot(xy(2,1), xy(2,2),  'LineWidth', 2, 'Color', 'red'); 

% Determine the endpoints of the longest line segment
len = norm(lines(k).point1 - lines(k).point2); % distance between point1 and point2
if ( len > max_len )
max_len = len; 
xy_long = xy; 
end
end
% Highlinght the longest line segment by coloring it cyan
plot(xy_long(:, 1), xy_long(:, 2), 'LineWidth', 2, 'Color', 'cyan');



% BW = edge(I,'canny');
% [H, Theta, Rho] = hough(BW,'RhoResolution',0.5,'Theta',-90:0.5:89.5);
% P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
% x = Theta(P(:,2));
% y = Rho(P(:,1));
% % set(0,'defaultFigurePosition',[100,100,1000,500]);
% % set(0,'defaultFigureColor',[1 1 1]);
% axes(handles.p2);
% imshow(imadjust(mat2gray(H)),'XData',Theta,'YData',Rho,'InitialMagnification','fit');
% axis on;
% axis normal;
% hold on;
% plot(x,y,'s','color','white');
% lines = houghlines(BW,Theta,Rho,P,'FillGap',5,'MinLength',7);
% axes(handles.p3);
% imshow(I);
% hold on;
% maxlen = 0;
% for k=1:length(lines)
%     xy = [lines(k).point1-lines(k).point2];
%     plot(xy(:,1),xy(:,2),'linewidth',2,'color','green');
%     plot(xy(1,1),xy(1,2),'linewidth',2,'color','yellow');
%     plot(xy(2,1),xy(2,2),'linewidth',2,'color','red');
%     len = norm(lines(k).point1-lines(k).point2);
%     if(len>maxlem)
%         maxlen = len;
%         xylong = xy;
%     end
% end
% hold on;
% plot(xylong(:,1),xylong(:,2),'color','blue');

% BW=edge(I, 'canny');
% [H, Theta, Rho]=hough(BW, 'RhoResolution', 0.5, 'Theta', -90:0.5:89.5);
% P=houghpeaks(H, 5, 'threshold', ceil(0.3*max(H(:))));
% x=Theta(P(:, 2));
% y=Rho(P(:, 1));
% axes(handles.p2);
% % set(0,'defaultFigurePosition',[100,100,1000,500]);
% % set(0,'defaultFigureColor',[1 1 1])
% % subplot(121);
% imshow(imadjust(mat2gray(H)), 'XData', Theta, 'YData', Rho,...
%     'InitialMagnification', 'fit');
% axis on; 
% axis normal;
% hold on;
% plot(x, y, 's', 'color', 'white');
% lines=houghlines(BW, Theta, Rho, P, 'FillGap', 5, 'MinLength', 7);
% axes(handles.p3);
% imshow(I);
% hold on;
% maxlen=0;
% for k=1:length(lines)
%     xy=[lines(k).point1; lines(k).point2];
%     plot(xy(:,1), xy(:, 2), 'linewidth', 2, 'color', 'green');
%     plot(xy(1,1), xy(1, 2), 'linewidth', 2,  'color', 'yellow');
%     plot(xy(2,1), xy(2, 2), 'linewidth', 2,  'color', 'blue');
%     len=norm(lines(k).point1-lines(k).point2);
%     if (len>maxlen)
%         maxlen=len;
%         xylong=xy;
%     end
% end
% hold on;
% plot(xylong(:, 1), xylong(:, 2), 'color', 'red');






% --------------------------------------------------------------------
function Untitled_95_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_95 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_96_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_96 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p3,'reset');
warning('off');
i = handles.outputimg;
k = ifft2(i);
axes(handles.p3);
imshow(uint8(k));
title('傅里叶反变换');

% --------------------------------------------------------------------
function Untitled_97_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_97 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_108_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_108 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I=handles.imgdata;  
I=im2double(I);
num  = size(I);
if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
end
cla(handles.p2,'reset');
cla(handles.p3,'reset');
J=fftshift(fft2(I));    
[x, y]=meshgrid(-128:127, -128:127);
z=sqrt(x.^2+y.^2);
D1=10;  D2=30;
n=6;
H1=1./(1+(z/D1).^(2*n));
H2=1./(1+(z/D2).^(2*n));
K1=J.*H1;
K2=J.*H2;
L1=ifft2(ifftshift(K1));
L2=ifft2(ifftshift(K2));
axes(handles.p2);
imshow(L1);
title('巴特沃斯低通滤波（截止频率为10Hz）');
axes(handles.p3);
imshow(L2);
title('巴特沃斯低通滤波（截止频率为30Hz）');



% --------------------------------------------------------------------
function Untitled_109_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_109 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I=handles.imgdata;  
I=im2double(I);
num = size(I);
if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
end
cla(handles.p2,'reset');
cla(handles.p3,'reset');
J=fftshift(fft2(I));    
[x, y]=meshgrid(-128:127, -128:127);
z=sqrt(x.^2+y.^2);
D1=10;  D2=40;
n1=4;  n2=8;
H1=1./(1+(D1./z).^(2*n1));
H2=1./(1+(D2./z).^(2*n2));
K1=J.*H1;
K2=J.*H2;
L1=ifft2(ifftshift(K1));
L2=ifft2(ifftshift(K2));
axes(handles.p2);
imshow(L1);
title('巴特沃斯高通滤波（截止频率为20Hz）');
axes(handles.p3);
imshow(L2);
title('巴特沃斯高通滤波（截止频率为40Hz）');



% --------------------------------------------------------------------
function Untitled_98_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_98 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
cla(handles.p3,'reset');
cla(handles.p2,'reset');
j = fft2(I);
k = abs(j/256);
axes(handles.p2);
imshow(uint8(k));
title('fft2');
handles.outputimg = j;
guidata(hObject,handles);

% --------------------------------------------------------------------
function Untitled_99_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_99 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I = I*exp(1);
I(I>255)=255;
j = fft2(I);
j = fftshift(j);
h = abs(j/255);
axes(handles.p2);
imshow(uint8(h));
title('变亮');
handles.outputimg = j;
guidata(hObject,handles);


% --------------------------------------------------------------------
function Untitled_100_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_100 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
cla(handles.p3,'reset');
cla(handles.p2,'reset');
j = fft2(I);
j = fftshift(j);
h = abs(j/256);
axes(handles.p2);
imshow(uint8(h));
title('fft2 & fffshift');
handles.outputimg = j;
guidata(hObject,handles);

% --------------------------------------------------------------------
function Untitled_101_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_101 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I = imrotate(I,45,'bilinear');
j = fft2(I);
j = fftshift(j);
h = abs(j/256);
axes(handles.p2);
imshow(uint8(h));
title('旋转');
handles.outputimg = j;
guidata(hObject,handles);

% --------------------------------------------------------------------
function Untitled_102_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_102 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I = imnoise(I,'gaussian',0,0.01);
j = fft2(I);
j = fftshift(j);
h = abs(j/256);
axes(handles.p2);
imshow(uint8(h));
title('高斯噪声');
handles.outputimg = j;
guidata(hObject,handles);

% --------------------------------------------------------------------
function Untitled_103_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_103 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = handles.imgdata;
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I = imnoise(I,'salt & pepper',0.2);
j = fft2(I);
j = fftshift(j);
h = abs(j/256);
axes(handles.p2);
imshow(uint8(h));
title('椒盐噪声');
handles.outputimg = j;
guidata(hObject,handles);

% --------------------------------------------------------------------
function Untitled_106_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_106 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% I = handles.imgdata;
% j = fft2(I);
% j = fftshift(j);
% fftr = real(j);
% ffti = imag(j);
% a = sqrt(fftr.^2+ffti.^2);
% a = (a-min(min(a)))/(max(max(a))-min(min(a)))*255;
% 
% axes(handles.p2);
% imshow(a);
% title('幅值谱');
% axes(handles.p2);
% imshow(abs(NC)*100);
% title('纯幅值谱重建');
% handles.outputimg = j;
% guidata(hObject,handles);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
TA = handles.imgdata; 
FA = fft2(TA);    %FFT2
MFA = abs(FA);    %幅度取绝对值
fftr = real(fftshift(FA));
ffti = imag(fftshift(FA));
% a = sqrt(fftr.^2+ffti.^2);
% a = (a-min(min(a)))/(max(max(a))-min(min(a)))*255;
margin=log(abs(fftshift(FA)));  
axes(handles.p2);
% imshow(margin,[]);
imshow(mat2gray(margin));
handles.outputimg = margin;
guidata(hObject,handles);
title('幅值谱');
NNFC = MFA.*exp(1i*1);%无相位谱，纯幅度谱  subplot(338)----位置信息丢失
NNC = ifft2(NNFC);
axes(handles.p3);imshow(abs(NNC)/290);title('纯幅度谱重建');

% --------------------------------------------------------------------
function Untitled_107_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_107 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% I = handles.imgdata;
% j = fft2(I);
% b = angle(j);
% axes(handles.p2);
% imshow(real(b));
% title('相位谱');
% handles.outputimg =j;
% guidata(hObject,handles)
cla(handles.p3,'reset');
cla(handles.p2,'reset');
TA = handles.imgdata; 
FA = fft2(TA);    %FFT2
AFA = angle(FA);  %相位
NFC = 1.*exp(1i*AFA); %无幅度谱，纯相位谱  
NC = ifft2(NFC);
axes(handles.p2);
imshow(real(AFA));
title('相位谱');   
axes(handles.p3);
imshow(abs(NC)*100);
title('纯相位谱重建')

function Untitled_112_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_112 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I=handles.imgdata; 
num = size(I);
    if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
    end
theta=0:1:179;
[R, xp]=radon(I, theta);
J=iradon(R, theta);
axes(handles.p2);
imagesc(theta, xp, R);
xlabel('Radon变换结果');
% axis normal;
axes(handles.p3);
imshow(uint8(J));



function Untitled_113_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_113 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I=handles.imgdata;%读入图像
num = size(I);
if numel(num)>2
        answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
        switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
        end
end
% axes(handles.p1);
% % figure;
% imshow(I);
% title('头部灰度图像');
theta1=0:10:170;
theta3=0:2:178;
[R1, xp]=radon(I, theta1);
[R3, xp]=radon(I, theta3);
axes(handles.p2);
imagesc(theta3, xp, R3);
xlabel('投影角度间隔为2°(90个角度)的radon变换图像');
% colormap(handles.p2, hot);
% colorbar;
axes(handles.p3);
imagesc(theta1, xp, R1);
xlabel('投影角度间隔为10°(18个角度)的radon变换图像');
% colormap(handles.p3, hot);
% colorbar;
J1=iradon(R1, 10);
J3=iradon(R3, 2);
figure;
subplot(121);
imshow(J1);
title('18个角度radon反变换得到的图像');
subplot(122);
imshow(J3);
title('90个角度radon反变换得到的图像');

function Untitled_110_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_110 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.p3,'reset');
cla(handles.p2,'reset');
Aa = get(handles.Aa,'string');
if isempty(Aa)
        % mode=struct('WindowStyle','modal','Interpreter','tex');%modal即为模态
        errordlg('请在<输入>框内设置Radon变换角度','警告','modal');
        return;
end
kos = Aa;
Aa = str2double(Aa);
I = handles.imgdata;
num = size(I);
if numel(num)>2
    answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
    switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
    end
end
[R, xp]=radon(I,Aa);
theta = 0:180;
[k,xp1] = radon(I,theta);
% axes(handles.p1);
% imshow(I);
axes(handles.p2);
plot(xp,R);%索引仅仅是因为有两个输出值
xlabel([kos,'°方向上的Radon变换']);
% axes(handles.p3);
% figure;
% plot(xp1,k);
% xlabel('0-180°的radon变换');
axes(handles.p3);
imagesc(theta,xp1,k);
xlabel('0-180°的radon变换');
% ylabel('x''');
% colormap(handles.p3,hot),colorbar;


% --------------------------------------------------------------------
function Untitled_111_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_111 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I=handles.imgdata; 
num = size(I);
if numel(num)>2
    answer = questdlg('需要的输入应该是灰度图像，是否重新选择?', 'Dessert Menu', '是','否','否');
    switch answer
        case '是'
            start_Callback(hObject, eventdata, handles);
            return;
        case '否'
            return;
    end
end
cla(handles.p3,'reset');
cla(handles.p2,'reset');
I = im2double(I);
J=mat2gray(I);
BW=edge(J);
axes(handles.p2);
imshow(BW);
title('边缘信息');
theta=0:179;
[R, xp]=radon(BW, theta);
axes(handles.p3);
imagesc(theta, xp, R);
xlabel('检测直线结果');
% colormap(handles.p3,hot);
% colorbar;
Rmax=max(max(R));
s1 = sprintf('radon变换结果最大值：%.2f\n',Rmax);
disp(s1);
[row, column]=find(R>=Rmax);
s2 = sprintf('最大值对应的行列：\n行: %d   列：%d\n',row,column);
disp(s2);
x=xp(row);
s3 = sprintf('最大值对应的位置：%.2f\n',x);
disp(s3);
angel=theta(column);
s4 = sprintf('最大值对应的角度：%.2f\n',angel);
disp(s4);



% --- Executes on button press in Aa.
function Aa_Callback(hObject, eventdata, handles)
% hObject    handle to Aa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


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


% --- Executes on button press in notlinear1.


% --------------------------------------------------------------------
function Untitled_119_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_119 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
R=handles.imgdata; 
G = R;
num = size(G);
if numel(num)>2
G = rgb2gray(G);
end
J=double(G);                                
H=(log(J+1))/10;                             
%set(0,'defaultFigurePosition',[100,100,1000,500]);
%image(handles.p2,H);
cla(handles.p3,'reset');
cla(handles.p2,'reset');
axes(handles.p2);
imshow(H);
handles.outputimg=H;
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function cut_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cut (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
