function [sys,x0,str,ts] = mpc08522(t,x,u,flag)
% �ó����ܣ���LTVMPC �ͳ����򻯶���ѧģ�ͣ�С�Ƕȼ��裩��ƿ���������ΪSimulink�Ŀ�����
% ����汾 V1.0��MATLAB�汾��R2019a,����S�����ı�׼��ʽ��
% Q_40*40,R_5*5,a_6*6,b_6*1,d_k 6*1,A_7*7,B_7*1,C_2*7
% Nx=6;%״̬���ĸ�����Nu=1;%�������ĸ�����Ny=2;%������ĸ�����Np =20;%Ԥ��ʱ��Nc=5;%����ʱ��
% ״̬��=[y_dot,x_dot,phi,phi_dot,Y,X]��������Ϊǰ��ƫ��delta_f
%����������ɢ���Ի��ڶ��ַ�������״̬�켣�����Ի��������ڽ�����һʱ��״̬�ռ�Ԥ���ʱ�򣬲�û���õ�״̬������������ö������һ��ƫ��d_k����ʾ���������������ͬ������������ϸ���ܡ�
switch flag,
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
 case 2
  sys = mdlUpdates(t,x,u); % Update discrete state 
 case 3
  sys = mdlOutputs(t,x,u); % Calculate outputs
 case {1,4,9} % Unused flags
  sys = [];
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 17;%%�������gp�˴�Ӧ��Ϊ17.����gpΪ11
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes); 
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];    
global U;%UΪ���ǵĿ�����
U=[0 ];%��������ʼ��,���������һ�������켣����������ȥ����UΪһά��

% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
 ts  = [0 0];        % sample time: [period, offset]������ʱ��Ӱ�����ʹ����������
%End of mdlInitializeSizes
global VehiclePara; 
    VehiclePara.m   = 2298.44;   %mΪ��������,Kg; Sprung mass = 1370
    VehiclePara.g   = 9.8;
    VehiclePara.Lf  = 1.33;  % 1.05
    VehiclePara.Lr  = 1.81;  % 1.55
    VehiclePara.L   = 2.33;  %VehiclePara.Lf + VehiclePara.Lr;
    VehiclePara.Iz  = 7152;   %IΪ������Z���ת���������������в���  
    VehiclePara.Ccf  = 66900;
    VehiclePara.Ccr  = 62700;
    VehiclePara.Clf  = 66900;
    VehiclePara.Clr  = 62700;
global times;
times = 0;
global MPCParameter; 
    MPCParameter.Ts  = 0.02; % 	���沽��      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
     global a b; %����a,b����Ϊȫ����
     global VehiclePara;
     global MPCParameter;
     global U; %����UΪȫ����
%      global times;
%      times = times+1
    tic
    Nx=6;%״̬���ĸ���
    Nu=1;%�������ĸ���
    Ny=2;%������ĸ���
    Np =10;%Ԥ��ʱ��
    Nc=4;%����ʱ��
    Row=10000;%�ɳ�����Ȩ��
    fprintf('Update start, t=%6.3f\n',t)
    %% ����·������
    load ref11
%     load gpdata
    %%
    T=0.02;%���沽��
   %����ӿ�ת��,x_dot�����һ���ǳ�С�������Ƿ�ֹ���ַ�ĸΪ���������������ǽ�����״̬����˳��һ�¡�
   %����Ϊ���ǵ�״̬���������������ǵĳ�ʼֵ���ĺ�С���������Ժ���
    y_dot=u(1)/3.6; %�����ٶȻ�Ϊm/s
    x_dot=u(2)/3.6+0.0001;%CarSim�������km/h��ת��Ϊm/s
    phi=u(3)*3.141592654/180; %CarSim�����Ϊ�Ƕȣ��Ƕ�ת��Ϊ���ȣ����ʱ���Ƕ���ʹ�û���
    phi_dot=u(4)*3.141592654/180;%���ٶ�
    Y=u(5);%��λΪm������λ��
    X=u(6);%��λΪ�ף�����λ��
    Sf=u(7);%��ǰ�ֻ�����
    Sr=u(8);%��ǰ�ֻ�����
%     gpmean_vy = u(12);
%     gpmean_vx = u(13);
%     gpmean_yaw_rate = u(14);
%     gpmean_yaw = u(15);
%     gpcov_vy = u(16);
%     gpcov_yaw_rate = u(17);
%% ������������
%syms sfΪǰ�ֻ����ʣ�srΪ���ֻ�����
%     Sf=0.2; Sr=0.2;
%syms lf%ǰ�־��복�����ĵľ��룬lrΪ���־��복�����ĵľ���
    lf=1.33;lr=1.81;
%syms C_cfǰ�����Ժ����ƫ�նȣ� C_cr�������Ժ����ƫ�ն� ��C_lf ǰ�������ƫ�նȣ� C_lr ���������ƫ�ն�
    Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%mΪ����������gΪ�������ٶȣ�IΪ������Z���ת���������������в���
    m=2298;g=9.8;I=7152;
   

%% �ο��켣����
    X_predict=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵ�����λ����Ϣ�����Ǽ��������켣�Ļ���
    phi_ref=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵĲο���ڽ���Ϣ
    Y_ref=zeros(Np,1);%���ڱ���Ԥ��ʱ���ڵĲο�����λ����Ϣ

%     times = ceil(t/0.01)+1;
%     %  ���¼���״̬������״̬�������������һ�� 
%     fprintf('time is ,%f\n',times);
    
    kesi=zeros(Nx+Nu,1);%״̬��6����������1��
    kesi(1)=y_dot;%
    kesi(2)=x_dot;
    kesi(3)=phi; 
    kesi(4)=phi_dot;
    kesi(5)=Y;
    kesi(6)=X; %����6��Ϊ���ǵ�״̬��
    kesi(7)=U(1); %����ǿ�����ǰ��ƫ�ǣ����ڹ켣���ٹ�����ͨ����ǰ��ƫ�ǽ��п��ƣ������ٶȱ��ֲ���
    delta_f=U(1);%ǰ��ת��,����Ҫ��
    fprintf('Update start, u(1)=%4.2f\n',U(1))   
    %Ȩ�ؾ������� 
    Q_cell=cell(Np,Np);%�ܵ�Ԫ��QΪ20��20�еģ�40*40
    for i=1:1:Np 
        for j=1:1:Np
            if i==j
               Q_cell{i,j}=[2000 0;0 10000;];
%                  Q_cell{i,j}=[300 0;0 100;]; %�����ǽ�����Խ��ߵĵط�����Ϊ�þ�������Ϊֹȫ��Ϊ0
            else 
                Q_cell{i,j}=zeros(Ny,Ny);    %20��20�еľ�����ÿһ��С��λ���϶���2*2��С����           
            end
        end 
    end 
   Q=cell2mat(Q_cell); %�ܵ�Ԫ��QΪ20��20�еģ�ÿ��λ�ö���2*2��С����Q����Ϊ40*40
   R=5*10^2*eye(Nu*Nc);
    %�����Ҳ����Ҫ�ľ����ǿ������Ļ��������ö���ѧģ�ͣ��þ����복������������أ�ͨ���Զ���ѧ��������ſ˱Ⱦ���õ���aΪ6*6��bΪ6*1
     [a, b] = func_Model_linearization_Jacobian(kesi,Sf,Sr,MPCParameter,VehiclePara);
%         a=[                 1 - (259200*T)/(1723*x_dot),                                                         -T*(phi_dot + (2*((460218*phi_dot)/5 - 62700*y_dot))/(1723*x_dot^2) - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2)),                                    0,                     -T*(x_dot - 96228/(8615*x_dot)), 0, 0
%         T*(phi_dot - (133800*delta_f)/(1723*x_dot)),                                                                                                                  (133800*T*delta_f*((154*phi_dot)/125 + y_dot))/(1723*x_dot^2) + 1,                                    0,           T*(y_dot - (824208*delta_f)/(8615*x_dot)), 0, 0
%                                                   0,                                                                                                                                                                                  0,                                    1,                                                   T, 0, 0
%             (33063689036759*T)/(7172595384320*x_dot), T*(((2321344006605451863*phi_dot)/8589934592000 - (6325188028897689*y_dot)/34359738368)/(4175*x_dot^2) + (5663914248162509*((154*phi_dot)/125 + y_dot))/(143451907686400*x_dot^2)),                                   0, 1 - (813165919007900927*T)/(7172595384320000*x_dot), 0, 0
%                                           T*cos(phi),                                                                                                                                                                         T*sin(phi),  T*(x_dot*cos(phi) - y_dot*sin(phi)),                                                   0, 1, 0
%                                          -T*sin(phi),                                                                                                                                                                         T*cos(phi), -T*(y_dot*cos(phi) + x_dot*sin(phi)),                                                   0, 0, 1];
%    
%     b=[                                                               133800*T/1723
%        T*((267600*delta_f)/1723 - (133800*((154*phi_dot)/125 + y_dot))/(1723*x_dot))
%                                                                                  0
%                                                 5663914248162509*T/143451907686400
%                                                                                  0
%                                                                                  0];  
    d_k=zeros(Nx,1);%����ƫ��,6*1
    state_k1=zeros(Nx,1);%Ԥ����һʱ��״̬�������ڼ���ƫ�6*1
    %���¼�Ϊ��ģ�;���ɢ������Ԥ����һʱ��״̬��
%     gpmean_vy = gpdata(times,1);
%     gpmean_vx = gpdata(times,2);
%     gpmean_yaw_rate = gpdata(times,3);
    %  ϵͳ��һʱ��״̬��
%     state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m+(t>2.5&&t<6)*gpmean_vy+(t>8)*gpmean_vy)+(t>=6&&t<=8)*gpmean_vy+(t<=2.5)*gpmean_vy;
% %     state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr-Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
%     state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m+(t>2.5&&t<6)*gpmean_vx+(t>8)*gpmean_vx)+(t>=6&&t<=8)*gpmean_vx+(t<=2.5)*gpmean_vx;
%     state_k1(3,1)=phi+T*(phi_dot);
%     state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I+(t>2.5&&t<6)*gpmean_yaw_rate+(t>8)*gpmean_yaw_rate)+(t>=6&&t<=8)*gpmean_yaw_rate+(t<=2.5)*gpmean_yaw_rate;
%     state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
%     state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi)); %����Ϊֹ����һʱ�̵�״̬���Ѿ��Ƶ����
    
%     state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m+gpmean_vy);
% %     state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr-Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
%     state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m+gpmean_vx);
%     state_k1(3,1)=phi+T*phi_dot;
%     state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I+gpmean_yaw_rate);
%     state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
%     state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi)); %����Ϊֹ����һʱ�̵�״̬���Ѿ��Ƶ����

    state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m);
%     state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr-Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
    state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
    state_k1(3,1)=phi+T*phi_dot;
    state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I);
    state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
    state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi)); %����Ϊֹ����һʱ�̵�״̬���Ѿ��Ƶ����
    %% +Bc dk
    d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);%��ƫ�����d_k��������ʽ��15a)
   % d_k=state_k1;
    d_piao_k=zeros(Nx+Nu,1);%ƫ�����d_k��������ʽ���ּ��˿���������һ�У�ά��7*1�ο�falcone(B,4c)
    d_piao_k(1:6,1)=d_k; %��d_k��������ʽ���и�ֵ��ǰ6��Ϊ���ǵ�ƫ��
    d_piao_k(7,1)=0;%���һ��Ϊ���ǵĿ�����
    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;%��һ�е�һ��6*6
    A_cell{1,2}=b;%��һ�еڶ���Ϊ6*1
    A_cell{2,1}=zeros(Nu,Nx);%�ڶ��е�һ��Ϊ1*6
    A_cell{2,2}=eye(Nu);%����еڶ���Ϊ1*1
    B_cell{1,1}=b;%B�����һ�е�һ��Ϊb,6*1
    B_cell{2,1}=eye(Nu);%�ڶ��е�һ��Ϊ1*1��λ��
    %A=zeros(Nu+Nx,Nu+Nx);
    A=cell2mat(A_cell);%A����ά��7*7
    B=cell2mat(B_cell);%B����ά��7*1
    C=[0 0 1 0 0 0 0;0 0 0 0 1 0 0;];%����ֻ���״̬���ռ�ĵ�����������ڽǣ��͵������������ƫ������
    %���¾�����󷨼�����ʽ��21��
    PSI_cell=cell(Np,1);%������̵ĵ�һ��ϵ������ά�� 20*1
    THETA_cell=cell(Np,Nc);%������̵ĵڶ���ϵ������ά�� 20*5
    GAMMA_cell=cell(Np,Np);%ά�� 20*20
    PHI_cell=cell(Np,1);%ά�� 20*1
    %����������ʽ��21���ĵ�������
    for p=1:1:Np
        PHI_cell{p,1}=d_piao_k;%  7��1�е�ƫ���������˵�������Ҫʵʱ���µģ�����Ϊ�˼�㣬������һ�ν���
        for q=1:1:Np 
            if q<=p  %�����Ǿ���
                GAMMA_cell{p,q}=C*A^(p-q); %�þ�����C��A�������
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu); %ÿһ��Ԫ���Ĵ�СΪ2�������������*7��״̬�ռ����ĸ�����
            end 
        end
    end  %���ոþ���Ϊ40*140
    %����������ʽ��21���ĵ�
    for j=1:1:Np %j��1��20
     PSI_cell{j,1}=C*A^j; %�����������ʽ��21���е�һ���ϵ������2*7*7*7=2*7
        for k=1:1:Nc %k��1��5�������������ʽ��21���еڶ����ϵ������
            if k<=j  %�����ǵ�������ֵ
                THETA_cell{j,k}=C*A^(j-k)*B;  
            else %����������ȫ��Ϊ0
                THETA_cell{j,k}=zeros(Ny,Nu); %ÿһ��СԪ������2*1С����
            end
        end
    end
     %������ʽ��21����ǰ�����ϵ������PSIΪ״̬�ռ�kesi��ϵ������THETAΪ����������ϵ������
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]����20��1�У�ÿһ��СԪ������2*7��С��������Ϊ40*7
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]��40*5
    %������ʽ��21���ĵ�������
    GAMMA=cell2mat(GAMMA_cell);%��д��GAMMA��ά��40*140
    PHI=cell2mat(PHI_cell);%���ڸ�Ԫ������ÿһ��λ�ö���7*1�ľ���һ��20�У�����һ����140*1
    %144���Ѿ����QԪ�����飬���ڽ���ת���ɾ�����ʽ
    Q=cell2mat(Q_cell); %�ܵ�Ԫ��QΪ20��20�еģ�ÿ��λ�ö���2*2��С����Q����Ϊ40*40
    %������ι滮��H����
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+R;%��һ�е�һ�е�λ��Ϊ5*5�ľ���
    H_cell{1,2}=zeros(Nu*Nc,1);%��һ�еڶ��е�λ��Ϊ5*1
    H_cell{2,1}=zeros(1,Nu*Nc);%�ڶ��е�һ�е�λ��Ϊ1*5
    H_cell{2,2}=Row;%�ڶ��еڶ���Ϊһ��ֵ���ɳ�����
    H=cell2mat(H_cell);%�������֮��HԪ������ת��Ϊ��������ΪHΪ6*6
    H=(H+H')/2;
    error_1=zeros(Ny*Np,1);%40*7*7*1=40*1
    Yita_ref_cell=cell(Np,1);%�ο���Ԫ������Ϊ20*1
%     
   
    for p=1:1:Np
        X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%��������ϵ�������ٶ�
        X_predict(p,1)=X+X_DOT*p*T;%���ȼ����δ��X��λ�ã�
        %�ο�·����Ϣ�����ο�������λ�ã�����λ�ã���ڽ�
        last=size(ref11,1);
        pt = ref11(:,1);
        if X_predict(p,1)>=ref11(last,2)
%             Y_ref(p,1)=path66(last,6);
%             phi_ref(p,1)=path66(last,7);
            Y_ref(p,1)=0;
            phi_ref(p,1)=0;
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
        else
            x_refer=ref11(:,2); %���õ�����λ��
            y_refer=ref11(:,3); %���õĺ���λ��
            phi_refer=ref11(:,4);
            Y_ref(p,1)=interp1(pt,y_refer,t+(i-1)*T);
            phi_ref(p,1)=interp1(pt,phi_refer,t+(i-1)*T);
            Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
        end
    end
    
    v_y_vector = ref11(:,7);
    yaw_rate_vector = ref11(:,5);
    phi_rate = interp1(pt,yaw_rate_vector,t);
    v_y = interp1(pt,v_y_vector,t);
    
    Yita_ref=cell2mat(Yita_ref_cell);%�����ǵõ������Ԫ��ת��Ϊ����
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; %��ƫ������й�ʽ��23�������У���ΪҪ������ι滮��f�������Ա���������
    g_cell=cell(1,2);%���Ǽ����Ĺ�ʽ��23�����档
    g_cell{1,1}=2*error_1'*Q*THETA;
    g_cell{1,2}=0;
    g=-cell2mat(g_cell);
    f=g';%���󲿷�һ�ɰ������ǵ�ppt�Ƶ�������ѧϰ
    
 %% ����ΪԼ����������
 %������Լ��
    A_t=zeros(Nc,Nc);%��falcone���� P181
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p %�����Ǿ�������Խ���
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%������ڿ˻�
    Ut=kron(ones(Nc,1),U(1));
    
    umin=-0.1744;%ά������Ʊ����ĸ�����ͬ��ǰ��ƫ�ǵ���Լ��
    umax=0.1744;%ǰ��ƫ�ǵ���Լ��
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    
    %�����Լ��
    ycmax=[0.3;6];  %��ڽǺͺ���λ�Ƶ�Լ��
    ycmin=[-0.3;0];
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);
    
    %���ι滮A����
    A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
    A_cons=cell2mat(A_cons_cell);%����ⷽ�̣�״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
    %���ι滮��b����
    b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};
    b_cons=cell2mat(b_cons_cell);%����ⷽ�̣�״̬������ʽԼ����ȡֵ
    
    %��������Լ��
    M=10; 
    delta_umin=-0.0148;%ǰ��ƫ�Ǳ仯������Լ��
    delta_umax=0.0148;%ǰ��ƫ�Ǳ仯������Լ��
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;0];%����ⷽ�̣�״̬���½磬��������ʱ���ڿ����������ɳ�����
    ub=[delta_Umax;M];%����ⷽ�̣�״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����
    
    %% ��ʼ������
%    options = optimset('Algorithm','interior-point-convex');
    options = optimset('Display','off', ...
        'TolFun', 1e-8, ...
        'MaxIter', 2000, ...
        'Algorithm', 'interior-point-convex', ...
        'FinDiffType', 'forward', ...
        'RelLineSrchBnd', [], ...
        'RelLineSrchBndDuration', 1, ...
        'TolConSQP', 1e-8);
    warning off all  % close the warnings during computation
    x_start=zeros(Nc+1,1);%����һ����ʼ��
    [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);
    fprintf('exitflag=%d\n',exitflag);
    fprintf('H=%4.2f\n',H(1,1));
    fprintf('f=%4.2f\n',f(1,1));
    %% �������
    if (1 ~= exitflag) %if optimization NOT succeeded.
                X(1) = 0.0;
                fprintf('MPC solver not converged!\n');                  
    end

    fprintf("X(1)=%4.2f\n",X(1));
    u_piao=X(1);%�õ���������
    U(1)=kesi(7,1)+u_piao;%��ǰʱ�̵Ŀ�����Ϊ��һ��ʱ�̿���+��������
    %U(1)=u_piao;
    OUT(1)=U(1);
    y_refers=interp1(pt,y_refer,t);
    x_refers=interp1(pt,x_refer,t);
    phi_refers=interp1(pt,phi_refer,t);
    OUT(2) =y_refers;
    OUT(3) = x_refers;
    OUT(4)=phi_refers*180/pi;
%%��������/0.000526260393642774
    OUT(5) = phi_rate*180/pi;
    OUT(6) = v_y;

    sys= OUT;
    toc
    
