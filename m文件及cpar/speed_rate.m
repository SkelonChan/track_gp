function [sys,x0,str,ts] =speed_rate(t,x,u,flag)
%***************************************************************% 
%Carsim����������ٶȣ�������ٶ�
%Carsim���룺�����ſ��ȣ��ƶ�����ѹ��
%�����������ٶȿ��ƣ����ԶԼ��ٶȵı仯�ʽ���Լ��
%ǰһʱ�̵��������ٶ�+�������ٵļ��ٶȱ仯�����������ٶ�
%---------------------------------------------------------------%
%***************************************************************% 
    switch flag,
        case 0 % Initialization %
            [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
        case 2 % Update %
            sys = mdlUpdates(t,x,u); % Update discrete states
        case 3 % Outputs %
            sys = mdlOutputs(t,x,u); % Calculate outputs
        case {1,4,9} % Unused flags
            sys = [];            
        otherwise % Unexpected flags %
            error(['unhandled flag = ',num2str(flag)]); % Error handling
    end %  end of switch    
%  End sfuntmpl

%==============================================================
% Initialization, flag = 0��mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;%��������ģ������Ľṹ����simsizes������
sizes.NumContStates  = 0;  %ģ������״̬�����ĸ���
sizes.NumDiscStates  = 2;  %ģ����ɢ״̬�����ĸ������ٶȺͼ��ٶ�
sizes.NumOutputs     = 6;  %S��������������������������ź�ɲ���������������
sizes.NumInputs      = 2;  %S����ģ����������ĸ��� = CarSim�������
sizes.DirFeedthrough = 1;  %ģ���Ƿ����ֱ�ӹ�ͨ(direct feedthrough). 1 means there is direct feedthrough.
% ֱ����ͨ��ʾϵͳ�������ɱ����ʱ���Ƿ��ܵ�����Ŀ��ơ�
% a.  ���������mdlOutputs��flag==3��������u�ĺ����������������u��mdlOutputs�б����ʣ������ֱ����ͨ��
% b.  ����һ���䲽��S-Function�ġ���һ������ʱ�䡱������mdlGetTimeOfNextVarHit��flag==4���п��Է�������u��
% ��ȷ����ֱ����ͨ��־��ʮ����Ҫ�ģ���Ϊ��Ӱ��ģ���п��ִ��˳�򣬲����ü���������
sizes.NumSampleTimes = 1;  %ģ��Ĳ�������
sys = simsizes(sizes);    %������󸳸�sys���
x0 = zeros(sizes.NumDiscStates,1);%�趨��ʼ��״̬��
global UU; %�趨UΪȫ�ֱ���
UU=[0]; %��ʼ�Ŀ�����Ϊ0,0
str = [];             % ��������.
ts  = [0.01 0];       % ��������sample time=0.05,50ms ��������

%--Global parameters and initialization
global InitialGapflag; 
    InitialGapflag = 0; % the first few inputs don't count. Gap it.
global MPCParameters; 
    MPCParameters.Np      = 10;%Ԥ��ʱ��
    MPCParameters.Nc      = 5;%����ʱ��
    MPCParameters.Nx      = 2; %״̬������
    MPCParameters.Nu      = 1; %����������
    MPCParameters.Ny      = 1; %���������
    MPCParameters.Ts      = 0.01; %����ʱ��
    MPCParameters.Q       = 100; %״̬��Ȩ��ϵ��
    MPCParameters.R       = 1; %������Ȩ��ϵ��
    MPCParameters.refspeedT = 3; %����0: ���ɽ�Ծ���ٶ�����; 
                                 %����1:��������ʽ���ٶ�����
    MPCParameters.umin      = -8.0;  % �������ٶ�����
    MPCParameters.umax      = 5.5;  % �������ٶ�����
    MPCParameters.dumin     = -8.0; % �����Ӽ��ٶ�����
    MPCParameters.dumax     = 6.0; % �����Ӽ��ٶ�����
global WarmStart;
    WarmStart = zeros(MPCParameters.Np,1); %���ι滮���ĳ�ʼλ��
%  End of mdlInitializeSizes

%==============================================================
% ������ɢ״̬��
%==============================================================
function sys = mdlUpdates(t,x,u)
%  ����û���õ�������̣��ں��ڵĳ���ģ�黯ʱ���Լ�������������ܡ�
    sys = x;    
% End of mdlUpdate.

%==============================================================
% flag = 3���������
%==============================================================
function sys = mdlOutputs(t,x,u)
global UU
global InitialGapflag;
global MPCParameters;
global WarmStart;
Vx    =10;  %�����ٶȼ���һ��״̬��m/s
a_x   = 0;  %������ٶȼ��ڶ���״̬��
a_des = 0;  %�������ٶȣ�Ҳ����������Ҫ���
UU=a_des;
t_Start = tic; % ��ʼ��ʱ 
if InitialGapflag < 2 %ȥ��ǰ��������
    InitialGapflag = InitialGapflag + 1;%
else
    InitialGapflag = InitialGapflag + 1;
    %***********Step (1).���³���״̬*************************% 
    Vx   = u(1)/3.6;  %���������ٶȣ�ǧ��ÿСʱ����Ϊ��ÿ��
    a_x  = u(2)*9.8;  %����������ٶȣ�g����Ϊ��ÿ���η��룬��Ϊ1��g=9.8 m/s2
    Nx=MPCParameters.Nx;
    Nu=MPCParameters.Nu;
    kesi=zeros(Nx+Nu,1); %�����µ�״̬������ ��3*1ά����
    kesi(1)=Vx;%kʱ�̵��ٶ�
    kesi(2)=a_x;%kʱ�̵ļ��ٶ�
    kesi(3)=UU(1);%k-1ʱ�̵��������ٶ�

    %********Step(2): ���ɲο����ٶȹ켣 *******************%
    switch MPCParameters.refspeedT,
        case 0 % default, step speed profile
            %----���ɽ���ʽ�������ٶ�����----------------------%
            SpeedProfile = func_ConstantSpeed(InitialGapflag, MPCParameters);
        case 1 % sine-wave speed profile
            %----�������Ҳ���ʽ�������ٶ�����--------------%
            SpeedProfile = func_SineSpeed(InitialGapflag,MPCParameters);
        case 3 
            SpeedProfile = func_speed(InitialGapflag, MPCParameters);
            %�����ٶ�
        otherwise % Unexpected flags %
            error(['unexpected speed-profile:',num2str(MPCParameters.refspeedT)]); % Error handling
    end %  end of switch 
    
    %****Step(3): ����״̬�ռ����A B �� C***%
    Ts = MPCParameters.Ts; % 50ms   %�������ڸ�ֵ
    K=1; %ϵͳ����
    tod=0.1; %ʱ�䳣��
    a =  [1 Ts;0 1-K*Ts/tod];  %������ɢ�����a���� ��ʱ�䳣��=����ʱ��
    b =  [0;K*Ts/tod];  %������ɢ�����b����
    A_cell=cell(2,2); % ����2*2��Ԫ������
    A_cell{1,1}=a;  %��a����ŵ�A_cell�ĵ�һ�е�һ��λ��
    A_cell{1,2}=b;  %��b����ŵ�A_cell�ĵ�һ�еڶ���λ��
    A_cell{2,1}=zeros(Nu,Nx); %��2*3�������ŵ�A_cell�ڶ��еĵ�һ��λ��
    A_cell{2,2}=eye(Nu); %��2*2�ĵ�λ��ŵ�A_cell�ڶ��еĵڶ���λ��
    
    B_cell=cell(2,1); %����2*1��Ԫ������
    B_cell{1,1}=b;%��b����ŵ�B_cell�ĵ�һ��
    B_cell{2,1}=eye(Nu);%��2*2�ĵ�λ��ŵ�B_cell�ڶ���
    
    A=cell2mat(A_cell); %�����A�����������Ƶ���һʱ�̵�״̬�ռ�ʱ���A 
    B=cell2mat(B_cell); %�����B�����������Ƶ���һʱ�̵�״̬�ռ�ʱ���B
    C=[1 0 0];%ֻ����ٶ�
    StateSpaceModel.A=A;
    StateSpaceModel.B=B;
    StateSpaceModel.C=C;
    
    %****Step(4):���MPC����ؾ���********************%
    %����Theta PHI ���󣬸���״̬�ռ�����Լ�MPC�Ĳ��������
    [PHI, THETA] = func_Update_PHI_THETA(StateSpaceModel, MPCParameters);

    %������ι滮�е�H f���󣬸���״̬���������ٶȹ켣��PHI��Theta�Լ�MPC�������
    [H, f, g] = func_Update_H_f(kesi, SpeedProfile, PHI, THETA, MPCParameters); 
    
    %****Step(5):���ι滮���********************%
    [A, b, Aeq, beq, lb, ub] = func_Constraints_du_quadprog(MPCParameters, UU(1));%���ô˺��������Լ������
    options = optimset('Display','off', ...
        'TolFun', 1e-3, ...
        'MaxIter', 50, ...
        'Algorithm', 'interior-point-convex', ...
        'FinDiffType', 'forward', ...
        'RelLineSrchBnd', [], ...
        'RelLineSrchBndDuration', 1, ...
        'TolConSQP', 1e-8);
    warning off all  %�ڼ�������йر����о���
    
    UU0 = WarmStart;
    [X, FVAL, EXITFLAG] = quadprog(H, g, A, b, Aeq, beq, lb, ub, UU0, options); %
    WarmStart = shiftHorizon(X);     % Prepare restart, nominal close loop
    if (1 ~= EXITFLAG) %���û��������Ž�
        X(1) = 0.0;
        fprintf('MPC solver not converged!\n');
    end
    UU(1)=kesi(3)+ X(1);%ǰһʱ�̵��������ٶ�+�������ٵļ��ٶȱ仯��
    a_des =  UU(1);    
end % end of if Initialflag < 1 % 

    %****Step(6):  �������ļ��ٶ�����Throttle��Brake;********************%
    [Throttle, Brake] = func_AccelerationTrackingController(a_des);

t_Elapsed = toc( t_Start ); %computation time 

sys = [Throttle; Brake;t_Elapsed; Vx; a_x; a_des]; %���ţ��ƶ���ʱ�䣬�ٶȣ����ٶȣ��������ٶ�
% end  %End of mdlOutputs.

%==============================================================
% sub functions
%============================================================== 
%����������ʽ�������ٶ�����
function [Vref] = func_SineSpeed(Index, MPCParameters)

    %****Sine wave parameters
    T = 5; %�����ٶ����ߵ����ڣ�unit: s
    freq = 1/T; %�����ٶ����ߵ�Ƶ�ʣ�unit: Hz
    Amplit = 10;%�����ٶ����ߵķ�ֵ
    offst = 20; %�����ٶ����ߵ�ƫ��
    
    Ts = MPCParameters.Ts; %����ʱ��=0.05��unit: s
    Np = MPCParameters.Np; % Ԥ��ʱ��30
    Vref = cell(Np,1);
    t0 = Index*Ts;
    
    for i = 1:1:Np
        t = t0 + i*Ts;
        Vref{i,1}   =   Amplit*sin(2*pi*freq*t) + offst;   
    end
    
function [Vref] = func_speed(Index, MPCParameters)
%     load ref44
%     pt = path5(:,4);
%     v_ref=path5(:,8);
     load path88 %%vx35-55km/h
%    load path150 %%129--198km/h
    pt = path5(:,6);
    v_ref = path5(:,7);
    
    Ts = MPCParameters.Ts; %����ʱ��=0.05��unit: s
    Np = MPCParameters.Np; % Ԥ��ʱ��30
    Vref = cell(Np,1);
    t0 = Index*Ts;
    
    for i = 1:1:Np
        t = t0 + i*Ts;
        Vref{i,1}   =   interp1(pt,v_ref,t);   
    end


% end %EoF
% ���ɽ�����ʽ�������ٶ����� 
function [Vref] = func_ConstantSpeed(InitialGapflag, MPCParameters)
   
    Ts = MPCParameters.Ts; %����ʱ��=0.05��unit: s
    Np = MPCParameters.Np; % Ԥ��ʱ��30
    Vref = cell(Np,1);
    
    % �Զ�����ݵ���ʽ
    if InitialGapflag < 400
        Vset = 10;
    else
        if InitialGapflag < 800
            Vset = 10;
        else
            if InitialGapflag < 1500
                Vset = 20;
            else
                Vset = 5;
            end
        end
    end

    for i = 1:1:Np
        Vref{i,1}  =  Vset;   
    end

% end %EoF
% ������λ���������������ٶ�ת��Ϊ���ſ��������ƶ�����ѹ��������
function [Throttle, Brake] = func_AccelerationTrackingController(ahopt)

    K_brake         = 0.4;%�������ٶ� ������ѹ����ת������
    K_throttle      = 0.3; %0.05���������ٶȵ����ſ�������ת������
    Brake_Sat       = 25; %�ƶ�����ѹ���ı���ֵ
    Throttle_Sat    = 1; %���ŵı���ֵ

    if ahopt < 0 % Brake control
        Brake = K_brake * ahopt;
        if Brake > Brake_Sat
            Brake = Brake_Sat;
        end
        Throttle = 0;
    else % throttle control 
        Brake       = 0;
        Throttle    = K_throttle *ahopt;
        if Throttle > Throttle_Sat
            Throttle = Throttle_Sat;
        end
        if Throttle < 0
            Throttle = 0;
        end
    end
% end %EoF

function u0 = shiftHorizon(u) %shift control horizon
    u0 = [u(:,2:size(u,2)), u(:,size(u,2))];  %  size(u,2))
    
function [PHI, THETA] = func_Update_PHI_THETA(StateSpaceModel, MPCParameters)
%***************************************************************%
% Ԥ��������ʽ Y(t)=PHI*kesi(t)+THETA*DU(t) 
% Y(t) = [Eta(t+1|t) Eta(t+2|t) Eta(t+3|t) ... Eta(t+Np|t)]'
%***************************************************************%
    Np = MPCParameters.Np;
    Nc = MPCParameters.Nc;
    Nx = MPCParameters.Nx;
    Ny = MPCParameters.Ny;
    Nu = MPCParameters.Nu;
    A = StateSpaceModel.A;
    B = StateSpaceModel.B;
    C = StateSpaceModel.C;

    PHI_cell=cell(Np,1);                            %PHI=[CA CA^2  CA^3 ... CA^Np]' 
    THETA_cell=cell(Np,Nc);                         %THETA
    for j=1:1:Np
        PHI_cell{j,1}=C*A^j;                       %  demision:Ny* Nx
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=C*A^(j-k)*B;        %  demision:Ny*Nu
            else 
                THETA_cell{j,k}=zeros(Ny,Nu);
            end
        end
    end
    PHI=cell2mat(PHI_cell);    % size(PHI)=[(Ny*Np) * Nx]
    THETA=cell2mat(THETA_cell);% size(THETA)=[Ny*Np Nu*Nc]
% end %EoF


function[H, f, g] = func_Update_H_f(kesi, SpeedProfile, PHI, THETA, MPCParameters)
%***************************************************************%
% trajectory planning
%***************************************************************%
    Np = MPCParameters.Np;
    Nc = MPCParameters.Nc;   
    Q  = MPCParameters.Q;
    R  = MPCParameters.R;
        
    Qq = kron(eye(Np),Q);  %           Q = [Np*Nx] *  [Np*Nx] 
    Rr = kron(eye(Nc),R);  %           R = [Nc*Nu] *  [Nc*Nu]

    Vref = cell2mat(SpeedProfile); 
    error = PHI * kesi;    %[(Nx*Np) * 1]

    H = 2*(THETA'*Qq*THETA + Rr);  
    f = 2*(error' - Vref')*Qq*THETA;
    g = f';
% end %EoF

function  [A, b, Aeq, beq, lb, ub] = func_Constraints_du_quadprog(MPCParameters, U)
%************************************************************************%
% generate the constraints of the vehicle
%  
%************************************************************************%
    Np   = MPCParameters.Np;
    Nc   = MPCParameters.Nc;    
    dumax = MPCParameters.dumax;
    dumin = MPCParameters.dumin;
    umin = MPCParameters.umin;  
    umax = MPCParameters.umax;  
    Umin = kron(ones(Nc,1),umin);
    Umax = kron(ones(Nc,1),umax);
    Ut   = kron(ones(Nc,1),U);
%----(1) A*x<=b----------%
    A_t=zeros(Nc,Nc);
    for p=1:1:Nc
        for q=1:1:Nc
            if p >= q 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_cell=cell(2,1);
    A_cell{1,1} = A_t; %
    A_cell{2,1} = -A_t;
    A=cell2mat(A_cell);  %
    
    
    b_cell=cell(2,1);
    b_cell{1,1} = Umax - Ut; %
    b_cell{2,1} = -Umin + Ut;
    b=cell2mat(b_cell);  % 

%----(2) Aeq*x=beq----------%
    Aeq = [];
    beq = [];

%----(3) lb=<x<=ub----------%
    lb=kron(ones(Nc,1),-dumax);
    ub=kron(ones(Nc,1),dumax);
% end %EoF
