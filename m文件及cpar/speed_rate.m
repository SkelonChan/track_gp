function [sys,x0,str,ts] =speed_rate(t,x,u,flag)
%***************************************************************% 
%Carsim输出：纵向速度；纵向加速度
%Carsim输入：节气门开度；制动主缸压力
%控制增量版速度控制：可以对加速度的变化率进行约束
%前一时刻的期望加速度+即将来临的加速度变化量＝期望加速度
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
% Initialization, flag = 0，mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;%用于设置模块参数的结构体用simsizes来生成
sizes.NumContStates  = 0;  %模块连续状态变量的个数
sizes.NumDiscStates  = 2;  %模块离散状态变量的个数即速度和加速度
sizes.NumOutputs     = 6;  %S函数的输出，包括控制量（油门和刹车）和其它监测量
sizes.NumInputs      = 2;  %S函数模块输入变量的个数 = CarSim的输出量
sizes.DirFeedthrough = 1;  %模块是否存在直接贯通(direct feedthrough). 1 means there is direct feedthrough.
% 直接馈通表示系统的输出或可变采样时间是否受到输入的控制。
% a.  输出函数（mdlOutputs或flag==3）是输入u的函数。即，如果输入u在mdlOutputs中被访问，则存在直接馈通。
% b.  对于一个变步长S-Function的“下一个采样时间”函数（mdlGetTimeOfNextVarHit或flag==4）中可以访问输入u。
% 正确设置直接馈通标志是十分重要的，因为它影响模型中块的执行顺序，并可用检测代数环。
sizes.NumSampleTimes = 1;  %模块的采样次数
sys = simsizes(sizes);    %设置完后赋给sys输出
x0 = zeros(sizes.NumDiscStates,1);%设定初始的状态量
global UU; %设定U为全局变量
UU=[0]; %初始的控制量为0,0
str = [];             % 保留参数.
ts  = [0.01 0];       % 采样周期sample time=0.05,50ms 连续采样

%--Global parameters and initialization
global InitialGapflag; 
    InitialGapflag = 0; % the first few inputs don't count. Gap it.
global MPCParameters; 
    MPCParameters.Np      = 10;%预测时域
    MPCParameters.Nc      = 5;%控制时域
    MPCParameters.Nx      = 2; %状态量个数
    MPCParameters.Nu      = 1; %控制量个数
    MPCParameters.Ny      = 1; %输出量个数
    MPCParameters.Ts      = 0.01; %采样时间
    MPCParameters.Q       = 100; %状态量权重系数
    MPCParameters.R       = 1; %控制量权重系数
    MPCParameters.refspeedT = 3; %换成0: 生成阶跃的速度曲线; 
                                 %换成1:生成正弦式的速度曲线
    MPCParameters.umin      = -8.0;  % 期望加速度下限
    MPCParameters.umax      = 5.5;  % 期望加速度上限
    MPCParameters.dumin     = -8.0; % 期望加加速度下限
    MPCParameters.dumax     = 6.0; % 期望加加速度上限
global WarmStart;
    WarmStart = zeros(MPCParameters.Np,1); %二次规划求解的初始位置
%  End of mdlInitializeSizes

%==============================================================
% 更新离散状态量
%==============================================================
function sys = mdlUpdates(t,x,u)
%  基本没有用到这个过程；在后期的程序模块化时可以继续开发这个功能。
    sys = x;    
% End of mdlUpdate.

%==============================================================
% flag = 3，计算输出
%==============================================================
function sys = mdlOutputs(t,x,u)
global UU
global InitialGapflag;
global MPCParameters;
global WarmStart;
Vx    =10;  %纵向速度即第一个状态量m/s
a_x   = 0;  %纵向加速度即第二个状态量
a_des = 0;  %期望加速度，也就是我们需要求的
UU=a_des;
t_Start = tic; % 开始计时 
if InitialGapflag < 2 %去掉前两个输入
    InitialGapflag = InitialGapflag + 1;%
else
    InitialGapflag = InitialGapflag + 1;
    %***********Step (1).更新车辆状态*************************% 
    Vx   = u(1)/3.6;  %车辆纵向速度，千米每小时换算为米每秒
    a_x  = u(2)*9.8;  %车辆纵向加速度，g换算为米每二次方秒，因为1个g=9.8 m/s2
    Nx=MPCParameters.Nx;
    Nu=MPCParameters.Nu;
    kesi=zeros(Nx+Nu,1); %构造新的状态量矩阵 ，3*1维矩阵
    kesi(1)=Vx;%k时刻的速度
    kesi(2)=a_x;%k时刻的加速度
    kesi(3)=UU(1);%k-1时刻的期望加速度

    %********Step(2): 生成参考的速度轨迹 *******************%
    switch MPCParameters.refspeedT,
        case 0 % default, step speed profile
            %----生成阶梯式的期望速度曲线----------------------%
            SpeedProfile = func_ConstantSpeed(InitialGapflag, MPCParameters);
        case 1 % sine-wave speed profile
            %----设置正弦波形式的期望速度曲线--------------%
            SpeedProfile = func_SineSpeed(InitialGapflag,MPCParameters);
        case 3 
            SpeedProfile = func_speed(InitialGapflag, MPCParameters);
            %跟踪速度
        otherwise % Unexpected flags %
            error(['unexpected speed-profile:',num2str(MPCParameters.refspeedT)]); % Error handling
    end %  end of switch 
    
    %****Step(3): 定义状态空间矩阵A B 及 C***%
    Ts = MPCParameters.Ts; % 50ms   %采样周期赋值
    K=1; %系统增益
    tod=0.1; %时间常数
    a =  [1 Ts;0 1-K*Ts/tod];  %线性离散化后的a矩阵 令时间常数=采样时间
    b =  [0;K*Ts/tod];  %线性离散化后的b矩阵
    A_cell=cell(2,2); % 构建2*2的元胞数组
    A_cell{1,1}=a;  %将a矩阵放到A_cell的第一行第一个位置
    A_cell{1,2}=b;  %将b矩阵放到A_cell的第一行第二个位置
    A_cell{2,1}=zeros(Nu,Nx); %将2*3的零矩阵放到A_cell第二行的第一个位置
    A_cell{2,2}=eye(Nu); %将2*2的单位阵放到A_cell第二行的第二个位置
    
    B_cell=cell(2,1); %构建2*1的元胞数组
    B_cell{1,1}=b;%将b矩阵放到B_cell的第一行
    B_cell{2,1}=eye(Nu);%将2*2的单位阵放到B_cell第二行
    
    A=cell2mat(A_cell); %这里的A就是我们在推导下一时刻的状态空间时候的A 
    B=cell2mat(B_cell); %这里的B就是我们在推导下一时刻的状态空间时候的B
    C=[1 0 0];%只输出速度
    StateSpaceModel.A=A;
    StateSpaceModel.B=B;
    StateSpaceModel.C=C;
    
    %****Step(4):求解MPC的相关矩阵********************%
    %计算Theta PHI 矩阵，根据状态空间矩阵以及MPC的参数来求解
    [PHI, THETA] = func_Update_PHI_THETA(StateSpaceModel, MPCParameters);

    %计算二次规划中的H f矩阵，根据状态量，期望速度轨迹，PHI，Theta以及MPC参数求解
    [H, f, g] = func_Update_H_f(kesi, SpeedProfile, PHI, THETA, MPCParameters); 
    
    %****Step(5):二次规划求解********************%
    [A, b, Aeq, beq, lb, ub] = func_Constraints_du_quadprog(MPCParameters, UU(1));%调用此函数来求解约束部分
    options = optimset('Display','off', ...
        'TolFun', 1e-3, ...
        'MaxIter', 50, ...
        'Algorithm', 'interior-point-convex', ...
        'FinDiffType', 'forward', ...
        'RelLineSrchBnd', [], ...
        'RelLineSrchBndDuration', 1, ...
        'TolConSQP', 1e-8);
    warning off all  %在计算过程中关闭所有警告
    
    UU0 = WarmStart;
    [X, FVAL, EXITFLAG] = quadprog(H, g, A, b, Aeq, beq, lb, ub, UU0, options); %
    WarmStart = shiftHorizon(X);     % Prepare restart, nominal close loop
    if (1 ~= EXITFLAG) %如果没有求得最优解
        X(1) = 0.0;
        fprintf('MPC solver not converged!\n');
    end
    UU(1)=kesi(3)+ X(1);%前一时刻的期望加速度+即将来临的加速度变化率
    a_des =  UU(1);    
end % end of if Initialflag < 1 % 

    %****Step(6):  由期望的加速度生成Throttle和Brake;********************%
    [Throttle, Brake] = func_AccelerationTrackingController(a_des);

t_Elapsed = toc( t_Start ); %computation time 

sys = [Throttle; Brake;t_Elapsed; Vx; a_x; a_des]; %油门，制动，时间，速度，加速度，期望加速度
% end  %End of mdlOutputs.

%==============================================================
% sub functions
%============================================================== 
%生成正弦形式的期望速度曲线
function [Vref] = func_SineSpeed(Index, MPCParameters)

    %****Sine wave parameters
    T = 5; %正弦速度曲线的周期，unit: s
    freq = 1/T; %正弦速度曲线的频率，unit: Hz
    Amplit = 10;%正弦速度曲线的幅值
    offst = 20; %正弦速度曲线的偏移
    
    Ts = MPCParameters.Ts; %采样时间=0.05，unit: s
    Np = MPCParameters.Np; % 预测时域：30
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
    
    Ts = MPCParameters.Ts; %采样时间=0.05，unit: s
    Np = MPCParameters.Np; % 预测时域：30
    Vref = cell(Np,1);
    t0 = Index*Ts;
    
    for i = 1:1:Np
        t = t0 + i*Ts;
        Vref{i,1}   =   interp1(pt,v_ref,t);   
    end


% end %EoF
% 生成阶梯形式的期望速度曲线 
function [Vref] = func_ConstantSpeed(InitialGapflag, MPCParameters)
   
    Ts = MPCParameters.Ts; %采样时间=0.05，unit: s
    Np = MPCParameters.Np; % 预测时域：30
    Vref = cell(Np,1);
    
    % 自定义阶梯的形式
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
% 车辆下位控制器将期望加速度转化为油门控制量和制动主缸压力控制量
function [Throttle, Brake] = func_AccelerationTrackingController(ahopt)

    K_brake         = 0.4;%期望加速度 到主缸压力的转换比例
    K_throttle      = 0.3; %0.05，期望加速度到油门控制量的转化比例
    Brake_Sat       = 25; %制动主缸压力的饱和值
    Throttle_Sat    = 1; %油门的饱和值

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
% 预测输出表达式 Y(t)=PHI*kesi(t)+THETA*DU(t) 
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
