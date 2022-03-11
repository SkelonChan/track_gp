function [sys,x0,str,ts] = gpfunction_mean(t,x,u,flag)
%
switch flag,
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 2,
    sys=mdlUpdate(t,x,u);
  case 3,
    sys=mdlOutputs(t,x,u);
  case {1,4,9}%%无用标志位
    sys=[];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  =8;
sizes.NumOutputs     = 5;%%8
sizes.NumInputs      = 9;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);
x0  = [0.001;0.0001;0.0001;0.00001;0.00001;0.00001;0.00001;0.00001];

str = [];
T_all=10;%总的仿真时间
global times;  %%次数计次
times = 0;
global all_trained_y ;%%存贮每一步的状态量为GPR收集数据
all_trained_y = zeros(T_all/0.01,3);%%存贮每一步的状态量为GPR收集数据
global all_trained_x;%%存贮每一步的状态量为GPR收集数据
all_trained_x = zeros(T_all/0.01,5);%%存贮每一步的状态量为GPR收集数据
% initialize the array of sample times
%1
ts  = [0.01 0];
function sys=mdlUpdate(t,x,u)

sys = x;

function sys=mdlOutputs(t,x,u)
global times;
times = times+1;
global all_trained_x;
global all_trained_y;
    delta_y_dot=u(1); %横向速度化为m/s
    delta_x_dot=u(2);%CarSim输出的是km/h，转换为m/s
    delta_yaw_rate=u(3)*3.141592654/180;%角速度
    
    ref_y_dot = u(4);
    ref_x_dot = u(5);
    ref_phi_rate = u(6);

    control_delta = u(7);
    control_acc = u(8);
    delta_y = u(9);
%     ref_yaw = u(10);

    
%     train_x = [ref_y_dot,ref_x_dot,ref_phi_rate,control_delta,control_acc];
%     train_y = [delta_y_dot,delta_x_dot,delta_yaw_rate];
%     test_x = train_x; %%此处仅表示对应关系，实际中test_x为下一时刻的值
    %% generate GPR
    fprintf("当前次数为：%d\n",times);
    if times>1%舍去第一次数值

%         all_trained_y = zeros(T_all/0.01,3);%%存贮每一步的状态量为GPR收集数据
%         %%error 每次运行all_trained matrix 都被重置了************************
        %%Global变量-->对存贮矩阵和计次变量times
        all_trained_y(times,1) = delta_y_dot;
        all_trained_y(times,2) = delta_x_dot;
        all_trained_y(times,3) = delta_yaw_rate;
        all_trained_y(times,4) = delta_y;
        
%         all_trained_x = zeros(T_all/0.01,5);%%存贮每一步的参考状态量
        all_trained_x(times,1) = ref_y_dot;
        all_trained_x(times,2) = ref_x_dot;
        all_trained_x(times,3) = ref_phi_rate;
        all_trained_x(times,4) = control_delta;
        all_trained_x(times,5) = control_acc;
%         all_trained_x(times,6) = ref_yaw;
    end
    mean = [];
   %%  GPR
     if times >30
         gpt =7;%%回归区间
         trained_x= zeros(gpt,5);
         trained_y= zeros(gpt,4);
         for i=1:1:gpt
             %训练集
             trained_x(i,:) = all_trained_x(times-gpt+i-1,:);%将最近的25次真实状态量加载到gp_states中
             trained_y(i,:) = all_trained_y(times-gpt-1+i,:);%将最近的25次参考状态量加载到gp_states中
         end
         %%%测试集输入
         gp_test_x = [ ref_y_dot, ref_x_dot, ref_phi_rate, control_delta, control_acc];

%          meanfunc = @meanConst;
% %          covfunc = @covRQiso;
%         covfunc = @covSEard;
%          likfunc = @likGauss;
%          hyp = struct('mean',0, 'cov', [0 0 0 0 0 0], 'lik', -1);%%初始化超参数
%          hyp2 = minimize(hyp, @gp, -100, @infGaussLik, meanfunc, covfunc, likfunc,trained_x,trained_y);%%超参数优化
%          [gpmean,gpcov] = gp(hyp2, @infGaussLik, meanfunc, covfunc, likfunc,trained_x,trained_y, gp_test_x);
%          mean = [gpmean,gpcov];
        gmean=[0,0,0,0];
        gcov = [0];
        MGP=MRSM(trained_x,trained_y);
        [gmean,gcov] = predict_resp(MGP,gp_test_x);
        mean =[gmean,gcov(4,4)];%%取y-y的相关
     end
%% end generate GPR_mean;   
    
sys = mean;
% end mdlOutputs

