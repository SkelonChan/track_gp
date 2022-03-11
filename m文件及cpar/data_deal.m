clear all
clc


load("fr05_007hz.mat");
ref = ref.data;
real = real.data;
ref_vy = ref(1:10:end,1);
ref_vx = ref(1:10:end,2);
ref_yaw = ref(1:10:end,3);
ref_yaw_rate = ref(1:10:end,4);
ref_y = ref(1:10:end,5);
ref_x = ref(1:10:end,6);
delta = ref(1:10:end,7);
throttle = ref(1:10:end,8);
brake = ref(1:10:end,9);

real_vy = real(1:10:end,1);
real_vx = real(1:10:end,2);
real_yaw = real(1:10:end,3);
real_yaw_rate = real(1:10:end,4);
real_y = real(1:10:end,5);
real_x = real(1:10:end,6);

delta_vy = real_vy - ref_vy;
delta_vx = real_vx - ref_vx;
delta_yaw = real_yaw - ref_yaw;
delta_yaw_rate = real_yaw_rate - ref_yaw_rate;
delta_y = real_y - ref_y;
delta_x = real_x - ref_x;

MY = [delta_vy, delta_vx, delta_yaw, delta_yaw_rate, delta_y];
MX = [ref_vy, ref_vx, ref_yaw, ref_yaw_rate, ref_y, delta, throttle, brake];

MGP = MRSM(MX, MY);




