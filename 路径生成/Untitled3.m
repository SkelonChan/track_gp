%%x<=20,则
y = 0;
phi = 0;
phi_rate = 0;
%% x20--60
y = 3.5*(10*(t/d).^3-15*(t/d).^4+6*(t/d).^5);
phi = (21*t.^4)/20480000 - (21*t.^3)/256000 + (21*t.^2)/12800;
phi_rate = (21*t.^3)/5120000 - (63*t.^2)/256000 + (21*t)/6400;
%% x  60-100
y = 3.5;
phi = 0;
phi_rate = 0;
%% x  120-160
y = -3.5*(10*(t/d).^3-15*(t/d).^4+6*(t/d).^5)+3.5;
phi =  - (21*t.^4)/20480000 + (21*t.^3)/256000 - (21*t.^2)/12800;
phi_rate =  - (21*t.^3)/5120000 + (63*t.^2)/256000 - (21*t)/6400;
%% else 
y = 0;
phi = 0;
phi_rate = 0;