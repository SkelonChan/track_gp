t = [0:0.01:40];
d= 40;
y_ref = 3.5*(10*((t)/d).^3-15*((t)/d).^4+6*((t)/d).^5);
phi_dot2 = (21*t.^3)/5120000 - (63*t.^2)/256000 + (21*t)/6400;
cur = abs((21*t.^3)/5120000 - (63*t.^2)/256000 + (21*t)/6400)/(sqrt(1+((21*t.^4)/20480000 - (21*t.^3)/256000 + (21*t.^2)/12800).^2).^3)


hold on;
plot(t,cur);