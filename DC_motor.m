function [ out ] = DC_motor(u)


v = u(1);
theta_dot = u(2);
i = u(3);


J = 0.02; % 
b = 0.05;
Ke = 0.1; 
Kt = 0.2;
R = 3;
L = 0.5;
theta_double_dot = (Kt *i - b * theta_dot)/J;
i_dot = (v - Ke * theta_dot - R * i) / L;

out = [theta_double_dot,i_dot];

end

