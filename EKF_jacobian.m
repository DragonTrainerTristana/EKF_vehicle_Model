clear
clc

syms x_1 y_1 x_2 y_2

F1 = sqrt((x_2 - x_1)^2 + (y_2 - y_1)^2);
F2 = atan2((y_2 - y_1),(x_2 - x_1));

%F = [F1 ; F2];

%J = jacobian(F, [x_1 y_1 x_2 y_2]);
%J