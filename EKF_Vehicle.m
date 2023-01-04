    %data = readmatrix("mobility_0.csv")
load Xsaved.mat

count = 59900;
i_count = 1;
csv_number = 13;

Xsaved = zeros(count, 6);
realData = zeros(count, 2);
predictedData = zeros(count, 2);

for i = 1 : size(data, 1)
    if data(i, 2) == csv_number
        Xsaved(i_count, :) = data(i,:);
        i_count = i_count + 1;
    end
end

i_count = 1;    
for i = 1 : count
    
    

    if i == 1
        predictedData(i, 1) = Xsaved(i, 3);
        predictedData(i, 2) = Xsaved(i, 4);
        realData(i, 1) = Xsaved(i,3);
        realData(i, 2) = Xsaved(i,4);
    end
    if i ~= 1
        [pos_x, pos_y, distance] = SystemEKF(Xsaved,predictedData, i);
        predictedData(i, 1) = pos_x;
        predictedData(i, 2) = pos_y;
        realData(i, 1) = Xsaved(i,3);
        realData(i, 2) = Xsaved(i,4);
    end

end


subplot(2,1,1)
plot(predictedData(:,1),predictedData(:,2))
title('predictedData')

subplot(2,1,2)
plot(realData(:,1), realData(:,2))
title('realData')



err = immse(realData, predictedData)

function [pos_x, pos_y, distance] = SystemEKF(Xsaved,predictedData,number)

persistent H Q R
persistent x z P
persistent firstRun
persistent innov
persistent time
persistent numcount

z = zeros(3, 1);
z(1,1) = Xsaved(number, 3);
z(2,1) = Xsaved(number, 4);


if isempty(firstRun)
    var = 0.456;
    numcount = 1;
    innov = zeros(59900,3);
    time = 0.01;
    Q = [0.001 0 0;
        0 0.001 0;
        0 0 0.01];
    Q = Q * var;

    R = 100*eye(3);
    P = diag([1,1,0.1]);

    x = zeros(2,1);
    x(1,1) = Xsaved(1, 3);
    x(2,1) = Xsaved(1, 4);
    x(3,1) = Xsaved(1, 5);

    pos_x = x(1);
    pos_y = x(2);
    distance = 0;

    firstRun = 1;   
  
end

    if firstRun == 1
    
    

    arbiNum = number -1;

    a = randn;

    x = zeros(3,1);
    x(1, 1) = predictedData(arbiNum, 1) + time*Xsaved(arbiNum, 6)*cos(Xsaved(arbiNum, 5));
    x(2, 1) = predictedData(arbiNum, 2) + time*Xsaved(arbiNum, 6)*sin(Xsaved(arbiNum, 5));
    x(3,1) = 0;

    % x(t+1) = x(t) + distance*cos(theta);
    distance = time*Xsaved(arbiNum,6);
    A = Ajacob(Xsaved, number);
    %H = Hjacob(Xsaved, number, x, distance);
    H = diag([1,1,1]);
    %xp = A*x;
    
    
    %a = H*Pp*H' + R
    Pp = A*P*A' + Q;
    K = Pp*H'*inv(H*Pp*H' + R);
    
    %xp =x;
   
    xp = x;
    x = xp + K*(z - H*xp);
   
    
    P = Pp-K*H*Pp;
    pos_x = x(1);
    pos_y = x(2);   
    
    

    end
end

function A = Ajacob(Xsaved, number)

    A = zeros(3,3);
    
    distance = 0.01 * Xsaved(number -1, 6);

    A = zeros(3,3);
    theta = Xsaved(number-1, 5);
    speed = Xsaved(number-1, 6);
    A(1,1) = 1;
    A(1,2) = 0;
    %A(1,3) = cos(thet
    % a);
    A(1,3) = -distance*sin(theta);

    A(2,1) = 0;
    A(2,2) = 1;
    %A(2,3) = sin(theta);
    A(2,3) = distance*cos(theta);
    A(3,1) = 0;
    A(3,2) = 0;
    A(3,3) = 0;

end

function H = Hjacob(Xsaved, number,x, distance)

    theta_check = Xsaved(number, 5);

    H1_dx = -(Xsaved(number,3) - x(1) - distance*cos(theta_check))/((Xsaved(number,3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2)^(1/2);
    H1_dy = -(Xsaved(number,4) - x(2) - distance*sin(theta_check))/((Xsaved(number,3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2)^(1/2);
    dg1_dtheta = distance*(sin(theta_check)*(Xsaved(number,3) - x(1) - distance*cos(theta_check)) - cos(theta_check)*(Xsaved(number,4) - x(2) - distance*sin(theta_check)))/((Xsaved(number, 3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2)^(1/2);
            
    H2_dx = -(Xsaved(number,4) - x(2) - distance*sin(theta_check))/((Xsaved(number,3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2)^(1/2);
    H2_dy = -(Xsaved(number,3) - x(1) - distance*cos(theta_check))/((Xsaved(number,3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2)^(1/2);
    dg2_dtheta = -distance*sin(theta_check)*(Xsaved(number,4) - x(2) - distance*sin(theta_check))/((Xsaved(number,3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2) - distance*cos(theta_check)*(Xsaved(number,3) - x(1) - distance*cos(theta_check))/((Xsaved(number,3) - x(1) - distance*cos(theta_check))^2 + (Xsaved(number,4) - x(2) - distance*sin(theta_check))^2) - 1;
            
    H = [H1_dx H1_dy dg1_dtheta ; H2_dx H2_dy dg2_dtheta];

end