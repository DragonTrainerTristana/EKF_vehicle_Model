%data = readmatrix("mobility_0.csv")ds
count = 0;
i_count = 1;
k_count = 1;
for i = 1 : size(data, 1)
    if data(i,2) == 1
        count = count + 1;
    end
end

Xsaved = zeros(count, 6);
realData = zeros(count, 2);
predictedData = zeros(count, 3);

for i = 1 : size(data, 1)
    if data(i,2) == 3
       Xsaved(i_count, :) =  data(i,:);
       i_count = i_count + 1;
       
    end
end

%version

for i = 1 : size(Xsaved, 1) - 1
    [pos_x, pos_y, nothing] = SystemEKF(Xsaved, i);

    if i == 1
       predictedData(k_count, 1) = Xsaved(k_count,3);
       predictedData(k_count, 2) = Xsaved(k_count,4);
       predictedData(k_count, 3) = 0;
       
       realData(k_count, 1) = data(i, 3);
       realData(k_count, 2) = data(i, 4);
    end

       predictedData(k_count + 1, 1) = pos_x;
       predictedData(k_count + 1, 2) = pos_y;
       predictedData(k_count + 1, 3) = nothing;
       
       realData(k_count + 1, 1) = Xsaved(k_count + 1, 3);
       realData(k_count + 1, 2) = Xsaved(k_count + 1, 4);

       k_count = k_count + 1;

end

%{
for i = 1 : size(data, 1) -1
    if data(i,2) == 0
       Xsaved(i_count, :) =  data(i,:);
       [pos_x, pos_y, nothing] = SystemEKF(Xsaved(i_count, :));

       if i == 1
       predictedData(i_count, 1) = Xsaved(i_count,3);
       predictedData(i_count, 2) = Xsaved(i_count,4);
       predictedData(i_count, 3) = 0;
       
       realData(i_count, 1) = data(i, 3);
       realData(i_count, 2) = data(i, 4);
       end

       predictedData(i_count + 1, 1) = pos_x;
       predictedData(i_count + 1, 2) = pos_y;
       predictedData(i_count + 1, 3) = nothing;
       
       realData(i_count + 1, 1) = Xsaved(i_count + 1, 3);
       realData(i_count + 1, 2) = Xsaved(i_count + 1, 4);

       i_count = i_count + 1;
       
    end
end

%}
t = 0 : 0.1: count*0.01-0.01;

predictedData(:,3) = [];

figure
plot(realData(:,1), realData(:,2))
title('real Data')

figure
plot(predictedData(:,1), predictedData(:,2))
title('Predicted Data')

%figure
%plot(t, predictedData)

err = immse(realData, predictedData)



function [pos_x, pos_y, nothing] = SystemEKF(Xsaved, number)

persistent H Q R
persistent x P
persistent firstRun


theta = Xsaved(number, 5);

speed = Xsaved(number,6);
distance = 0.01*speed;

if isempty(firstRun)

    Q = [0.001 0 0;
        0 0.001 0;
        0 0 0.01];
    H = [1,1,0];
    R = 10;
    
    % row 1은 Pos_x, row 2는 pos_y, row 3는 distance가 되어야함
    x = [11116.09,5811.57,distance]'; % 임의 초깃값
    P = 10 * eye(3);

    firstRun = 1;
    pos_x = x(1);
    pos_y = x(2);
    nothing = x(3);
end

if firstRun == 1
A = Ajacob(Xsaved,number);
x(1,3) = distance;

    xp = A*x;
    Pp = A*P*A' + Q;

    z = zeros(1,3);
    z(1,1) = Xsaved(number + 1, 3); % pos_x 관측값 
    z(1,2) = Xsaved(number + 1, 4);% pos_y 관측값
    z(1,3) = 0;

   

    K = Pp*H'*inv(H*Pp*H' + R);

    x = xp + K*(z- H*xp);
    P = Pp - K*H*Pp;

    pos_x = x(1);
    pos_y = x(2);
    nothing = x(3);
end
end

function A = Ajacob(Xsaved, number)

A = zeros(3,3);
theta = Xsaved(number, 5);
speed = Xsaved(number, 6);
A(1,1) = 1;
A(1,2) = 0;
A(1,3) = -speed*sin(theta);
A(2,1) = 0;
A(2,2) = 1;
A(2,3) = speed*cos(theta);
A(3,1) = 0;
A(3,2) = 0;
A(3,3) = 0;

end