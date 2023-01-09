clc
clear
load Xsaved.mat


count = 59900; % 59900 rows per id
mobilityNum = 35; % id num 13
observationNum = 14; % id num 14

ranNum = zeros(1,10);

% Generate Random Number from 1 to 504
for i = 1 : 10
    ranNum(1,i) = randi([1, 504]);
    if(i ~= 1)
        for k = 1 : i-1
            if(ranNum(1,k) == ranNum(1,i))
                i = i - 1; %removing duplicates
            end
        end
    end
end

%-----------
mobNum = zeros(1, 10);
for i = 1 : 10
    mobNum(1, i) = ranNum(1,i);
end

%-----------
mobilityArray1 = zeros(count, 6);
mobilityArray2 = zeros(count, 6);
mobilityArray3 = zeros(count, 6);
mobilityArray4 = zeros(count, 6);
mobilityArray5 = zeros(count, 6);
mobilityArray6 = zeros(count, 6);
mobilityArray7 = zeros(count, 6);
mobilityArray8 = zeros(count, 6);
mobilityArray9 = zeros(count, 6);
mobilityArray10 = zeros(count, 6);

%-----------
predictedData = zeros(count ,20);
realData = zeros(count ,20);
mobCount = ones(1,10);

preNum = 1;
realNum = 1;
%-----------


for i = 1 : size(data, 1)
    if data(i, 2) == mobNum(1,1)
        mobilityArray1(mobCount(1,1), :) = data(i,:);
        mobCount(1,1) = mobCount(1,1) + 1;
    end
    if data(i, 2) == mobNum(1,2)
        mobilityArray2(mobCount(1,2), :) = data(i,:);
        mobCount(1,2) = mobCount(1,2) + 1;
    end
    if data(i, 2) == mobNum(1,3)
        mobilityArray3(mobCount(1,3), :) = data(i,:);
        mobCount(1,3) = mobCount(1,3) + 1;
    end
    if data(i, 2) == mobNum(1,4)
        mobilityArray4(mobCount(1,4), :) = data(i,:);
        mobCount(1,4) = mobCount(1,4) + 1;
    end
    if data(i, 2) == mobNum(1,5)
        mobilityArray5(mobCount(1,5), :) = data(i,:);
        mobCount(1,5) = mobCount(1,5) + 1;
    end
    if data(i, 2) == mobNum(1,6)
        mobilityArray6(mobCount(1,6), :) = data(i,:);
        mobCount(1,6) = mobCount(1,6) + 1;
    end
    if data(i, 7) == mobNum(1,7)
        mobilityArray7(mobCount(1,7), :) = data(i,:);
        mobCount(1,7) = mobCount(1,7) + 1;
    end
    if data(i, 2) == mobNum(1,8)
        mobilityArray8(mobCount(1,8), :) = data(i,:);
        mobCount(1,8) = mobCount(1,8) + 1;
    end
    if data(i, 2) == mobNum(1,9)
        mobilityArray9(mobCount(1,9), :) = data(i,:);
        mobCount(1,9) = mobCount(1,9) + 1;
    end
    if data(i, 2) == mobNum(1,10)
        mobilityArray10(mobCount(1,10), :) = data(i,:);
        mobCount(1,10) = mobCount(1,10) + 1;
    end
end


%----------- Kalman Filter Algorithm
for i = 1 : 100 : count % count => 59900

    realData(realNum,1) = mobilityArray1(i, 3);
    realData(realNum,2) = mobilityArray1(i, 4);
    
    realData(realNum,3) = mobilityArray2(i, 3);
    realData(realNum,4) = mobilityArray2(i, 4);

    realData(realNum,5) = mobilityArray3(i, 3);
    realData(realNum,6) = mobilityArray3(i, 4);

    realData(realNum,7) = mobilityArray4(i, 3);
    realData(realNum,8) = mobilityArray4(i, 4);

    realData(realNum,9) = mobilityArray5(i, 3);
    realData(realNum,10) = mobilityArray5(i, 4);

    realData(realNum,11) = mobilityArray6(i, 3);
    realData(realNum,12) = mobilityArray6(i, 4);
    
    realData(realNum,13) = mobilityArray7(i, 3);
    realData(realNum,14) = mobilityArray7(i, 4);

    realData(realNum,15) = mobilityArray8(i, 3);
    realData(realNum,16) = mobilityArray8(i, 4);

    realData(realNum,17) = mobilityArray9(i, 3);
    realData(realNum,18) = mobilityArray9(i, 4);

    realData(realNum,19) = mobilityArray10(i, 3);
    realData(realNum,20) = mobilityArray10(i, 4);

    
    % gap of sequence time is 0.01, 100 gap is 1 so time is 1[s]
    for k = 1 : 20

        if i == 1
            predictedData(preNum,k) = 100;
        end

        if(i ~= 1)
            [pos_Data] = EKF(mobilityArray1, mobilityArray2,mobilityArray3,mobilityArray4,mobilityArray5,mobilityArray6,mobilityArray7,mobilityArray8,mobilityArray9,mobilityArray10,predictedData,preNum,realData);
            predictedData(preNum,k) = pos_Data(k,1);
        end
    end
    preNum = preNum + 1;
    realNum = realNum + 1;
   
  
end

%-------------- Kalman Filter Algorithm


%writematrix(combined_matrix,'mobility_ekf_9.csv');
%writematrix

%{
subplot(2,1,1)
plot(predictedData(:,1),predictedData(:,2))
title('predictedData')

subplot(2,1,2)
plot(realData(:,1), realData(:,2))
title('realData')

error = immse(predictedData,realData);
error
%}


function [pos_Data] = EKF(mobilityArray1, mobilityArray2,mobilityArray3,mobilityArray4,mobilityArray5,mobilityArray6,mobilityArray7,mobilityArray8,mobilityArray9,mobilityArray10,predictedData,preNum,realData)

persistent time
persistent firstRun
persistent Q P R
persistent H
persistent cov_v cov_a % covariance velocity & angle of mobility
persistent cov_r cov_a2 % covariance relativeDistance & angle of observation

if isempty(firstRun)

    time = 1;
    firstRun = 1;
    H = diag([1,1,1]);
    P = diag([1,1,0.1]);
    cov_v = 0.456;
    cov_a = 0.378;
    cov_r = 0.213;
    cov_a2 = 0.215;
    
end

pastNum = preNum - 1;

pos_past = predictedData(pastNum, :);
pos_Data = zeros(20,1);
pos_Predict = zeros(20,1); % A*x

pos_Observe = zeros(18, 1); % z(t)
pos_Estimated = zeros(20,1);




%----------- Prediction Step
pos_Predict(1, 1) = pos_past(1,1) + time*mobilityArray1(preNum, 6)*cos(mobilityArray1(pastNum, 5));
pos_Predict(1, 2) = pos_past(1,2) + time*mobilityArray1(preNum, 6)*sin(mobilityArray1(pastNum, 5));
A1 = Ajacob(mobilityArray1, preNum);

pos_Predict(1, 3) = pos_past(1,3) + time*mobilityArray2(preNum, 6)*cos(mobilityArray2(pastNum, 5));
pos_Predict(1, 4) = pos_past(1,4) + time*mobilityArray2(preNum, 6)*sin(mobilityArray2(pastNum, 5));
A2 = Ajacob(mobilityArray2, preNum);

pos_Predict(1, 5) = pos_past(1,5) + time*mobilityArray3(preNum, 6)*cos(mobilityArray3(pastNum, 5));
pos_Predict(1, 6) = pos_past(1,6) + time*mobilityArray3(preNum, 6)*sin(mobilityArray3(pastNum, 5));
A3 = Ajacob(mobilityArray3, preNum);

pos_Predict(1, 7) = pos_past(1,7) + time*mobilityArray4(preNum, 6)*cos(mobilityArray4(pastNum, 5));
pos_Predict(1, 8) = pos_past(1,8) + time*mobilityArray4(preNum, 6)*sin(mobilityArray4(pastNum, 5));
A4 = Ajacob(mobilityArray4, preNum);

pos_Predict(1, 9) = pos_past(1,9) + time*mobilityArray5(preNum, 6)*cos(mobilityArray5(pastNum, 5));
pos_Predict(1, 10) = pos_past(1,10) + time*mobilityArray5(preNum, 6)*sin(mobilityArray5(pastNum, 5));
A5 = Ajacob(mobilityArray5, preNum);

pos_Predict(1, 11) = pos_past(1,11) + time*mobilityArray6(preNum, 6)*cos(mobilityArray6(pastNum, 5));
pos_Predict(1, 12) = pos_past(1,12) + time*mobilityArray6(preNum, 6)*sin(mobilityArray6(pastNum, 5));
A6 = Ajacob(mobilityArray6, preNum);

pos_Predict(1, 13) = pos_past(1,13) + time*mobilityArray7(preNum, 6)*cos(mobilityArray7(pastNum, 5));
pos_Predict(1, 14) = pos_past(1,14) + time*mobilityArray7(preNum, 6)*sin(mobilityArray7(pastNum, 5));
A7 = Ajacob(mobilityArray7, preNum);

pos_Predict(1, 15) = pos_past(1,15) + time*mobilityArray8(preNum, 6)*cos(mobilityArray8(pastNum, 5));
pos_Predict(1, 16) = pos_past(1,16) + time*mobilityArray8(preNum, 6)*sin(mobilityArray8(pastNum, 5));
A8 = Ajacob(mobilityArray8, preNum);

pos_Predict(1, 17) = pos_past(1,17) + time*mobilityArray9(preNum, 6)*cos(mobilityArray9(pastNum, 5));
pos_Predict(1, 18) = pos_past(1,18) + time*mobilityArray9(preNum, 6)*sin(mobilityArray9(pastNum, 5));
A9 = Ajacob(mobilityArray9, preNum);

pos_Predict(1, 19) = pos_past(1,19) + time*mobilityArray10(preNum, 6)*cos(mobilityArray10(pastNum, 5));
pos_Predict(1, 20) = pos_past(1,20) + time*mobilityArray10(preNum, 6)*sin(mobilityArray10(pastNum, 5));
A10 = Ajacob(mobilityArray10, preNum);

%----------- Observation & Correction Step

relativeDistance = zeros(9, 1);
relativeAng = zeros(9,1);
   
% if id_1 observe the others

observeCount = 1;

for i = 1 : 2 : 19
    A_x = realData(preNum, i) + randn*3;
    A_y = realData(preNum, i + 1) + randn*3;

    for k = 1 : 2 : 19
        if i ~= k
            H_x = realData(preNum,i) + randn*3;
            H_y = realData(preNum,i + 1) + randn*3;
            relativeDistance(observeCount, 1) = sqrt((A_x- H_x)^2 + (A_y- H_y)^2);
            relativeAng(observeCount,1) = atan2((A_x - H_y),(A_y - H_x));
            observeCount = observeCount + 1;
        end 
    end
    

    %Calculation 
    observeCount = 1;
    for k = 1 : 2 : 17

        pos_Observe(k,1) = A_x + relativeDis(observeCount,1)*cos(relativeAng(observeCount,1));
        pos_Observe(k+1,1) = A_y + relativeDis(observeCount,1)*sin(relativeAng(observeCount,1));
        observeCount = observeCount + 1;
    end
    

    
    pos_Observe = zeros(18, 1); % Initalize
end



end

function A = Ajacob(mobilityArray, number)

    A = zeros(3,3);
    theta = mobilityArray(number-1, 5);
    speed = mobilityArray(number, 6);
    distance = 0.01 * speed;

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
    A(3,3) = 1;

end