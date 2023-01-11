clc
clear
load Xsaved.mat


count = 59900; % 59900 rows per id
mobilityNum = 35; % id num 13
observationNum = 14; % id num 14

ranNum = zeros(1,12);

% Generate 10 Random Numbers from 1 to 504
for i = 1 : 12
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
mobNum = zeros(1, 12);
for i = 1 : 12
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
mobilityArray11 = zeros(count, 6);
mobilityArray12 = zeros(count, 6);

%-----------
dividedCount = 599;
predictedData = zeros(dividedCount ,24);
realData = zeros(dividedCount ,24);
mobCount = ones(1,14);

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
    if data(i, 2) == mobNum(1,7)
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
    if data(i, 2) == mobNum(1,11)
        mobilityArray11(mobCount(1,11), :) = data(i,:);
        mobCount(1,11) = mobCount(1,11) + 1;
    end    
    if data(i, 2) == mobNum(1,12)
        mobilityArray12(mobCount(1,12), :) = data(i,:);
        mobCount(1,12) = mobCount(1,12) + 1;

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

    realData(realNum,21) = mobilityArray11(i, 3);
    realData(realNum,22) = mobilityArray11(i, 4);

    realData(realNum,23) = mobilityArray12(i, 3);
    realData(realNum,24) = mobilityArray12(i, 4);

    
    % gap of sequence time is 0.01, 100 gap is 1 so time is 1[s]
    if i == 1
        predictedData(preNum, :) = 100; % initialize all position 100 (x,y)
    end

    if i ~= 1
        [pos_Data] = EKF(mobilityArray1, mobilityArray2,mobilityArray3,mobilityArray4,mobilityArray5,mobilityArray6,mobilityArray7,mobilityArray8,mobilityArray9,mobilityArray10,mobilityArray11,mobilityArray12,predictedData,preNum,realData);
        predictedData(preNum, :) = pos_Data;
   
    end
     
    preNum = preNum + 1;
    realNum = realNum + 1;
   
  
end

%{
subplot(2,1,1)
scatter(predictedData(:,5),predictedData(:,6))
title('predictedData')

subplot(2,1,2)
scatter(realData(:,5), realData(:,6))
title('realData')
%}
error1 = immse(predictedData(:,2),realData(:,2))
error3 = immse(predictedData(:,4),realData(:,4))
error5 = immse(predictedData(:,6),realData(:,6))

final_predict = array2table(predictedData(:,1:16),"VariableNames",["id1_x","id1_y","id2_x","id2_y","id3_x","id3_y","id4_x","id4_y","id5_x","id5_y","id6_x","id6_y","id7_x","id7_y","id8_x","id8_y"]);
final_real = array2table(realData(:,1:16),"VariableNames",["id1_x","id1_y","id2_x","id2_y","id3_x","id3_y","id4_x","id4_y","id5_x","id5_y","id6_x","id6_y","id7_x","id7_y","id8_x","id8_y"]);

writetable(final_predict,'predictedData16.csv')
writetable(final_real,'realData16.csv')

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


function [pos_Data] = EKF(mobilityArray1, mobilityArray2,mobilityArray3,mobilityArray4,mobilityArray5,mobilityArray6,mobilityArray7,mobilityArray8,mobilityArray9,mobilityArray10,mobilityArray11,mobilityArray12,predictedData,preNum,realData)

persistent time
persistent firstRun
persistent Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 Q9 Q10 Q11 Q12  R
persistent P1 P2 P3 P4 P5 P6 P7 P8 P9 P10 P11 P12 
persistent H
persistent cov_v cov_a % covariance velocity & angle of mobility
persistent cov_r cov_a2 % covariance relativeDistance & angle of observation

if isempty(firstRun)

    time = 1;
    firstRun = 1;
    H = diag([1,1,1]);

    P1 = diag([1,1,0.1]);
    P2 = diag([1,1,0.1]);
    P3 = diag([1,1,0.1]);
    P4 = diag([1,1,0.1]);
    P5 = diag([1,1,0.1]);
    P6 = diag([1,1,0.1]);
    P7 = diag([1,1,0.1]);
    P8 = diag([1,1,0.1]);
    P9 = diag([1,1,0.1]);
    P10 = diag([1,1,0.1]);
    P11 = diag([1,1,0.1]);
    P12 = diag([1,1,0.1]);
    

    cov_v = 0.456;
    cov_a = 0.378;
    cov_r = 0.213;
    cov_a2 = 0.215;
    
end

pastNum = preNum - 1;

pos_past = predictedData(pastNum, :);

pos_Predict = zeros(24,1); % A*x

pos_Observe = zeros(22, 1); % z(t)
pos_Estimated = zeros(24,1);




%----------- Prediction Step <- Initialize Covariance matrix Pp and Q(for Kalman Gain)

%id 1

pos_Predict(1, 1) = pos_past(1,1) + time*mobilityArray1((preNum-1)*100+1, 6)*cos(mobilityArray1((preNum-1)*100, 5));
pos_Predict(2, 1) = pos_past(1,2) + time*mobilityArray1((preNum-1)*100+1, 6)*sin(mobilityArray1((preNum-1)*100, 5));
A1 = Ajacob(mobilityArray1, preNum);
a = cos(mobilityArray1(pastNum,5));
b = sin(mobilityArray1(pastNum,5));
Q1 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp1 = A1*P1*A1' + Q1;

%id 2
pos_Predict(3, 1) = pos_past(1,3) + time*mobilityArray2(preNum, 6)*cos(mobilityArray2(pastNum, 5));
pos_Predict(4, 1) = pos_past(1,4) + time*mobilityArray2(preNum, 6)*sin(mobilityArray2(pastNum, 5));
A2 = Ajacob(mobilityArray2, preNum);
a = cos(mobilityArray2(pastNum,5));
b = sin(mobilityArray2(pastNum,5));
Q2 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp2 = A2*P2*A2' + Q2;

%id 3
pos_Predict(5, 1) = pos_past(1,5) + time*mobilityArray3(preNum, 6)*cos(mobilityArray3(pastNum, 5));
pos_Predict(6, 1) = pos_past(1,6) + time*mobilityArray3(preNum, 6)*sin(mobilityArray3(pastNum, 5));
A3 = Ajacob(mobilityArray3, preNum);
a = cos(mobilityArray3(pastNum,5));
b = sin(mobilityArray3(pastNum,5));
Q3 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp3 = A3*P3*A3' + Q3;

%id 4
pos_Predict(7, 1) = pos_past(1,7) + time*mobilityArray4(preNum, 6)*cos(mobilityArray4(pastNum, 5));
pos_Predict(8, 1) = pos_past(1,8) + time*mobilityArray4(preNum, 6)*sin(mobilityArray4(pastNum, 5));
A4 = Ajacob(mobilityArray4, preNum);
a = cos(mobilityArray4(pastNum,5));
b = sin(mobilityArray4(pastNum,5));
Q4 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp4 = A4*P4*A4' + Q4;

%id 5
pos_Predict(9, 1) = pos_past(1,9) + time*mobilityArray5(preNum, 6)*cos(mobilityArray5(pastNum, 5));
pos_Predict(10, 1) = pos_past(1,10) + time*mobilityArray5(preNum, 6)*sin(mobilityArray5(pastNum, 5));
A5 = Ajacob(mobilityArray5, preNum);
a = cos(mobilityArray5(pastNum,5));
b = sin(mobilityArray5(pastNum,5));
Q5 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp5 = A5*P5*A5' + Q5;

%id 6
pos_Predict(11, 1) = pos_past(1,11) + time*mobilityArray6(preNum, 6)*cos(mobilityArray6(pastNum, 5));
pos_Predict(12, 1) = pos_past(1,12) + time*mobilityArray6(preNum, 6)*sin(mobilityArray6(pastNum, 5));
A6 = Ajacob(mobilityArray6, preNum);
a = cos(mobilityArray6(pastNum,5));
b = sin(mobilityArray6(pastNum,5));
Q6 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp6 = A6*P6*A6' + Q6;

%id 7
pos_Predict(13, 1) = pos_past(1,13) + time*mobilityArray7(preNum, 6)*cos(mobilityArray7(pastNum, 5));
pos_Predict(14, 1) = pos_past(1,14) + time*mobilityArray7(preNum, 6)*sin(mobilityArray7(pastNum, 5));
A7 = Ajacob(mobilityArray7, preNum);
a = cos(mobilityArray7(pastNum,5));
b = sin(mobilityArray7(pastNum,5));
Q7 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp7 = A7*P7*A7' + Q7;

%id 8
pos_Predict(15, 1) = pos_past(1,15) + time*mobilityArray8(preNum, 6)*cos(mobilityArray8(pastNum, 5));
pos_Predict(16, 1) = pos_past(1,16) + time*mobilityArray8(preNum, 6)*sin(mobilityArray8(pastNum, 5));
A8 = Ajacob(mobilityArray8, preNum);
a = cos(mobilityArray8(pastNum,5));
b = sin(mobilityArray8(pastNum,5));
Q8 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp8 = A8*P8*A8' + Q8;

%id 9
pos_Predict(17, 1) = pos_past(1,17) + time*mobilityArray9(preNum, 6)*cos(mobilityArray9(pastNum, 5));
pos_Predict(18, 1) = pos_past(1,18) + time*mobilityArray9(preNum, 6)*sin(mobilityArray9(pastNum, 5));
A9 = Ajacob(mobilityArray9, preNum);
a = cos(mobilityArray9(pastNum,5));
b = sin(mobilityArray9(pastNum,5));
Q9 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp9 = A9*P9*A9' + Q9;

%id 10
pos_Predict(19, 1) = pos_past(1,19) + time*mobilityArray10(preNum, 6)*cos(mobilityArray10(pastNum, 5));
pos_Predict(20, 1) = pos_past(1,20) + time*mobilityArray10(preNum, 6)*sin(mobilityArray10(pastNum, 5));
A10 = Ajacob(mobilityArray10, preNum);
a = cos(mobilityArray10(pastNum,5));
b = sin(mobilityArray10(pastNum,5));
Q10 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp10 = A10*P10*A10' + Q10;

%id 11
pos_Predict(21, 1) = pos_past(1,21) + time*mobilityArray11(preNum, 6)*cos(mobilityArray11(pastNum, 5));
pos_Predict(22, 1) = pos_past(1,22) + time*mobilityArray11(preNum, 6)*sin(mobilityArray11(pastNum, 5));
A11 = Ajacob(mobilityArray11, preNum);
a = cos(mobilityArray11(pastNum,5));
b = sin(mobilityArray11(pastNum,5));
Q11 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp11 = A11*P11*A11' + Q11;

%id 12
pos_Predict(23, 1) = pos_past(1,23) + time*mobilityArray12(preNum, 6)*cos(mobilityArray12(pastNum, 5));
pos_Predict(24, 1) = pos_past(1,24) + time*mobilityArray12(preNum, 6)*sin(mobilityArray12(pastNum, 5));
A12 = Ajacob(mobilityArray12, preNum);
a = cos(mobilityArray12(pastNum,5));
b = sin(mobilityArray12(pastNum,5));
Q12 = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 0];
Pp12 = A12*P12*A12' + Q12;

%----------- Observation & Correction Step

relativeDistance = zeros(9, 1);
relativeAng = zeros(9,1);
   
% if id_1 observe the others

observeCount = 1;

%Internal Observation 
n_ii = 30*randn;
pos_Predict(:,1) = pos_Predict(:,1) + n_ii; %(x,y)
for i = 1 : 2 : 15

    A_x = realData(preNum, i);
    A_y = realData(preNum, i + 1);
    for k = 1 : 2 : 15
        if i ~= k
            H_x = realData(preNum,i);
            H_y = realData(preNum,i + 1);
            %External Observation
            relativeDistance(observeCount, 1) = sqrt((A_x- H_x)^2 + (A_y- H_y)^2) + randn*25; % relDis Noise n_d
            relativeAng(observeCount,1) = atan2((A_y - H_y),(A_x - H_x)) + randn*1*pi/180; % relAng Noise n_phi
            observeCount = observeCount + 1;
        end 
    end
    
    arbi_x = 0;
    arbi_y = 0;

    %Calculation 
    observeCount = 1;
    for k = 1 : 2 : 14

        pos_Observe(k,1) = A_x + relativeDistance(observeCount,1)*cos(relativeAng(observeCount,1));
        arbi_x = arbi_x + pos_Observe(k,1);
        pos_Observe(k+1,1) = A_y + relativeDistance(observeCount,1)*sin(relativeAng(observeCount,1));
        arbi_y = arbi_y + pos_Observe(k+1,1);
        observeCount = observeCount + 1;
    end
    z = zeros(3,1);
    newX = zeros(3,1);
    arbi_x = arbi_x /7;
    arbi_y = arbi_y /7;

    z(1,1) = arbi_x;
    z(2,1) = arbi_y;
    z(3,1) = 0;

    R = 20;
    %id = 1
    if i == 1
        K1 = Pp1*H*inv(H*Pp1*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(1,1);
        xp(2,1) = pos_Predict(i+1,1);    
        xp(3,1) = 0;
        %xp(1,1) = realData(preNum, i);
        %xp(2,1) = realData(preNum, i+1);
        
     

        %Et = z;   
        Et = xp + K1*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end

    %id = 2
    if i == 3
        K2 = Pp2*H*inv(H*Pp2*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        
        Et = xp + K2*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end

    %id = 3
    if i == 5
        K3 = Pp3*H*inv(H*Pp3*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K3*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end

    %id = 4
    if i == 7
        K4 = Pp4*H*inv(H*Pp4*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K4*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);    
    end

    %id = 5
    if i == 9
        K5 = Pp5*H*inv(H*Pp5*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K5*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);        
    end
    
    %id = 6
    if i == 11
        K6 = Pp6*H*inv(H*Pp6*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K6*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);    
    end
    
    %id = 7
    if i == 13
        K7 = Pp7*H*inv(H*Pp7*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K7*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end

    %id = 8
    if i == 15
        K8 = Pp8*H*inv(H*Pp8*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K8*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);    
    end

    %id = 9
    if i == 17
        K9 = Pp9*H*inv(H*Pp9*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K9*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);    
    end

    %if i = 10
    if i == 19
        K10 = Pp10*H*inv(H*Pp10*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K10*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end

    %id 11
    if i == 21
        K11 = Pp11*H*inv(H*Pp11*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K11*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end    

    %id 12
    if i == 23
        K12 = Pp12*H*inv(H*Pp12*H' + R);
        xp = zeros(3,1);
        xp(1,1) = pos_Predict(i,1);
        xp(2,1) = pos_Predict(i+1,1);
        xp(3,1) = 0;
   
        Et = xp + K11*(z- H*xp);
        pos_Estimated(i,1) = Et(1);
        pos_Estimated(i+1,1) = Et(2);
    end       
    pos_Observe = zeros(22, 1); % Initalize
end

pos_Data = pos_Estimated';

end

function A = Ajacob(mobilityArray, number)

    A = zeros(3,3);
    theta = mobilityArray(number-1, 5);
    speed = mobilityArray(number, 6);
    distance = speed;

    A(1,1) = 1;
    A(1,2) = 0;
    %A(1,3) = cos(theta);
    A(1,3) = -distance*sin(theta);

    A(2,1) = 0;
    A(2,2) = 1;
    %A(2,3) = sin(theta);
    A(2,3) = distance*cos(theta);
    A(3,1) = 0;
    A(3,2) = 0;
    A(3,3) = 1;

end

function H = Hjacob(H_x,H_y,Xsaved, number,x, distance)

    theta_check = Xsaved(number, 5);

    H1_dx = -(H_x - x(1) - distance*cos(theta_check))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2)^(1/2);
    H1_dy = -(H_y - x(2) - distance*sin(theta_check))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2)^(1/2);
    dg1_dtheta = distance*(sin(theta_check)*(H_x - x(1) - distance*cos(theta_check)) - cos(theta_check)*(H_y - x(2) - distance*sin(theta_check)))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2)^(1/2);
            
    H2_dx = -(H_y - x(2) - distance*sin(theta_check))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2)^(1/2);
    H2_dy = -(H_x - x(1) - distance*cos(theta_check))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2)^(1/2);
    dg2_dtheta = -distance*sin(theta_check)*(H_y - x(2) - distance*sin(theta_check))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2) - distance*cos(theta_check)*(H_x - x(1) - distance*cos(theta_check))/((H_x - x(1) - distance*cos(theta_check))^2 + (H_y - x(2) - distance*sin(theta_check))^2) - 1;
            
    H = [H1_dx H1_dy dg1_dtheta ; H2_dx H2_dy dg2_dtheta];

end