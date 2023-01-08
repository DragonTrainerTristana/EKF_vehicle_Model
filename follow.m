clc
clear
load Xsaved.mat

% Array count Data
i_count = 1;

count = 59900; % 59900 rows per id
mobilityNum = 13; % id num 13
observationNum = 14; % id num 14

mobilityArray = zeros(count, 6);
observationArray = zeros(count, 6);
realData = zeros(count, 2); % mobility ID real Data
predictedData = zeros(count ,2); % predicted by EKF


for i = 1 : size(data, 1)
    if data(i, 2) == mobilityNum
        mobilityArray(i_count, :) = data(i,:);
        i_count = i_count + 1;
    end
end

i_count = 1; % initialize again

for i = 1 : size(data, 1)
    if data(i, 2) == observationNum
        observationArray(i_count, :) = data(i,:);
        i_count = i_count + 1;
    end
end

%-------------- Kalman Filter Algorithm

for i = 1 : count
    
    if i == 1 % Initialize EKF HyperParameter
    
        % Initialize first point of mobility model
        predictedData(i, 1) = mobilityArray(i, 3);
        predictedData(i, 2) = mobilityArray(i, 4);

        realData(i, 1) = mobilityArray(i,3);
        realData(i, 2) = mobilityArray(i,4);
    end

    if i ~= 1 % iterate num of count
        [pos_x, pos_y, distance] = SystemEKF(mobilityArray,observationArray,predictedData, i);
        predictedData(i, 1) = pos_x;
        predictedData(i, 2) = pos_y;
        realData(i, 1) = mobilityArray(i,3);
        realData(i, 2) = mobilityArray(i,4);
    end

end

subplot(2,1,1)
plot(predictedData(:,1),predictedData(:,2))
title('predictedData')

subplot(2,1,2)
plot(realData(:,1), realData(:,2))
title('realData')

function [pos_x, pos_y, distance] = SystemEKF(mobilityArray,observationArray,predictedData,num)

persistent Q R P % covariance matrix
persistent firstRun
persistent time;
persistent x z newX;
persistent cov_v cov_a % covariance velocity & angle of mobility
persistent cov_r cov_a2 % covariance relativeDistance & angle of observation


    if isempty(firstRun)
        
        cov_v = 0.456;
        cov_a = 0.378;
        cov_r = 0.213;
        cov_a2 = 0.215;

        P = diag([1,1,0.1]);
        Q = zeros(3,3);

        R = zeros(2,2);

        x = zeros(2,1);
        newX = zeros(2,1); 
        z = zeros(3,1);
        x(1,1) = mobilityArray(1, 3);
        x(2,1) = mobilityArray(1, 4);
        x(3,1) = 0;

        pos_x = x(1);
        pos_y = x(2);
        distance = 0;

        time = 0.01;
        firstRun = 1;
    end

    if firstRun == 1
    
        arbiNum = num - 1;

        %-- Mobility Model Prediction(Estimation) Step
        x(1, 1) = predictedData(arbiNum, 1) + time*mobilityArray(num, 6)*cos(mobilityArray(arbiNum, 5) * pi/180);
        x(2, 1) = predictedData(arbiNum, 2) + time*mobilityArray(num, 6)*sin(mobilityArray(arbiNum, 5) * pi/180);
        x(3, 1) = 0;
        xH = zeros(3,1);
        xH(1,1) = predictedData(arbiNum, 1);
        xH(2,1) = predictedData(arbiNum, 2);
        xH(3,1) = 0;

        a = cos(mobilityArray(arbiNum, 5) * pi/180);
        b = sin(mobilityArray(arbiNum, 5) * pi/180);

        A = Ajacob(mobilityArray, num);
        
        Q = [time^2*a^2*cov_v^2 time^2*a*b*cov_v^2 0; time^2*a*b*cov_v^2 (time*b*cov_a)^2 0; 0 0 cov_a^2];

        Pp = A*P*A' + Q;

        %-- Observaton Model Correction Step
        H_x = observationArray(num,3); % position x
        H_y = observationArray(num,4); % position y
        relativeDis = sqrt((x(1,1) - H_x)^2 + (x(2,1) - H_y)^2);
        relativeAng = atan2((H_y-x(2,1)),(H_x-x(1,1)));
        relativeAng = relativeAng * 180/pi; 

        H = Hjacob(H_x,H_y,mobilityArray, num,x, relativeDis);

        z(1 ,1) = (cov_r)^2*(cos(relativeAng))^2 + relativeDis^2*cov_a2^2*(sin(relativeAng))^2;
        z(2 ,1) = (cov_r)^2*(sin(relativeAng))^2 + relativeDis^2*cov_a2^2*(cos(relativeAng))^2;
        %z(3, 1) = 0;
        
        R = [cov_r^2 0; 0 cov_a2^2];
        K = Pp*H'*inv(H*Pp*H' + R);
        xp = x;
        HH = H*xH;
        HHH = [HH(1,1);H(2,1);0];
        xp(3,:) = [];
        
        newX = xp + K'*(z - HHH); % Correction X
        P = Pp-K*H*Pp;

        pos_x = newX(1);
        pos_y = newX(2);  
        distance = relativeDis;
        

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

