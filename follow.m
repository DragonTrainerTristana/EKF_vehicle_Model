load Xsaved.mat

i_count = 1;
Xsaved = zeros(count, 6);
csv_number = 13;
for i = 1 : size(data, 1)
    if data(i, 2) == csv_number
        Xsaved(i_count, :) = data(i,:);
        i_count = i_count + 1;
    end
end


T = 0.01; % Sampling period (s),
K = 59900; % Data Length
r_max = 1; % Threshold

%Initial matrices
state_hat = zeros(K, 3);
x_hat = zeros(K,1);
y_hat = zeros(K,1);
theta_hat = zeros(K,1);
P_hat = zeros(3,3,K);

%Initial values
state_hat(1,:) = [Xsaved(1,3),Xsaved(1,4),Xsaved(1,5)].';
x_hat(1) = state_hat(1,1);
y_hat(1) = state_hat(1,2);
theta_hat(1) = state_hat(1,3);
P_hat(:,:,1) = diag([1, 1, 0.1]);
v_var = 0.044;

for k = 2:1:K % Initialize했으니까 2부터 시작해야함
    
    Q = [v_var*(T*cos(theta_hat(k - 1)))^2, v_var*(T^2*cos(theta_hat(k - 1))*sin(theta_hat(k - 1))), 0;
        v_var*(T^2*cos(theta_hat(k - 1))*sin(theta_hat(k - 1))), v_var*(T*sin(theta_hat(k - 1)))^2, 0;
        0, 0, 1];
      
    % Jacobian motion model
    F = [1 0 -T*Xsaved(k,6)*sin(theta_hat(k - 1));
        0 1 T*Xsaved(k,6)*cos(theta_hat(k - 1));
        0 0 1];
    
    % Predictor
    P_check = F*P_hat(:,:,k - 1)*F.' + Q;
    x_check = x_hat(k - 1) + T*Xsaved(k,6)*cos(theta_hat(k - 1));
    y_check = y_hat(k - 1) + T*Xsaved(k,6)*sin(theta_hat(k - 1));
    theta_check = Xsaved(k, 5);
    
    state_check = [x_check, y_check, theta_check].';

    % Observation

    cnt = 0;
    G = zeros(2,3);
    innov = zeros(2,1);
    R = zeros(2,2);

    for i = 1:1:17
    
        
    
    end

end



