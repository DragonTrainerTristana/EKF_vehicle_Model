predictedData = zeros(count, 2);
x = zeros(3,1);
x(1,1) = Xsaved(1,3);
x(2,1) = Xsaved(1,4);
x(3,1) = Xsaved(1,5);
predictedData(1,1) = x(1,1);
predictedData(1,2) = x(2,1);

for i = 1: count -1
    predictedData(i + 1,1) = predictedData(i,1) + 0.01*Xsaved(i,6)*cos(Xsaved(i,5));
    predictedData(i + 1,2) = predictedData(i,2) + 0.01*Xsaved(i,6)*sin(Xsaved(i,5));
end

subplot(2,1,1)
%plot(predictedData(:,1),predictedData(:,2))
plot(predictedData)

subplot(2,1,2)
%plot(realData(:,1), realData(:,2))
plot(realData)

mse = immse(predictedData, realData)