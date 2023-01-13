load X_Ntest10_pp1.0.mat
load Xh_Ntest10_pp1.0.mat

size(x(:,1))

data1 = x(:,1:2);

data2 = x(:,3:4);

data3 = x(:,5:6);

data4 = x(:,7:8);

data5 = x(:,9:10);

scatter(data1(:,1),data1(:,2))