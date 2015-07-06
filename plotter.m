raw_pairs = csvread('all_pairs.csv')';
%scatter(raw_pairs(1,:),raw_pairs(2,:),raw_pairs(3,:));
plot(raw_pairs(1,:),raw_pairs(2,:),'ko','MarkerSize',0.5);
figure;
raw_pairs = csvread('centers.csv')';
plot(raw_pairs(1,:),raw_pairs(2,:),'ko','MarkerSize',0.5);
figure;
plot(raw_pairs(1,:),raw_pairs(3,:),'ko','MarkerSize',0.5);
pause;