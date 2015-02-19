%work in progress; numerical comparison of ellipse parameters
clear
clc
close all

fileID1 = fopen('../nister_ellipses.txt');
fileID2 = fopen('../vlfeat_ellipses.txt');
data1 = textscan(fileID1,'%f32');
data2 = textscan(fileID2,'%f32');
N1 = size(data1{1,1},1);
N2 = size(data2{1,1},1);
data1_mat = zeros(max([N1,N2])/5,5);
data2_mat = zeros(max([N1,N2])/5,5);
r = 1;
for i=1:5:N1
    data1_mat(r,:) = data1{1,1}(i:i+4);
    r = r + 1;
end
r = 1;
for i=1:5:N2
    data2_mat(r,:) = data2{1,1}(i:i+4);
    r = r + 1;
end

data1_mat = sortrows(data1_mat,1);
data2_mat = sortrows(data2_mat,1);
diff = abs(data1_mat - data2_mat);
max = max(data1_mat,data2_mat);
norm_diff = abs(diff ./ max);
for i = 1:length(norm_diff)*5
    if norm_diff(i) > 1
        norm_diff(i) = 1;
    end
end
similarity = 1 - mean(norm_diff(:));