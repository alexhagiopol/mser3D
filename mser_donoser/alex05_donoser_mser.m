%Algorithm 8 of Donoser07 thesis
%Build the component tree struture.
clear
clc
close all

global Q_tree_par_set;
global Q_tree_rnk_set;
global Q_node_par_set;
global Q_node_rnk_set;
global nodes_made;
nodes_made = 0;
% format = '../Sequence1/frame%02d.jpg'; N = 48; % framing
format = '../Sequence2/frame%02d.jpg'; N = 50; % indoor
% format = '../Sequence3/frame%02d.jpg'; N = 47; % hulk
% format = '../Sequence4/frame%03d.jpg'; N = 101; % trees
% format = '../Sequence5/frame%03d.jpg'; N = 193; % garage
f = 1;
filename = sprintf(format,f);
img_color = imread(filename);
img = rgb2gray(img_color);  
img = imresize(img,0.25,'bicubic');

%Algorithm 8
CT = c_tree;
lowest_node = [];

%this is insanely slow....
for p = 1:(size(img,1)*size(img,2)) %p is a pixel id
    mser_make_set(Q_tree_par_set, Q_tree_rnk_set, p);
    mser_make_set(Q_node_par_set, Q_node_rnk_set, p);
    CT.make_node(p,img(p));
    lowest_node(p) = p;
    if mod(p,5000) == 0
        disp([num2str(p),' done out of ',num2str(size(img,1)*size(img,2))]);
        disp([num2str(nodes_made), ' nodes made']);
    end
end
%sort image pixels in descending order based on value
[rows,cols] = size(img);
unsorted_img_vals = [img(1:end)]';
unsorted_img_indices = [1:rows*cols]';
unsorted_img = [unsorted_img_indices,cast(unsorted_img_vals,'like',unsorted_img_indices)];
sorted_img = sortrows(unsorted_img,-2);


