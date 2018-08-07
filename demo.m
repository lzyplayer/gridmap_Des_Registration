%% harris_getted
map1=rgb2gray(imread('Fr1_5.png','png'));
map2=rgb2gray(imread('Fr2_5.png','png'));
% imshowpair(map1,map2);
imshow(map1);hold on;%figure;
cornersM1=detectHarrisFeatures(map1);plot(cornersM1.selectStrongest(200));
figure;
imshow(map2);hold on
cornersM2=detectHarrisFeatures(map2);plot(cornersM2.selectStrongest(200));

% imshowpair(map1,map2);