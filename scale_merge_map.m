function merge_im = scale_merge_map(map1,map2,warp_matrix)
%UNTITLED 此处显示有关此函数的摘要
%  warp_matrix -> map1 = map2

%size_get_warp
map1_size_template = imwarp(map1,affine2d(warp_matrix'));
%
outputView = imref2d(max(size(map2),size(map1_size_template))); % a bit larger
map1_warped = imwarp(map1,affine2d(warp_matrix'),'OutputView',outputView,'FillValues',255);
%get size
map1_size = size(map1_warped);
map2_size = size(map2);
max_size=max(map1_size,map2_size);
%ready im
box1 = uint8(ones(max_size)*255);
box2 = box1;
box1(1:map1_size(1),1:map1_size(2))=map1_warped;
box2(1:map2_size(1),1:map2_size(2))=map2;
merge_im = min(box1,box2);

imshow( merge_im);

end

