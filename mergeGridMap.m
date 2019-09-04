function merged_map = mergeGridMap(map1,map2)
%MERGEGRIDMAP 此处显示有关此函数的摘要
%   此处显示详细说明

maxRC = max(size(map2),size(map1));
gmap1 = uint8(255*ones(maxRC));
gmap2 = uint8(255*ones(maxRC));
gmap1(1:size(map1,1),1:size(map1,2))=map1;
gmap2(1:size(map2,1),1:size(map2,2))=map2;
merged_map = min(gmap1,gmap2);

