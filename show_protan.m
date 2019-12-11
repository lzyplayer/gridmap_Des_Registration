function show_protan(map1,map2,pointSet1,pointSet2,s)
%SHOW_PROTAN show point match like SIFT_match_show
%   pointSet1 = n x 2
%   pointSet2 = n x 2

mapPair = joinImage(map1,map2);
figure('Position', [100 100 size(mapPair,2) size(mapPair,1)]);
imshow(mapPair);
axis on
hold on
%²î¾àmap2
xdistance = size(map1,2);
N=size(pointSet1,1);
for i=1:N
    line([s*pointSet1(i,1),xdistance+s*pointSet2(i,1)],[s*pointSet1(i,2),s*pointSet2(i,2)],'Color','r')
end

end

