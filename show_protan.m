function show_protan(map1,map2,pointSet1,pointSet2,s,pointOriSet1,pointOriSet2)
%SHOW_PROTAN show point match like SIFT_match_show
%   pointSet1 = n x 2
%   pointSet2 = n x 2
map1_ydown=0;
size_1 = size(map1);
size_1(1)=size_1(1)+map1_ydown;
map1_box =uint8(ones(size_1)*255);
map1_box(map1_ydown+1:end,:) = map1;


mapPair = joinImage(map1_box,map2);
figure('Position', [100 100 size(mapPair,2) size(mapPair,1)]);
imshow(mapPair);
% axis on
hold on
%²î¾àmap2
xdistance = size(map1_box,2);
if nargin==7
    N=size(pointOriSet1,1);
    for i=1:N
        line([s*pointOriSet1(i,1),xdistance+s*pointOriSet2(i,1)],[s*pointOriSet1(i,2)+map1_ydown,s*pointOriSet2(i,2)],'Color',[0.2010 0.6450 0.9330],'LineWidth',0.5);
        %         line([s*pointOriSet1(i,1),xdistance+s*pointOriSet2(i,1)],[s*pointOriSet1(i,2)+map1_ydown,s*pointOriSet2(i,2)],'Color','r','LineWidth',0.8);
    end
end
N=size(pointSet1,1);
for i=1:N
    line([s*pointSet1(i,1),xdistance+s*pointSet2(i,1)],[s*pointSet1(i,2)+map1_ydown,s*pointSet2(i,2)],'Color','r','LineWidth',0.8)%,'LineStyle',':','LineStyle','--'
end

% mapPair = joinImage(map1,map2);
% figure('Position', [100 100 size(mapPair,2) size(mapPair,1)]);
% imshow(mapPair);
% % axis on
% hold on
% %²î¾àmap2
% xdistance = size(map1,2);
% if nargin==7
%     N=size(pointOriSet1,1);
%     for i=1:N
%         line([s*pointOriSet1(i,1),xdistance+s*pointOriSet2(i,1)],[s*pointOriSet1(i,2),s*pointOriSet2(i,2)],'Color','r','LineStyle','--');
%     end
% end
% N=size(pointSet1,1);
% for i=1:N
%     line([s*pointSet1(i,1),xdistance+s*pointSet2(i,1)],[s*pointSet1(i,2),s*pointSet2(i,2)],'Color','r')%,'LineStyle',':'
% end

end

