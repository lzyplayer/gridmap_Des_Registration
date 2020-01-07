function show_protangonation_claw_y(map1,map2,pointSet1,pointSet2,s,pointOriSet1,pointOriSet2)
%SHOW_PROTAN show point match like SIFT_match_show
%   pointSet1 = n x 2
%   pointSet2 = n x 2
map1_ydown=0;
size_1 = size(map1);
size_1(1)=size_1(1)+map1_ydown;
map1_box =uint8(ones(size_1)*255);
map1_box(map1_ydown+1:end,:) = map1;


mapPair = joinImage_y(map1_box,map2);
figure('Position', [100 100 size(mapPair,2) size(mapPair,1)]);
imshow(mapPair);
% axis on
hold on
%²î¾àmap2
ydistance =  size(map1_box,1);
xdistance = size(map1_box,1);
if nargin==7
    N=size(pointOriSet1,1);
    for i=1:N
        line([s*pointOriSet1(i,1),xdistance+s*pointOriSet2(i,1)],[s*pointOriSet1(i,2)+map1_ydown,s*pointOriSet2(i,2)],'Color',[0.2010 0.6450 0.9330],'LineWidth',0.5);
        %         line([s*pointOriSet1(i,1),xdistance+s*pointOriSet2(i,1)],[s*pointOriSet1(i,2)+map1_ydown,s*pointOriSet2(i,2)],'Color','r','LineWidth',0.8);
    end
end
N=size(pointSet1,1);
%% 
main_point1 = [s*pointSet1(1,1), s*pointSet1(1,2)];
main_point2 = [s*pointSet2(1,1),ydistance+s*pointSet2(1,2)];
% show  claw
for i=2:N
    line([main_point1(1),s*pointSet1(i,1)],[main_point1(2),s*pointSet1(i,2)],'Color','b','LineWidth',2.6);
    line([main_point2(1),s*pointSet2(i,1)],[main_point2(2),ydistance+s*pointSet2(i,2)],'Color','b','LineWidth',2.6);
%     line([s*pointSet1(i,1),xdistance+s*pointSet2(i,1)],[s*pointSet1(i,2)+map1_ydown,s*pointSet2(i,2)],'Color','r','LineWidth',0.8)%,'LineStyle',':','LineStyle','--'
end
line([main_point1(1),main_point2(1)],[main_point1(2),main_point2(2)],'Color','r','LineWidth',3.0);

%show spot
showPoint(s*pointSet1,false,20);
showPoint([s*pointSet2(:,1),ydistance+s*pointSet2(:,2)],false,20);
showPoint([main_point1',main_point2']);




end

