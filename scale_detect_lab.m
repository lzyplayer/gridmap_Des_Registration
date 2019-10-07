figure;
grid_size=0.01;

Mdata = pointCMap1;
% Ddata = seleCorM2.Location/100;
Ddata = cursor_info1.Position;
gen_result = generate_analyse_data(Mdata,Ddata,grid_size);
ring_point_num = diff(gen_result);
ring_area = grid_size^2+(1:length(ring_point_num))*grid_size*2;
ring_num_noscale = ring_point_num./ring_area;%(1+0.5*(1:length(ring_point_num)));
subplot(2,1,1);
plot(ring_num_noscale);
axis([1 60 0 inf])
axis auto

r=grid_size*22;
Ddata = cursor_info1.Position;
pos = [Ddata-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1,1]);
axis equal

subplot(2,1,2);
Mdata = pointCMap2;
% Ddata = seleCorM2.Location/100;
Ddata = cursor_info2.Position;
gen_result = generate_analyse_data(Mdata,Ddata,grid_size);
ring_point_num = diff(gen_result);
ring_area = grid_size^2+(1:length(ring_point_num))*grid_size*2;
ring_num_noscale = ring_point_num./ring_area;
plot(ring_num_noscale);


r=grid_size*9;
Ddata = cursor_info2.Position;
pos = [Ddata-r 2*r 2*r];
rectangle('Position',pos,'Curvature',[1,1]);
axis equal
%% peak
index=(1:length(ring_point_num));%*grid_size;
[pks,locs,w,p] =findpeaks(ring_num_noscale,index,'MinPeakDistance',2,'NPeaks',10,'MinPeakProminence',7,'SortStr','descend');
text(locs+.02,pks,num2str((1:numel(pks))'))