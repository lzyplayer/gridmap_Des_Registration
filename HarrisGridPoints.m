classdef HarrisGridPoints
    %UNTITLED2 �˴���ʾ�йش����ժҪ
    %   �˴���ʾ��ϸ˵��
    
    properties (SetAccess = public)
        Location;
        ScaleRadius;
        N;
        scale_step_num;
    end
    
    methods
        
        function obj = HarrisGridPoints(in_Location,in_ScaleRadius)
            %HarrisGridPoints ��������ʵ��
            %   �˴���ʾ��ϸ˵��
            %% check input
            if((size(in_Location,1)~=size(in_ScaleRadius,1)) || size(in_Location,2)~=2 )
                error("check input size");
            end
            %% init
            obj.scale_step_num=size(in_ScaleRadius,2);
            obj.Location = repmat(in_Location,obj.scale_step_num,1);
            scales_stack=[];
            for i=1:obj.scale_step_num
                scales_stack=[scales_stack;in_ScaleRadius(:,i)];
            end
            obj.ScaleRadius = scales_stack;
            obj.N = size(scales_stack,1);
        end
       
    end
end

