function drawTrajectory(varargin)
[~,args,nargs] = axescheck(varargin{:});
% nargs input nums// args{1}=path1,args{2}=path2,...

trajectory=cell(1,nargs);time_limit=0;
for i=1:nargs
    [~,~,trajectory{i}]=getTrajectory(args{i});
    time_limit=max(time_limit,size(trajectory{i},2));
end
for i=1:time_limit
    for j=1:nargs
        if i<size(trajectory{j},2)
            multicomet3(trajectory{j}(1,i:i+1),trajectory{j}(2,i:i+1),trajectory{j}(3,i:i+1))
        end
    end
    pause(0.5)
end
end

