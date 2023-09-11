function multicomet3(varargin)
%COMET3 3-D Comet-like trajectories.
%   COMET3(Z) displays an animated three dimensional plot of the vector Z.
%   COMET3(X,Y,Z) displays an animated comet plot of the curve through the
%   points [X(i),Y(i),Z(i)].
%   COMET3(X,Y,Z,p) uses a comet of length p*length(Z). Default is p = 0.1.
%
%   COMET3(AX,...) plots into AX instead of GCA.
%
%   Example:
%       t = -pi:pi/500:pi;
%       comet3(sin(5*t),cos(3*t),t)
%
%   See also COMET.

%   Charles R. Denham, MathWorks, 1989.
%   Revised 2-9-92, LS and DTP; 8-18-92, 11-30-92 CBM.
%   Copyright 1984-2019 The MathWorks, Inc.

% Parse possible Axes input
[ax,args,nargs] = axescheck(varargin{:});

if nargs < 1
    error(message('MATLAB:narginchk:notEnoughInputs'));
elseif nargs > 4
    error(message('MATLAB:narginchk:tooManyInputs'));
end

% Parse the rest of the inputs
if nargs < 2, x = args{1}; end
if nargs == 2, y = args{2}; end
if nargs < 3, z = x; x = 1:length(z); y = 1:length(z); end
if nargs == 3, [x,y,z] = deal(args{:}); end
if nargs < 4, p = 0.10; end
if nargs == 4, [x,y,z,p] = deal(args{:}); end

if ~isscalar(p) || ~isreal(p) || p < 0 || p >= 1
    error(message('MATLAB:comet3:InvalidP'));
end

pause_time=0;
x = matlab.graphics.chart.internal.datachk(x);
y = matlab.graphics.chart.internal.datachk(y);
z = matlab.graphics.chart.internal.datachk(z);

ax = newplot(ax);
if ~strcmp(ax.NextPlot,'add')
    % If NextPlot is 'add', assume other objects are driving the limits.
    % Otherwise, set the limits so that the axes limits don't jump around
    % during animation.
    [minx,maxx] = minmax(x);
    [miny,maxy] = minmax(y);
    [minz,maxz] = minmax(z);
    if ~isa(ax,'matlab.graphics.axis.GeographicAxes')
        axis(ax,[minx maxx miny maxy minz maxz])
        view(ax,3);
    else
        % Latitudes are the first axis (x) and longitudes are the second
        % axis (y).
        latmin = minx;
        latmax = maxx;
        lonmin = miny;
        lonmax = maxy;
        
        % Buffer the limits for better display.
        bufferInPercent = 5;
        f = 1 + bufferInPercent / 100;
        half = f * (latmax - latmin)/2;
        latlim = [-half, half] + (latmin + latmax)/2;
        latlim(1) = max(latlim(1), -90);
        latlim(2) = min(latlim(2),  90);
        half = f * (lonmax - lonmin)/2;
        lonlim = [-half, half] + (lonmin + lonmax)/2;
        
        geolimits(ax,latlim,lonlim)
    end
end

co = get(ax,'colororder');


if size(co,1)>=3
    colors = [ co(1,:);co(2,:);co(3,:)];
    lstyle = '-';
else
    colors = repmat(co(1,:),3,1);
    lstyle ='--';
end

m = length(z);
k = round(p*m);

head = line('parent',ax,'color',colors(1,:),'marker','o', ...
    'xdata',x(1),'ydata',y(1),'zdata',z(1),'tag','head');
   
% Choose first three colors for head, body, and tail
body = animatedline('parent',ax,'color',colors(2,:),'linestyle',lstyle,...
                    'MaximumNumPoints',max(1,k),'Tag','body');
tail = animatedline('parent',ax,'color',colors(3,:),'linestyle','-',...
                    'MaximumNumPoints',1+m, 'Tag','tail');

if length(x) < 2000
    updateFcn = @()drawnow;
else
    updateFcn = @()drawnow('limitrate');
end

% Grow the body
for i = 1:k
    % Protect against deleted objects following the call to drawnow.
    if ~(isvalid(head) && isvalid(body))
        return
    end
    set(head,'xdata',x(i),'ydata',y(i),'zdata',z(i))
    addpoints(body,x(i),y(i),z(i));
    updateFcn();
    pause(pause_time)
end

% Add a drawnow to capture any events / callbacks
drawnow

% Primary loop
m = length(x);
for i = k+1:m
    % Protect against deleted objects following the call to drawnow.
    if ~(isvalid(head) && isvalid(body) && isvalid(tail))
        return
    end
    set(head,'xdata',x(i),'ydata',y(i),'zdata',z(i))
    addpoints(body,x(i),y(i),z(i));
    addpoints(tail,x(i-k),y(i-k),z(i-k));
    updateFcn();
    pause(pause_time)
end
drawnow

% Clean up the tail
for i = m+1:m+k
    % Protect against deleted objects following the call to drawnow.
    if ~isvalid(tail)
        return
    end
    addpoints(tail, x(i-k),y(i-k),z(i-k));
    updateFcn();
    pause(pause_time)
end
drawnow

% same subfunction as in comet
function [minx,maxx] = minmax(x)
minx = min(x(isfinite(x)));
maxx = max(x(isfinite(x)));
if minx == maxx
    minx = maxx-1;
    maxx = maxx+1;
end
