%author: Girish Ratanpal, ECE, UVa. 
%transform axes units to normalized units for 2-D figures only. 
%works for linear,log scales and also reverse axes.
%DATE: JAN. 6, 2006. previous version: aug 12, 2005.
%
%
%FUNCTION DESCRIPTION:
% function [y_norm] = y_to_norm_v2(y_begin,y_end)
% function returns a 1x2 array, y_norm, with normalized unitsy_begin and
% y_end for the annotation object. 
%     
% function arguments:
%1. y_begin: enter the point where the annotation object begins on the axis 
%         using axis units
%2. y_end: end of annotation object, again in axis units. 
%         

%EXAMPLE: first plot the graph on the figure. 
%then use the following command for placing an arrow:
%h_object =
%annotation('arrow',x_to_norm_v2(x_begin,x_end),y_to_norm_v2(y_begin,y_end));
%******************************************
%CODE FOR x_norm_v2() IS COMMENTED AT THE END.
%******************************************

function [y_norm] = y_to_norm_v2(y_begin,y_end)

if nargin ~= 2
     error('Wrong number of input arguments! y_to_norm_v2(y_begin,y_end)')
end

h_axes = get(gcf,'CurrentAxes');    %get axes handle.
axesoffsets = get(h_axes,'Position');%get axes position on the figure. 
y_axislimits = get(gca, 'ylim');     %get axes extremeties.
y_dir = get(gca,'ydir');
y_scale = get(gca,'YScale');

%get axes length
y_axislength_lin = abs(y_axislimits(2) - y_axislimits(1));


if strcmp(y_dir,'normal')      %axis not reversed
  if strcmp(y_scale,'log')
    %get axes length in log scale.
    y_axislength_log = abs(log10(y_axislimits(2)) - log10(y_axislimits(1)));  
    
    %normalized distance from the lower left corner of figure. 
    y_begin_norm = axesoffsets(2)+axesoffsets(4)*abs(log10(y_begin)-log10(y_axislimits(1)))/(y_axislength_log);
    y_end_norm = axesoffsets(2)+axesoffsets(4)*abs(log10(y_end)-log10(y_axislimits(1)))/(y_axislength_log);

    y_norm = [y_begin_norm y_end_norm];
  elseif strcmp(y_scale,'linear')%linear scale.
    %normalized distance from the lower left corner of figure.  
    y_begin_norm = axesoffsets(2)+axesoffsets(4)*abs((y_begin-y_axislimits(1))/y_axislength_lin);
    y_end_norm = axesoffsets(2)+axesoffsets(4)*abs((y_end-y_axislimits(1))/y_axislength_lin);

    y_norm = [y_begin_norm y_end_norm];  
  else
      error('use only lin or log in quotes for scale')
  end   
   
elseif strcmp(ydir,'reverse')  %axis is reversed
    if strcmp(y_scale,'log')
        %get axes length in log scale.
        y_axislength_log = abs(log10(y_axislimits(2)) - log10(y_axislimits(1)));
        %normalized distance from the lower left corner of figure. 
        y_begin_norm = axesoffsets(2)+axesoffsets(4)*abs(log10(y_axislimits(2))-log10(y_begin))/(y_axislength_log);
        y_end_norm = axesoffsets(2)+axesoffsets(4)*abs(log10(y_axislimits(2))-log10(y_end))/(y_axislength_log);

        y_norm = [y_begin_norm y_end_norm]; 
    elseif strcmp(y_scale,'linear')
        %normalized distance from the lower left corner of figure. 
        y_begin_norm = axesoffsets(2)+axesoffsets(4)*abs((y_axislimits(2)-y_begin)/y_axislength_lin);
        y_end_norm = axesoffsets(2)+axesoffsets(4)*abs((y_axislimits(2)-y_end)/y_axislength_lin);

        y_norm = [y_begin_norm y_end_norm];
    else
        error('use only lin or log in quotes for scale')
    end
else
    error('use only r or nr in quotes for reverse')
end



