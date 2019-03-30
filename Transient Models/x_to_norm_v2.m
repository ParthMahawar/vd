%********************************************************
function [x_norm] = x_to_norm_v2(x_begin,x_end)

if nargin ~= 2
     error('Wrong number of input arguments.')
end

h_axes = get(gcf,'CurrentAxes');    %get axes handle

axesoffsets = get(h_axes,'Position');
x_axislimits = get(gca, 'xlim');     %get axes extremeties.
x_dir = get(gca,'xdir');
x_scale = get(gca,'xscale');

%get axes length
x_axislength_lin = abs(x_axislimits(2) - x_axislimits(1));


if strcmp(x_dir,'normal')
  if strcmp(x_scale,'log')
    x_axislength_log = abs(log10(x_axislimits(2)) - log10(x_axislimits(1)));  
    x_begin_norm = axesoffsets(1)+axesoffsets(3)*abs(log10(x_begin)-log10(x_axislimits(1)))/(x_axislength_log);
    x_end_norm = axesoffsets(1)+axesoffsets(3)*abs(log10(x_end)-log10(x_axislimits(1)))/(x_axislength_log);

    x_norm = [x_begin_norm x_end_norm];
  elseif strcmp(x_scale,'linear')
    x_begin_norm = axesoffsets(1)+axesoffsets(3)*abs((x_begin-x_axislimits(1))/x_axislength_lin);
    x_end_norm = axesoffsets(1)+axesoffsets(3)*abs((x_end-x_axislimits(1))/x_axislength_lin);

    x_norm = [x_begin_norm x_end_norm];  
  else
      error('use only lin or log in quotes for scale')
  end   
   
elseif strcmp(x_dir,'reverse')
    if strcmp(x_scale,'log')
        x_axislength_log = abs(log10(x_axislimits(2)) - log10(x_axislimits(1)));
        x_begin_norm = axesoffsets(1)+axesoffsets(3)*abs(log10(x_axislimits(2))-log10(x_begin))/(x_axislength_log);
        x_end_norm = axesoffsets(1)+axesoffsets(3)*abs(log10(x_axislimits(2))-log10(x_end))/(x_axislength_log);

        x_norm = [x_begin_norm x_end_norm]; 
    elseif strcmp(x_scale,'linear')
        x_begin_norm = axesoffsets(1)+axesoffsets(3)*abs((x_axislimits(2)-x_begin)/x_axislength_lin);
        x_end_norm = axesoffsets(1)+axesoffsets(3)*abs((x_axislimits(2)-x_end)/x_axislength_lin);

        x_norm = [x_begin_norm x_end_norm];
    else
        error('use only lin or log in quotes for scale')
    end
else
    error('use only r or nr in quotes for reverse')
end