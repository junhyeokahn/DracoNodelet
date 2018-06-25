function ret = fn_latex_label(axis, label, font_size)

switch (axis)
    case 'x'
        xlabel(label, 'interpreter', 'latex', 'fontsize', font_size);
    case 'y'
        ylabel(label, 'interpreter', 'latex', 'fontsize', font_size);
end
set(gca, 'fontsize',font_size);
end