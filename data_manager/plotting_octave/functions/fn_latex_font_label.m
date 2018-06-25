function ret = fn_latex_font_label(axis, label, font_size)

switch (axis)
    case 'x'
        xlabel(label, 'fontname', 'Times New Roman', 'fontsize', font_size);
    case 'y'
        ylabel(label, 'fontname', 'Times New Roman', 'fontsize', font_size);
end
set(gca, 'fontsize',font_size);
end