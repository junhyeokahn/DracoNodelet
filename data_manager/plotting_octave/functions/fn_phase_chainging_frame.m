function phase_changing_frame = fn_phase_chainging_frame(phase)
j=1;
phase_changing_frame = 1;

for i = 1:length(phase)-1
  if(phase(i+1) ~= phase(i))
    phase_changing_frame(j) = i+1;
    j = j+1;
  end
end

end