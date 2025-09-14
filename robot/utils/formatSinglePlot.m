l = findobj(gcf, 'Type', 'Legend');

title("Joint");
l(1).Position = [ 0.85 0.80 0 0 ];
ylabel('position (rad)');
xlabel('time (s)');

set(gcf, "Renderer", "painters", "Position", [0 0 1000 1000/3])
