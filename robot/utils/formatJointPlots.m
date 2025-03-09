l = findobj(gcf, 'Type', 'Legend');

subplot(311);
title("Joint 1");
l(3).Position = [ 0.89 0.95 0 0 ];
ylabel('position (rad)');
xlabel('time (s)');

subplot(312);
title("Joint 2");
l(2).Position = [ 0.89 0.95-0.35 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(313);
title("Joint 3");
l(1).Position = [ 0.89 0.95-0.35-0.35 0 0 ];
ylabel('position (rad)');
xlabel('time (s)');

set(gcf, "Renderer", "painters", "Position", [0 0 1000 1000])
