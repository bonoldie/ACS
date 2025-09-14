l = findobj(gcf, 'Type', 'Legend');

subplot(311);
title("X");
l(3).Position = [ 0.89 0.95 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(312);
title("Y");
l(2).Position = [ 0.89 0.95-0.35 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(313);
title("Z");
l(1).Position = [ 0.89 0.95-0.35-0.35 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

set(gcf, "Renderer", "painters", "Position", [0 0 1000 1000])
