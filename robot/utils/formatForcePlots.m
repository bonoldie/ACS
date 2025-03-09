l = findobj(gcf, 'Type', 'Legend');

subplot(411);
title("pose(x)");
l(4).Position = [ 0.89 0.95 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(412);
title("pose(y)");
l(3).Position = [ 0.89 0.95-0.25 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(413);
title("pose(z)");
l(2).Position = [ 0.89 0.95-0.25-0.25 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(414);
title("External wrench");
l(1).Position = [ 0.89 0.95-0.25-0.25-0.25 0 0 ];
ylabel('force (N)');
xlabel('time (s)');

set(gcf, "Renderer", "painters", "Position", [0 0 1000 1000])
