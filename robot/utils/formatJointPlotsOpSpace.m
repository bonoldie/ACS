l = findobj(gcf, 'Type', 'Legend');

subplot(611);
title("X");
l(6).Position = [ 0.89 0.95-(0.165*0) 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(612);
title("Y");
l(5).Position = [ 0.89 0.95-(0.165*1) 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(613);
title("Z");
l(4).Position = [ 0.89 0.95-(0.165*2) 0 0 ];
ylabel('position (m)');
xlabel('time (s)');

subplot(614);
title("\textbf{$\phi$}", Interpreter="latex", FontSize=15);
l(3).Position = [ 0.89 0.95-(0.165*3) 0 0 ];
ylabel('position (rad)');
xlabel('time (s)');

subplot(615);
title("\textbf{$\theta$}", Interpreter="latex", FontSize=15);
l(2).Position = [ 0.89 0.95-(0.165*4) 0 0 ];
ylabel('position (rad)');
xlabel('time (s)');

subplot(616);
title("\textbf{$\psi$}", Interpreter="latex", FontSize=15);
l(1).Position = [ 0.89 0.95-(0.165*5) 0 0 ];
ylabel('position (rad)');
xlabel('time (s)');


set(gcf, "Renderer", "painters", "Position", [0 0 1000 1000])
