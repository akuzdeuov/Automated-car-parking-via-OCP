function vizualization(x,R,sigma,N,x_constr,xT)
%-------------------------------------------------------------------------
% x - matrix with all states (Nx3)
% R - radius of the obstacles and circles on the car (meter)
% sigma - distance between centers of the circles on the car (meter)
% N - number of samples
% x_constr - coordinates of the two obstacles [x1 y1; x2 y2] (meter)
%-------------------------------------------------------------------------

% angle vector for drawing circles  around obstacles
ang=0:0.01:2*pi; 
% x and y cooridnates for drawing obstacles represented as a circle
xc=R*cos(ang);
yc=R*sin(ang);

for ind = 1:N+1
    %plot the right obstacle
    plot(x_constr(1,1)+xc, x_constr(1,2)+yc, 'Color', 'r', 'LineWidth',2);
    axis([-5 5 -5 5]);
    hold on 
    %plot the left obstacle
    plot(x_constr(2,1)+xc, x_constr(2,2)+yc, 'Color', 'r', 'LineWidth',2);
    axis([-5 5 -5 5]);
    hold on 
    %plot the back circle
    plot(x(ind,1)-sigma*cos(x(ind,3))+xc, x(ind,2)-sigma*sin(x(ind,3))+yc, 'Color', 'b', 'LineWidth',2);
    axis([-5 5 -5 5]);
    hold on 
    %plot the middle circle
    plot(x(ind,1)+xc, x(ind,2)+yc, 'Color', 'b', 'LineWidth',2);
    axis([-5 5 -5 5]);
    hold on
    %plot the front circle
    plot(x(ind,1)+sigma*cos(x(ind,3))+xc, x(ind,2)+sigma*sin(x(ind,3))+yc, 'Color', 'g', 'LineWidth',2);
    axis([-5 5 -5 5]);
    grid on
    xlabel('x'); ylabel('y');
    hold off 
    
    % show errors in states xT - x_current 
    dx1 = ['Error px = ',num2str(xT(1) - x(ind,1)), ' meter'];
    dx2 = ['Error py = ',num2str(xT(2) - x(ind,2)), ' meter'];
    dx3 = ['Error theta = ',num2str(xT(3) - x(ind,3)), ' radian'];
    
    text(-4, -3, dx1,'HorizontalAlignment','left');
    text(-4, -3.5, dx2,'HorizontalAlignment','left');
    text(-4, -4, dx3,'HorizontalAlignment','left');
    
    pause(0.5)
end