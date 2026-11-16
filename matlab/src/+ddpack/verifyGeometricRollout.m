function audit = verifyGeometricRollout(problem, controls, time)
%VERIFYGEOMETRICROLLOUT Analytic ZOH rollout for the supported kinematics.
% Clearance lower bound uses distance's speed-Lipschitz bound between samples.
% This is a floating-point engineering bound, not interval arithmetic.
audit = struct('applicable',false,'sampled_clearance',NaN, ...
    'clearance_lower_bound',NaN,'terminal_error',NaN,'substeps',64);
planar = isa(problem.dynamics,'Dynamics2D_cbf_mspddp');
flight = isa(problem.dynamics,'KinematicFlightDynamics');
if ~(planar || flight) || isempty(problem.obstacle_centers), return; end
audit.applicable=true; x=problem.x0; minimum=inf;
if planar, dim=2; speed=problem.dynamics.v; else, dim=3; speed=x(4); end
maximumStep=0;
for k=1:numel(time)-1
    h=(time(k+1)-time(k))/audit.substeps; maximumStep=max(maximumStep,h);
    for j=0:audit.substeps
        y=advance(x,controls(:,k),j*h,planar,speed);
        gaps=sqrt(sum((y(1:dim)-problem.obstacle_centers).^2,1)) ...
            -problem.obstacle_radii(:).'-problem.safety_margin;
        minimum=min(minimum,min(gaps));
    end
    x=y;
end
audit.sampled_clearance=minimum;
% Every instant is at most half a sample interval from a checked endpoint.
audit.clearance_lower_bound=minimum-abs(speed)*maximumStep/2 ...
    -1e-10*(1+norm(problem.x0)+abs(speed)*(time(end)-time(1)));
audit.terminal_error=norm(x-problem.xf);
end

function y=advance(x,u,h,planar,speed)
y=x;
if planar
    y(1)=x(1)+speed*integralTrig(x(3),u(1),h,false);
    y(2)=x(2)+speed*integralTrig(x(3),u(1),h,true);
    y(3)=x(3)+h*u(1);
else
    g=x(5); p=x(6); a=u(2); b=u(1);
    y(1)=x(1)+speed/2*(integralTrig(g-p,a-b,h,false)+integralTrig(g+p,a+b,h,false));
    y(2)=x(2)+speed/2*(integralTrig(p+g,b+a,h,true)+integralTrig(p-g,b-a,h,true));
    y(3)=x(3)+speed*integralTrig(g,a,h,true);
    y(5:6)=x(5:6)+h*[a;b];
end
end

function value=integralTrig(angle,rate,h,useSin)
z=rate*h/2;
if abs(z)<1e-4, factor=1-z*z/6+z^4/120; else, factor=sin(z)/z; end
if useSin, value=h*factor*sin(angle+z); else, value=h*factor*cos(angle+z); end
end
