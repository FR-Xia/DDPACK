function [nextX,nextU,nextTime,record] = defectLineSearch(x,u,tf,target,dyn,cost,bounds,direction,epsilon,tmin,tmax,weight,reduction,geometry)
%DEFECTLINESEARCH Evaluate bounded candidates without changing the nominal.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
n=numel(x); h=1/(n-1); bounds=bounds(:);
nextX=x;nextU=u;nextTime=tf;
baseCost=objective(x,u,tf);baseDefect=norm(x{end}-target);
baseGeometry=geometryViolation(x);geometryWeight=10*weight*double(geometry.dimension>0);
baseMerit=baseCost+weight*baseDefect+geometryWeight*baseGeometry;
timeDirection=min(tmax,max(tmin,tf+direction.time_change))-tf;
shrink=min(0.5,reduction);if ~(isfinite(shrink) && shrink>0 && shrink<1),shrink=0.5;end
trials=zeros(0,17);step=1;
record=struct('accepted',false,'cost_before',baseCost,'cost_after',baseCost, ...
    'merit_before',baseMerit,'merit_after',baseMerit,'trials',trials);
for attempt=1:24
    candidateX=cell(n,1);candidateU=u(1:n);candidateX{1}=x{1};
    candidateTime=min(tmax,max(tmin,tf+step*timeDirection));
    finite=isfinite(direction.time_change) && isfinite(candidateTime) && candidateTime>0;
    for k=1:n-1
        if ~finite,break;end
        control=u{k}+step*epsilon*direction.feedforward{k}+ ...
            direction.feedback{k}*(candidateX{k}-x{k})+ ...
            direction.time_gain{k}*(candidateTime-tf);
        if geometry.dimension>0
            for ob=1:size(geometry.centers,2)
                distance=norm(candidateX{k}(1:geometry.dimension)-geometry.centers(:,ob));
                clearance=distance^2-geometry.radii(ob)^2;
                if clearance<0
                    if distance==0,finite=false;break;end
                    adjust=(geometry.gain*abs(clearance)-2*distance)/(2*distance);
                    if size(geometry.centers,2)==1
                        control=control+adjust;
                    elseif geometry.dimension==3
                        control(2)=control(2)+adjust;
                    else
                        control=control+adjust;
                    end
                end
            end
        end
        if ~finite,break;end
        candidateU{k}=min(bounds,max(-bounds,control));
        candidateX{k+1}=candidateX{k}+h*dyn.F(candidateX{k},candidateU{k},candidateTime);
        finite=isreal(candidateX{k+1}) && all(isfinite(candidateX{k+1}(:))) && ...
            isreal(candidateU{k}) && all(isfinite(candidateU{k}(:)));
    end
    candidateCost=Inf;candidateDefect=Inf;candidateGeometry=Inf;candidateMerit=Inf;
    if finite
        candidateU{n}=candidateU{n-1};
        candidateCost=objective(candidateX,candidateU,candidateTime);
        candidateDefect=norm(candidateX{end}-target);
        candidateGeometry=geometryViolation(candidateX);
        candidateMerit=candidateCost+weight*candidateDefect+geometryWeight*candidateGeometry;
        finite=isreal(candidateCost) && isfinite(candidateMerit);
    end
    accepted=finite && candidateMerit<baseMerit-1e-12*max(1,abs(baseMerit));
    trials(end+1,:)=[attempt,step,epsilon,tf,candidateTime,weight,baseCost, ...
        candidateCost,baseDefect,candidateDefect,baseMerit,candidateMerit, ...
        baseGeometry,candidateGeometry,geometryWeight,finite,accepted]; %#ok<AGROW>
    if accepted
        nextX=candidateX;nextU=candidateU;nextTime=candidateTime;
        record.accepted=true;record.cost_after=candidateCost;record.merit_after=candidateMerit;
        break;
    end
    step=step*shrink;
end
record.trials=trials;

    function value=objective(states,controls,time)
        value=cost.phi(states{end},target);
        for j=1:n-1,value=value+cost.L(states{j},target,controls{j},time,h);end
    end
    function value=geometryViolation(states)
        value=0;
        for ob=1:size(geometry.centers,2)
            center=geometry.centers(:,ob);
            for edgeIndex=1:n-1
                a=states{edgeIndex}(1:geometry.dimension);b=states{edgeIndex+1}(1:geometry.dimension);
                edge=b-a;denominator=edge.'*edge;fraction=0;
                if denominator>0,fraction=min(1,max(0,(center-a).'*edge/denominator));end
                value=max(value,max(0,geometry.radii(ob)-norm(a+fraction*edge-center)));
            end
        end
    end
end
