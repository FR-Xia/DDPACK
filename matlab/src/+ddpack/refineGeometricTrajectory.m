function [raw, verification] = refineGeometricTrajectory(raw, problem)
%REFINEGEOMETRICTRAJECTORY Optional multistart, bounded shooting refinement.
% Piecewise-linear CONTROL knots are optimized, then dynamics are reintegrated.
% This is a distinct bounded-time Gauss-Newton stage, not a plotting filter
% or a claim of global optimality. The original quadratic cost is preserved.
verification=struct('jacobian_scaled_error',NaN);
checkJacobian=nargout>1;
if ~isfield(problem,'geometric_refinement_iterations') || problem.geometric_refinement_iterations==0
    return;
end
assert(all(cellfun(@(name) isprop(problem.cost,name),{'Q_f','Q_s','R'})), ...
    'ddpack:refinement:QuadraticCost','Refinement requires a quadratic cost.');
x0=problem.x0; target=problem.xf; bounds=problem.u_max(:);
if iscell(raw.u), u=cell2mat(raw.u(:).'); else, u=raw.u; end
n=size(u,2); nu=size(u,1); nx=numel(x0); intervals=n-1;
assert(intervals>=3,'ddpack:refinement:Grid','Refinement needs at least four state nodes.');
tf=raw.final_time; originalTime=tf;
freeTime=problem.geometric_refinement_free_time;
variables=nu*min(problem.geometric_refinement_knots,intervals)+double(freeTime);
lower=[repmat(-bounds,min(problem.geometric_refinement_knots,intervals),1)];upper=-lower;
if freeTime, lower=[lower;problem.minimum_time];upper=[upper;problem.maximum_time];end
knots=min(problem.geometric_refinement_knots,intervals);
basis=interp1(linspace(0,1,knots),eye(knots),linspace(0,1,intervals),'linear');
maps=cell(1,intervals);
for k=1:intervals, maps{k}=[kron(basis(k,:),eye(nu)),zeros(nu,double(freeTime))]; end
q=chol(problem.cost.Q_s+1e-14*eye(nx)); r=chol(problem.cost.R+1e-14*eye(nu));
f=chol(problem.cost.Q_f+1e5*eye(nx));
lambda=problem.geometric_control_rate_weight;
positionDimension=size(problem.obstacle_centers,1);
radii=problem.obstacle_radii(:).'+problem.safety_margin;
% Independent canonical restart seeds avoid amplifying a legacy seed's
% roundoff-sensitive choice of which side of a symmetric obstacle to pass.
seeds={};
for axis=1:nu
    for sign=[1,-1]
        z=zeros(nu,knots);
        z(axis,:)=sign*0.4*bounds(axis)*cos(2*pi*linspace(0,1,knots));
        seeds{end+1}=z; %#ok<AGROW>
    end
end
[baseX,~,baseJ,baseSmooth,baseClear]=evaluate(u(:,1:intervals));
bestJ=baseJ; bestScore=baseJ+baseSmooth; bestU=u(:,1:intervals); bestX=baseX;bestTime=tf;
requireDecrease=true;
if isfield(problem,'require_cost_decrease'),requireDecrease=problem.require_cost_decrease;end
initialObjective=baseJ;
if isfield(raw,'initial_cost'),initialObjective=raw.initial_cost;
elseif ~isempty(raw.J),initialObjective=raw.J(1);end
baselineFeasible=physicallyFeasible(baseX,u(:,1:intervals),tf,baseJ);
baselineCostAllowed=~requireDecrease || baseJ<=initialObjective*(1+1e-10);
bestEligible=baselineFeasible && baselineCostAllowed;
records=zeros(numel(seeds),7); selected=0; bestHistory=[];
for start=1:numel(seeds)
    z=seeds{start}(:);
    if freeTime, z=[z;min(problem.maximum_time,max(tf,problem.tf*1.04))]; end
    z=min(upper,max(lower,z)); damping=1e-3; history=[];
    for iteration=1:problem.geometric_refinement_iterations
        [control,time]=unpack(z);
        [~,residual,~,~,~,jacobian]=evaluate(control,time);
        if checkJacobian && start==1 && iteration==1
            numerical=zeros(size(jacobian));
            for column=1:variables
                plus=z;minus=z;plus(column)=plus(column)+1e-6;minus(column)=minus(column)-1e-6;
                [up,tp]=unpack(plus);[um,tm]=unpack(minus);
                [~,rp]=evaluate(up,tp);[~,rm]=evaluate(um,tm);
                numerical(:,column)=(rp-rm)/2e-6;
            end
            verification.jacobian_scaled_error=max(abs(numerical-jacobian),[],'all')/max(1,max(abs(jacobian),[],'all'));
        end
        merit=0.5*(residual.'*residual); history(end+1)=merit; %#ok<AGROW>
        gradient=jacobian.'*residual;
        projected=z-min(upper,max(lower,z-gradient));
        if norm(projected,Inf)<1e-7, break; end
        normal=jacobian.'*jacobian; accepted=false;
        for retry=1:12
            factor=chol(normal+damping*diag(max(1,diag(normal))),'lower');
            direction=-(factor.'\(factor\gradient));
            candidate=min(upper,max(lower,z+direction));
            [candidateControl,candidateTime]=unpack(candidate);
            [~,candidateResidual]=evaluate(candidateControl,candidateTime);
            candidateMerit=0.5*(candidateResidual.'*candidateResidual);
            if isfinite(candidateMerit) && candidateMerit < merit-1e-12*max(1,merit)
                z=candidate; damping=max(1e-5,damping/3); accepted=true; break;
            end
            damping=damping*10;
        end
        if ~accepted, break; end
    end
    [control,time]=unpack(z);
    [x,~,j,smooth,clearance]=evaluate(control,time); error=norm(x(:,end)-target);
    feasible=physicallyFeasible(x,control,time,j);
    records(start,:)=[start,iteration,j,smooth,error,clearance,feasible];
    costAllowed=~requireDecrease || j<=initialObjective*(1+1e-10);
    if feasible && costAllowed && (~bestEligible || j+smooth<bestScore-1e-8*max(1,abs(bestScore)))
        bestScore=j+smooth;bestJ=j;[bestU,bestTime]=unpack(z);bestX=x;selected=start;bestHistory=history;
        bestEligible=true;
    end
end
raw.x=mat2cell(bestX,nx,ones(1,n)).';
raw.u=mat2cell([bestU,bestU(:,end)],nu,ones(1,n)).';
raw.returned_cost=bestJ;raw.final_time=bestTime;
raw.t=linspace(0,1,n);
if ~isfield(raw,'initial_cost') && ~isempty(raw.J)
    raw.initial_cost=raw.J(1);
end
% Preserve the primary PDDP/projection history as a separate phase.
raw.primary_trial_cost_history=raw.J;
raw.J=[raw.J(:);bestJ]; raw.J_accepted=bestJ;
raw.geometric_refinement=struct('enabled',true,'method','bounded_multistart_shooting_gauss_newton', ...
    'initial_time',originalTime,'returned_time',bestTime,'free_time',freeTime,'knots',knots,'rate_weight',lambda,'selected_start',selected, ...
    'baseline_objective',baseJ,'returned_objective',bestJ, ...
    'baseline_feasible',baselineFeasible,'baseline_cost_allowed',baselineCostAllowed, ...
    'initial_objective',initialObjective,'cost_decrease_required',requireDecrease, ...
    'selection_policy','physical_feasibility_then_initial_cost_policy_then_regularized_score', ...
    'baseline_rate_cost',baseSmooth,'returned_rate_cost',bestScore-bestJ, ...
    'baseline_clearance',baseClear,'starts',records,'penalty_history',bestHistory, ...
    'record_columns',{{'start','iterations','objective','rate_cost','terminal_error','clearance','feasible'}});
if norm(bestX(:,end)-target)<=problem.terminal_tolerance && selected>0
    raw.termination_reason='geometric_refinement_feasible';
    raw.convergence_status='converged_geometric_refinement';
end

    function [control,time]=unpack(z)
        control=reshape(z(1:nu*knots),nu,knots)*basis.'; time=tf;
        if freeTime, time=z(end);end
    end

    function feasible=physicallyFeasible(x,control,time,j)
        candidate=struct('x',{mat2cell(x,nx,ones(1,n)).'}, ...
            'u',{mat2cell([control,control(:,end)],nu,ones(1,n)).'}, ...
            't',linspace(0,1,n),'final_time',time,'J',j, ...
            'initial_cost',j,'returned_cost',j,'error',0);
        checkProblem=problem;checkProblem.require_cost_decrease=false;
        algorithm='geometric_safety_pddp_2d';
        if positionDimension==3,algorithm='geometric_safety_pddp_3d';end
        checked=ddpack.normalizeSolution(candidate,algorithm,checkProblem,0);
        feasible=checked.metadata.metrics.numerical_checks_pass;
    end

    function [x,residual,j,smooth,clearance,jacobian]=evaluate(control,time)
        if nargin<2,time=tf;end
        h=time/intervals;
        needJac=nargout>=6;
        x=zeros(nx,n);x(:,1)=x0;sensitivity=zeros(nx,variables);sx=cell(1,n);
        if needJac, sx{1}=sensitivity; end
        for index=1:intervals
            if needJac
                sensitivity=problem.dynamics.Phi(x(:,index),control(:,index),time,1/intervals)*sensitivity+ ...
                    problem.dynamics.beta(x(:,index),control(:,index),time,1/intervals)*maps{index};
                if freeTime, sensitivity(:,end)=sensitivity(:,end)+problem.dynamics.F(x(:,index),control(:,index),1)/intervals;end
                sx{index+1}=sensitivity;
            end
            x(:,index+1)=x(:,index)+h*problem.dynamics.F(x(:,index),control(:,index),1);
        end
        stateResidual=sqrt(h)*q*(x(:,1:intervals)-target);
        controlResidual=sqrt(h)*r*control;
        rateResidual=sqrt(lambda/h)*diff(control,1,2);
        terminal=x(:,end)-target;
        residual=[stateResidual(:);controlResidual(:);rateResidual(:);f*terminal];
        j=0.5*terminal.'*problem.cost.Q_f*terminal+ ...
            0.5*(sum(stateResidual(:).^2)+sum(controlResidual(:).^2));
        smooth=0.5*sum(rateResidual(:).^2);
        if needJac
            js=zeros(nx*intervals,variables);ju=zeros(nu*intervals,variables);jr=zeros(nu*(intervals-1),variables);
            for index=1:intervals
                js((index-1)*nx+(1:nx),:)=sqrt(h)*q*sx{index};
                ju((index-1)*nu+(1:nu),:)=sqrt(h)*r*maps{index};
                if index<intervals, jr((index-1)*nu+(1:nu),:)=sqrt(lambda/h)*(maps{index+1}-maps{index}); end
            end
            jacobian=[js;ju;jr;f*sx{n}];
            if freeTime
                jacobian(1:nx*intervals+nu*intervals,end)=jacobian(1:nx*intervals+nu*intervals,end)+[stateResidual(:);controlResidual(:)]/(2*time);
                rows=nx*intervals+nu*intervals+(1:numel(rateResidual));
                jacobian(rows,end)=jacobian(rows,end)-rateResidual(:)/(2*time);
            end
        end
        clearance=Inf;
        for obstacle=1:size(problem.obstacle_centers,2)
            for index=1:intervals
                a=x(1:positionDimension,index);delta=x(1:positionDimension,index+1)-a;
                offset=a-problem.obstacle_centers(:,obstacle);
                fraction=min(1,max(0,-offset.'*delta/max(delta.'*delta,1e-30)));
                closest=offset+fraction*delta;distance=norm(closest);
                gap=distance-radii(obstacle);clearance=min(clearance,gap);
                residual(end+1,1)=sqrt(1e5)*max(0,0.005-gap); %#ok<AGROW>
                if needJac
                    row=zeros(1,variables);
                    if gap<0.005 && distance>1e-12
                        positionSensitivity=(1-fraction)*sx{index}(1:positionDimension,:)+fraction*sx{index+1}(1:positionDimension,:);
                        row=-sqrt(1e5)*(closest.'/distance)*positionSensitivity;
                    end
                    jacobian(end+1,:)=row; %#ok<AGROW>
                end
            end
        end
    end
end
