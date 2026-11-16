function report = checkDerivatives(problem,state,control,varargin)
%CHECKDERIVATIVES Independently audit the selected model callback contract.
%   REPORT = ddpack.checkDerivatives(P,X,U,'Algorithm',ID) compares analytic
%   derivatives with component-scaled central differences. It does not solve
%   the problem, modify callbacks, or consume the random-number stream.
% Copyright (c) 2026 Feiran Xia
% Beijing Institute of Technology, China
% SPDX-License-Identifier: BSD-3-Clause
parser=inputParser;
positive=@(x)isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x)&&x>0;
addParameter(parser,'Algorithm','free_time_pddp',@(x)ischar(x)||(isstring(x)&&isscalar(x)));
addParameter(parser,'DifferenceStep',1e-5,positive);
addParameter(parser,'AbsoluteTolerance',1e-7,positive);
addParameter(parser,'RelativeTolerance',5e-6,positive);
addParameter(parser,'TimeScale',1.7,positive);
addParameter(parser,'IntegrationStep',0.037,positive);
parse(parser,varargin{:});opt=parser.Results;
[identity,~]=ddpack.resolveAlgorithm(opt.Algorithm,'Warn',false);
algorithm=identity.algorithm_id;fixed=strcmp(algorithm,'penalty_free_time_ddp');
if ~isstruct(problem)||~isscalar(problem)||~all(isfield(problem,{'xf','dynamics','cost'}))
    error('ddpack:derivatives:InvalidProblem','A scalar problem with xf, dynamics and cost is required.');
end
validateattributes(state,{'numeric'},{'column','real','finite','nonempty'});
validateattributes(control,{'numeric'},{'column','real','finite','nonempty'});
validateattributes(problem.xf,{'numeric'},{'column','real','finite','numel',numel(state)});
nx=numel(state);nu=numel(control);target=problem.xf;t=opt.TimeScale;h=opt.IntegrationStep;
d=problem.dynamics;c=problem.cost;
rows=struct('name',{},'category',{},'passed',{},'max_absolute_error',{}, ...
    'max_scaled_error',{},'max_tolerance_ratio',{},'message',{});
if fixed
    flow=@(x,u)d.F(x,u);transition=@(x,u)x+h*flow(x,u);
    add('dynamics.Phi',@()d.Phi(state,control,h),@()difference(@(x)transition(x,control),state,nx),[nx,nx]);
    add('dynamics.Beta',@()d.Beta(state,control,h),@()difference(@(u)transition(state,u),control,nx),[nx,nu]);
    running=@(x,u,~)c.L(x,target,u,h);
    gx=@(x,u,~)c.L_x(x,target,u,h);gu=@(x,u,~)c.L_u(x,u,h);
    hxx=@()c.L_xx(state,target,control,h);huu=@()c.L_uu(state,control,h);
    constraintStep=h;
else
    flow=@(x,u)d.F(x,u,t);transition=@(x,u)x+h*flow(x,u);
    add('dynamics.Phi',@()d.Phi(state,control,t,h),@()difference(@(x)transition(x,control),state,nx),[nx,nx]);
    if strcmp(algorithm,'free_time_pddp'),influence='Beta';else,influence='beta';end
    add(['dynamics.' influence],@()feval(influence,d,state,control,t,h),@()difference(@(u)transition(state,u),control,nx),[nx,nu]);
    if any(strcmp(algorithm,{'free_time_pddp','waypoint_pddp'})),timeMethod='theta';else,timeMethod='gamma';end
    add(['dynamics.' timeMethod],@()feval(timeMethod,d,state,control,t,h), ...
        @()difference(@(scale)state+h*d.F(state,control,scale),t,nx),[nx,1]);
    running=@(x,u,scale)c.L(x,target,u,scale,h);
    gx=@(x,u,scale)c.L_x(x,target,u,scale,h);gu=@(x,u,scale)c.L_u(x,u,scale,h);
    hxx=@()c.L_xx(state,target,control,t,h);huu=@()c.L_uu(state,control,t,h);
    constraintStep=t*h;
    add('terminal.phi_t',@()c.phi_t(state,target),@()difference(@(~)c.phi(state,target),t,1),[1,1]);
    add('terminal.phi_xt',@()c.phi_xt(state,target),@()difference(@(~)c.phi_x(state,target),t,nx),[nx,1]);
    add('terminal.phi_tt',@()c.phi_tt(state,target),@()difference(@(~)c.phi_t(state,target),t,1),[1,1]);
    add('running.L_t',@()c.L_t(state,target,control,t,h),@()difference(@(scale)running(state,control,scale),t,1),[1,1]);
    add('running.L_xt',@()c.L_xt(state,target,control,t,h),@()difference(@(scale)gx(state,control,scale),t,nx),[nx,1]);
    add('running.L_ut',@()c.L_ut(state,control,t,h),@()difference(@(scale)gu(state,control,scale),t,nu),[nu,1]);
    add('running.L_tt',@()c.L_tt(state,target,control,t), ...
        @()difference(@(scale)c.L_t(state,target,control,scale,h),t,1),[1,1]);
end
contract('dynamics.F',@()flow(state,control),[nx,1]);
contract('terminal.phi',@()c.phi(state,target),[1,1]);
contract('running.L',@()running(state,control,t),[1,1]);
add('terminal.phi_x',@()c.phi_x(state,target),@()difference(@(x)c.phi(x,target),state,1).',[nx,1]);
add('terminal.phi_xx',@()c.phi_xx(state,target),@()difference(@(x)c.phi_x(x,target),state,nx),[nx,nx]);
add('terminal.symmetry',@()c.phi_xx(state,target),@()c.phi_xx(state,target).',[nx,nx]);
add('running.L_x',@()gx(state,control,t),@()difference(@(x)running(x,control,t),state,1).',[nx,1]);
add('running.L_u',@()gu(state,control,t),@()difference(@(u)running(state,u,t),control,1).',[nu,1]);
add('running.L_xx',hxx,@()difference(@(x)gx(x,control,t),state,nx),[nx,nx]);
add('running.L_uu',huu,@()difference(@(u)gu(state,u,t),control,nu),[nu,nu]);
add('running.L_xu',@()c.L_xu(state,control,h),@()difference(@(u)gx(state,u,t),control,nx),[nx,nu]);
add('running.L_ux',@()c.L_ux(state,control,h),@()difference(@(x)gu(x,control,t),state,nu),[nu,nx]);
add('running.state_symmetry',hxx,@()transpose(hxx()),[nx,nx]);
add('running.control_symmetry',huu,@()transpose(huu()),[nu,nu]);
if isfield(problem,'constraints')&&~isempty(problem.constraints)
    for index=1:numel(problem.constraints)
        constraint=problem.constraints(index);
        contract(sprintf('constraint.%d.G',index),@()constraint.G(state,control,constraintStep),[1,1]);
        add(sprintf('constraint.%d.G_x',index),@()constraint.G_x(state,control,constraintStep), ...
            @()difference(@(x)constraint.G(x,control,constraintStep),state,1),[1,nx]);
        add(sprintf('constraint.%d.G_u',index),@()constraint.G_u(state,control,constraintStep), ...
            @()difference(@(u)constraint.G(state,u,constraintStep),control,1),[1,nu]);
    end
end
report=struct('success',all([rows.passed]),'algorithm',algorithm,'state',state,'control',control, ...
    'time_scaling_checked',~fixed, ...
    'options',opt,'checks',rows,'checked_count',numel(rows),'failed_count',sum(~[rows.passed]), ...
    'scope','Local callback consistency at the supplied point; no convergence or global correctness proof.', ...
    'excluded_checks',{{'Dynamics second derivatives (not consumed by these recursions)', ...
    'Derivatives of bounds, clipping, heuristic correction or complete solver iterations'}});

    function add(name,analytic,numerical,shape)
        row=struct('name',name,'category','derivative','passed',false, ...
            'max_absolute_error',Inf,'max_scaled_error',Inf,'max_tolerance_ratio',Inf,'message','');
        try
            a=value(analytic,shape);b=value(numerical,shape);delta=abs(a-b);
            scale=max(abs(a),abs(b));ratio=delta./(opt.AbsoluteTolerance+opt.RelativeTolerance*scale);
            row.max_absolute_error=max(delta(:));row.max_scaled_error=max(delta(:)./max(1,scale(:)));
            row.max_tolerance_ratio=max(ratio(:));row.passed=all(ratio(:)<=1);
            if ~row.passed,row.message='Analytic derivative differs from the independent reference.';end
        catch exception
            row.category='callback_contract';row.message=[exception.identifier ': ' exception.message];
        end
        rows(end+1)=row;
    end
    function contract(name,callback,shape)
        row=struct('name',name,'category','value_contract','passed',false, ...
            'max_absolute_error',0,'max_scaled_error',0,'max_tolerance_ratio',0,'message','');
        try
            value(callback,shape);row.passed=true;
        catch exception
            row.message=[exception.identifier ': ' exception.message];
        end
        rows(end+1)=row;
    end
    function output=value(callback,shape)
        output=callback();
        if ~isnumeric(output)||~isreal(output)||~isequal(size(output),shape)||any(~isfinite(output(:)))
            error('ddpack:derivatives:CallbackContract','Expected a finite real %d-by-%d callback result.',shape(1),shape(2));
        end
    end
    function jacobian=difference(callback,point,outputs)
        jacobian=zeros(outputs,numel(point));
        for column=1:numel(point)
            width=opt.DifferenceStep*max(1,abs(point(column)));plus=point;minus=point;
            plus(column)=plus(column)+width;minus(column)=minus(column)-width;
            a=value(@()callback(plus),[outputs,1]);b=value(@()callback(minus),[outputs,1]);
            jacobian(:,column)=(a-b)/(2*width);
        end
    end
end
