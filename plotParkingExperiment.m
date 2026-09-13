function figures = plotParkingExperiment(experiment, vehicle)
%plotParkingExperiment Create three invisible Step 10 evidence figures.
% F = plotParkingExperiment(EXPERIMENT,VEHICLE) returns xy/errors/inputs
% figure handles. The caller exports/closes them. EXPERIMENT contains scenario,
% plan, trajectory, simulation and evaluation from the accepted pipeline.
% No graphics/workspace state is required. Unavailable trajectories are rejected.
% Only XY presentation uses scenario.framePoseWorld; all stored states/errors
% remain parking-local. Steering is drawn per segment using left-hold stairs,
% with both stopped-join values and no connecting line across the jump.
% Start/target wheels show their analytic endpoint steering commands.
% These scene drawings do not constitute swept-body collision validation.

    if ~strcmp(experiment.plan.status.code,'feasible') || ...
            ~strcmp(experiment.trajectory.status,'success') || ...
            ~strcmp(experiment.simulation.status,'success') || isempty(experiment.trajectory.q)
        error('plotParkingExperiment:UnavailableTrajectory', ...
            'An accepted plan and successful timing/simulation are required.');
    end
    figures = struct();
    try
        figures.xy = newFigure('XY trajectory',1050,560);
        drawXY(figures.xy,experiment,vehicle);
        figures.errors = newFigure('Tracking errors',900,490);
        drawErrors(figures.errors,experiment);
        figures.inputs = newFigure('Steering and speed',900,650);
        drawInputs(figures.inputs,experiment,vehicle);
        finishFigureStyle(figures.xy);
        finishFigureStyle(figures.errors);
        finishFigureStyle(figures.inputs);
    catch exception
        handles = struct2cell(figures);
        for k = 1:numel(handles)
            if isgraphics(handles{k},'figure')
                close(handles{k});
            end
        end
        rethrow(exception)
    end
end

function fig = newFigure(name,width,height)
    fig = figure('Name',name,'NumberTitle','off','Visible','off', ...
        'Color','white','Units','pixels','Position',[80,80,width,height]);
end

function styleAxes(ax)
    set(ax,'FontName','Arial','FontSize',11,'LineWidth',0.8, ...
        'Box','on','XGrid','on','YGrid','on','GridAlpha',0.12, ...
        'Color','white','XColor',[0.15,0.15,0.15],'YColor',[0.15,0.15,0.15], ...
        'GridColor',[0.15,0.15,0.15]);
    hold(ax,'on');
end

function drawXY(fig,e,vehicle)
    layout = tiledlayout(fig,1,1,'Padding','compact');
    ax = nexttile(layout); styleAxes(ax);
    scenario = e.scenario;
    frame = scenario.framePoseWorld;
    poses = parkingLocalToWorld([scenario.rearCarPose,scenario.frontCarPose, ...
        scenario.initialPose,scenario.targetPose],frame);
    rear = drawVehicle(poses(:,1),vehicle,0,ax);
    set(rear,'DisplayName','Parked vehicles','FaceAlpha',0.6);
    front = drawVehicle(poses(:,2),vehicle,0,ax);
    set(front,'HandleVisibility','off','FaceAlpha',0.6);
    start = drawVehicle(poses(:,3),vehicle,e.trajectory.segments(1).delta(1),ax);
    set(start,'FaceColor',[0.1,0.6,0.35],'FaceAlpha',0.12, ...
        'EdgeColor',[0.05,0.4,0.2],'DisplayName','Ego start pose');
    target = drawVehicle(poses(:,4),vehicle,e.trajectory.segments(end).delta(end),ax);
    set(target,'FaceColor','none','EdgeColor',[0.5,0.2,0.65], ...
        'LineStyle','--','DisplayName','Target pose');
    reference = parkingLocalToWorld(e.trajectory.q,frame);
    simulated = parkingLocalToWorld(e.simulation.q,frame);
    planned = plot(ax,reference(1,:),reference(2,:),'-','Color',[0,0.35,0.7], ...
        'LineWidth',2.4,'DisplayName','Planned rear-axle path','Tag','plannedXY');
    propagated = plot(ax,simulated(1,:),simulated(2,:),'--','Color',[0.85,0.3,0.05], ...
        'LineWidth',1.7,'DisplayName','Forward-simulated (Euler)','Tag','simulatedXY');
    plot(ax,poses(1,3:4),poses(2,3:4),'o','Color',[0.2,0.2,0.2], ...
        'MarkerFaceColor','white','MarkerSize',5,'HandleVisibility','off');
    legendHandles = [planned,propagated,rear,start,target];
    for k = 2:numel(e.plan.segments)
        join = parkingLocalToWorld(e.plan.segments(k).startPose,frame);
        marker = plot(ax,join(1),join(2),'d','Color',[0.1,0.1,0.1], ...
            'MarkerFaceColor','white','MarkerSize',7,'DisplayName','Planned stopped join');
        if k == 2
            legendHandles(end+1) = marker; %#ok<AGROW>
        else
            set(marker,'HandleVisibility','off');
        end
    end
    axis(ax,'equal'); axis(ax,'tight');
    limits = axis(ax);
    padding = 0.06*max(limits(2)-limits(1),limits(4)-limits(3));
    xlim(ax,limits(1:2)+[-padding,padding]);
    ylim(ax,limits(3:4)+[-padding,padding]);
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
    title(ax,sprintf('%s | %.1f m gap | quality gate: %s', ...
        scenario.name,scenario.spaceLength,upper(e.evaluation.status)),'FontSize',13);
    subtitle(ax,'Rear-axle trajectories; vehicle outlines show start, target and parked poses','FontSize',10);
    legend(ax,legendHandles,'Location','southoutside','NumColumns',3,'FontSize',10);
end

function drawErrors(fig,e)
    layout = tiledlayout(fig,1,1,'Padding','compact');
    ax = nexttile(layout); styleAxes(ax);
    errors = e.evaluation.errors;
    plot(ax,errors.t,errors.xyErrorMagnitude,'Color',[0,0.35,0.7], ...
        'LineWidth',2,'DisplayName','XY error magnitude','Tag','xyError');
    plot(ax,errors.t,errors.xError,'--','Color',[0.85,0.3,0.05], ...
        'LineWidth',1.4,'DisplayName','x error (local)');
    plot(ax,errors.t,errors.yError,':','Color',[0.1,0.55,0.4], ...
        'LineWidth',1.8,'DisplayName','y error (local)');
    yline(ax,e.evaluation.settings.maxTimeMatchedXYError,'--', ...
        'Color',[0.65,0.1,0.15],'LineWidth',1.3, ...
        'DisplayName',sprintf('Maximum XY error limit (%.2f m)', ...
        e.evaluation.settings.maxTimeMatchedXYError),'Tag','xyErrorLimit');
    xlim(ax,errors.t([1,end]));
    ylim(ax,[min([errors.xError,errors.yError,0])-0.005, ...
        max([errors.xyErrorMagnitude,e.evaluation.settings.maxTimeMatchedXYError])+0.008]);
    xlabel(ax,'Elapsed time (s)'); ylabel(ax,'Position error (m)');
    title(ax,[e.scenario.name,' | time-matched tracking error'],'FontSize',13);
    subtitle(ax,sprintf('Time-weighted XY RMSE: %.5f m  |  POC limit: %.2f m', ...
        e.evaluation.metrics.timeWeightedXYRMSE,e.evaluation.settings.timeWeightedXYRMSE),'FontSize',11);
    legend(ax,'Location','southoutside','NumColumns',2,'FontSize',10);
end

function drawInputs(fig,e,vehicle)
    layout = tiledlayout(fig,2,1,'Padding','compact','TileSpacing','compact');
    steering = nexttile(layout); styleAxes(steering);
    speed = nexttile(layout); styleAxes(speed);
    color = [0,0.35,0.7];
    for k = 1:numel(e.trajectory.segments)
        part = e.trajectory.segments(k);
        stairs(steering,part.globalT,rad2deg(part.delta),'Color',color, ...
            'LineWidth',1.5,'HandleVisibility','off','Tag','segmentSteering');
        stairs(speed,part.globalT,part.v,'Color',color, ...
            'LineWidth',1.5,'HandleVisibility','off','Tag','segmentSpeed');
        if k < numel(e.trajectory.segments)
            outgoing = e.trajectory.segments(k+1);
            plot(steering,part.globalT(end),rad2deg(part.delta(end)),'o', ...
                'Color',color,'MarkerFaceColor','white','MarkerSize',6, ...
                'HandleVisibility','off','Tag','incomingSteering');
            plot(steering,outgoing.globalT(1),rad2deg(outgoing.delta(1)),'o', ...
                'Color',color,'MarkerFaceColor',color,'MarkerSize',6, ...
                'HandleVisibility','off','Tag','outgoingSteering');
        end
    end
    limit = rad2deg(vehicle.maxSteering);
    yline(steering,limit,'--','Color',[0.65,0.1,0.15],'LineWidth',1.3,'Tag','positiveSteeringLimit');
    yline(steering,-limit,'--','Color',[0.65,0.1,0.15],'LineWidth',1.3,'Tag','negativeSteeringLimit');
    yline(speed,0,':','Color',[0.4,0.4,0.4]);
    speedPadding = 0.06*max(1,max(abs(e.trajectory.v)));
    ylim(speed,[min([e.trajectory.v,0])-speedPadding, ...
        max([e.trajectory.v,0])+speedPadding]);
    ylim(steering,[-1,1]*max(limit,max(abs(rad2deg(e.trajectory.delta))))*1.18);
    ylabel(steering,'Steering (deg)'); ylabel(speed,'Signed speed (m/s)');
    xlabel(speed,'Elapsed time (s)');
    xlim(steering,e.trajectory.t([1,end])); xlim(speed,e.trajectory.t([1,end]));
    title(steering,sprintf('Steering commands | dashed limits: +/-%.0f deg',limit),'FontSize',12);
    subtitle(steering,sprintf('Continuous geometric maximum: %.4f deg | join: open = incoming, filled = outgoing', ...
        rad2deg(e.evaluation.metrics.maxSteering)),'FontSize',10);
    title(speed,'Stop-to-stop speed | negative = reverse','FontSize',12);
    title(layout,[e.scenario.name,' | applied inputs (Euler left hold)'],'FontName','Arial','FontSize',13,'Color',[0.15,0.15,0.15]);
    linkaxes([steering,speed],'x');
end

function finishFigureStyle(fig)
% Explicit export colors keep report figures independent of desktop theme.
    set(findall(fig,'Type','text'),'Color',[0.15,0.15,0.15]);
    set(findall(fig,'Type','legend'),'TextColor',[0.15,0.15,0.15], ...
        'Color','white','EdgeColor',[0.7,0.7,0.7]);
end
