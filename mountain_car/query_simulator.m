% This function calls the high fidelity mountain car simulator to find the
% robust satisfaction value for a specific initial condition (x0, v0)
function rsvalue = query_simulator(x0, v0)
    rsv = @(arg) (arg - 0.6);
    delta = 10; % 10s to satisfy robust satisfaction value
    set_param('CM/CM System/Pos','InitialCondition',num2str(x0));
    set_param('CM/CM System/Vel','InitialCondition',num2str(v0));
    simOut = sim('CM','SaveTime','on','TimeSaveName','tout');
    
    x = simOut.logsOut.get('pos').Values.Data;
    v = simOut.logsOut.get('vel').Values.Data;
    t = simOut.tout;
    st = (simOut.tout(end) < delta);
    % Computing RSV on trace limited to the first 10 sec
    if (st)
        rho = max(rsv(x));
    else
        t_less = t <= delta;
        t_delta = t(t_less == 1);
        n = length(t_delta);
        x_delta = x(1:n);
        rho = max(rsv(x_delta));
    end
    rsvalue = rho;
end