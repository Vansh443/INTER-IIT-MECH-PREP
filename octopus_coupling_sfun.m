function octopus_coupling_sfun(block)
    setup(block);
end

function setup(block)
    % 2 inputs: x1 (main curl), x2 (extra tip curl)
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;   % one vector output with 5 joint angles

    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).Dimensions = 1;
    block.InputPort(2).DirectFeedthrough = true;

    block.OutputPort(1).Dimensions = 5;

    block.NumDialogPrms = 0;
    block.SampleTimes = [0 0];

    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
end

function InitConditions(block)
end

function Outputs(block)
    x1 = block.InputPort(1).Data;   % actuator 1: main curl, joints 1-5
    x2 = block.InputPort(2).Data;   % actuator 2: extra curl, joints 3-5 only

    r = 0.01;         % pulley radius (m)
    theta_max = 0.35;  % max joint angle (rad) - tune to your CAD clearance

    theta = zeros(5,1);

    % --- Actuator 1: sequential base-to-tip closing across all 5 joints ---
    n_free = 5;
    remaining = x1;
    for i = 1:5
        share = remaining / (r * n_free);
        if share >= theta_max
            theta(i) = theta_max;
            remaining = remaining - theta_max * r;
            n_free = n_free - 1;
        else
            theta(i) = share;
        end
    end

    % --- Actuator 2: extra curl added only to joints 3, 4, 5 ---
    extra = x2 / r;
    for i = 3:5
        theta(i) = min(theta(i) + extra, theta_max);
    end

    block.OutputPort(1).Data = theta;
end