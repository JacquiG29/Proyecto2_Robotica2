function [speed, state] = braitenberg(state, wheel_weight_total, speed, WHEEL_WEIGHT_THRESHOLD, MAX_SPEED)

switch state
    case "f"
        if wheel_weight_total(1) > WHEEL_WEIGHT_THRESHOLD
            speed(1) = 0.7 * MAX_SPEED;
            speed(2) = -0.7 * MAX_SPEED;
            state = "l";
        elseif wheel_weight_total(2) > WHEEL_WEIGHT_THRESHOLD
            speed(1) = -0.7 * MAX_SPEED;
            speed(2) = 0.7 * MAX_SPEED;
            state = "r";
        else
            speed(1) = MAX_SPEED;
            speed(2) = MAX_SPEED;
        end
    case "l"
        if wheel_weight_total(1) > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total(2) > WHEEL_WEIGHT_THRESHOLD
            speed(1) = 0.7 * MAX_SPEED;
            speed(2) = -0.7 * MAX_SPEED;
        else
            speed(1) = MAX_SPEED;
            speed(2) = MAX_SPEED;
            state = "f";
        end
    otherwise
        if wheel_weight_total(1) > WHEEL_WEIGHT_THRESHOLD || wheel_weight_total(2) > WHEEL_WEIGHT_THRESHOLD
            speed(1) = -0.7 * MAX_SPEED;
            speed(2) = 0.7 * MAX_SPEED;
        else
            speed(1) = MAX_SPEED;
            speed(2) = MAX_SPEED;
            state = "f";
        end
end

end