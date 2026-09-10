def PID_controller(current_yaw, target_yaw, throttle):
    global pre_error, sum_error
    yaw_difference = target_yaw - current_yaw

    if abs(yaw_difference) > 180:
        if yaw_difference > 0:
            error = yaw_difference - 360
        else:
            error = yaw_difference + 360
    else:
        error = yaw_difference

    sum_error += error * 0.1
    d_error = error - pre_error
    Kp = 0.2 * math.sqrt(throttle) - 0.05
    Ki = 0
    Kd = 0 * math.sqrt(throttle)
    steer = Kp * error + Ki * sum_error + Kd * d_error

    steer = max(min(steer, 1), -1)

    pre_error = error

    return steer
