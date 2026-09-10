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