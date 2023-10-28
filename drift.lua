local DRIFT_CMD           = 86
local DEFAULT_DRIFT_M     = 10 -- (m) allowable drift distance if not specified
local DRIFT_MAV_NAME      = 'DRIFT'
local MAV_SEVERITY_NOTICE = 5
local MAV_SEVERITY_INFO   = 6
local ROVER_MODE_AUTO     = 10
local RUN_INTERVAL_MS     = 10

local THROTTLE_OUT_CH     = 3
local STEERING_FN         = 26
local THROTTLE_FN         = 70
local STEERING_SCR        = 94
local THROTTLE_SCR        = 95

local THROTTLE_TBL = { [true] = THROTTLE_SCR,
                       [false] = THROTTLE_FN }

local last_arm_state = nil
local last_id = -1
local drift_start_loc
local script_time_msn_index
local allowable_drift_dist = DEFAULT_DRIFT_M

function do_script_time()
    -- exit script time on waypoint change, mode change, or disarm
    if not arming:is_armed() or
        vehicle:get_mode() ~= ROVER_MODE_AUTO or
        mission:get_current_nav_index() ~= script_time_msn_index then
        vehicle:nav_script_time_done(last_id)
        allowable_drift_dist = DEFAULT_DRIFT_M
        gcs:send_text(MAV_SEVERITY_NOTICE, 'Drift canceled!')
        return await_script_time, RUN_INTERVAL_MS
    end

    SRV_Channels:set_output_scaled(STEERING_SCR, 0)
    SRV_Channels:set_output_scaled(THROTTLE_SCR, 0)

    -- compute drift distance and send to GCS
    local current_loc = assert(ahrs:get_location(), 'AHRS location error!')
    local drift_dist = current_loc:get_distance(drift_start_loc)
    gcs:send_named_float(DRIFT_MAV_NAME, drift_dist)

    -- resume mission if drift distance exceeds allowable
    if drift_dist > allowable_drift_dist then
        vehicle:nav_script_time_done(last_id)
        allowable_drift_dist = DEFAULT_DRIFT_M
        gcs:send_text(MAV_SEVERITY_NOTICE, ('Drifted %.1fm. Resuming nav!'):format(drift_dist))
        return await_script_time, RUN_INTERVAL_MS
    end

    return do_script_time, RUN_INTERVAL_MS
end

function await_script_time()
    -- report -1 for drift distance if script time is inactive
    gcs:send_named_float(DRIFT_MAV_NAME, -1)

    -- always use scripted steering, even when disarmed
    local steering = SRV_Channels:get_output_scaled(STEERING_FN)
    SRV_Channels:set_output_scaled(STEERING_SCR, steering)

    -- use actual throttle out during disarmed state
    -- switch to scripted throttle when armed
    local is_armed = arming:is_armed()
    if is_armed ~= last_arm_state then
        param:set(('SERVO%d_FUNCTION'):format(THROTTLE_OUT_CH), THROTTLE_TBL[is_armed])
        last_arm_state = is_armed
    end

    if not is_armed then  -- early return
        return await_script_time, RUN_INTERVAL_MS
    end

    local throttle = SRV_Channels:get_output_scaled(THROTTLE_FN)
    SRV_Channels:set_output_scaled(THROTTLE_SCR, throttle)

    -- enter script time routine if DRIFT_CMD is received
    -- first argument is allowable drift distance in meters
    local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
    if id and cmd == DRIFT_CMD then
        drift_start_loc = assert(ahrs:get_location(), 'AHRS location error!')
        if arg1 and arg1 > 0 then allowable_drift_dist = arg1 end
        last_id = id
        script_time_msn_index = mission:get_current_nav_index()
        gcs:send_text(MAV_SEVERITY_NOTICE, ('Allowing %.1fm drift...'):format(allowable_drift_dist))
        return do_script_time, RUN_INTERVAL_MS
    end

    return await_script_time, RUN_INTERVAL_MS
end

gcs:send_text(MAV_SEVERITY_INFO, 'Allow drift script loaded')

return await_script_time()
