local SCRIPT_NAME     = 'group motor control.lua'
local RUN_INTERVAL_MS	 = 200
local RC_OPTION   		= 300

local THROTTLE				=70
local THROTTLE_LEFT			=73
local THROTTLE_RIGHT		=74
local OFF					=135
local MOTOR_1				=33
local MOTOR_2				=34
local MOTOR_3				=35


local MAV_SEVERITY_INFO      = 6

-- wrapper for gcs:send_text()
local function gcs_msg(severity, txt)
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

-- ! setup/initialization logic
local rc_chan = rc:find_channel_for_option(RC_OPTION)
local last_sw_pos = nil

function update()
    local sw_pos = rc_chan:get_aux_switch_pos()  -- returns 0, 1, or 2
    if sw_pos == last_sw_pos then return update, RUN_INTERVAL_MS end

    if sw_pos == 0 then
		param:set('SERVO1_FUNCTION', OFF)
		param:set('SERVO2_FUNCTION', OFF)
		param:set('SERVO3_FUNCTION', OFF)

		param:set('SERVO13_FUNCTION', THROTTLE)
		param:set('SERVO14_FUNCTION', THROTTLE)
	)
		gcs_msg(MAV_SEVERITY_INFO, 'Ackermann  ')
		
	if sw_pos == 1 then
		param:set('SERVO1_FUNCTION', OFF)
		param:set('SERVO2_FUNCTION', OFF)
		param:set('SERVO3_FUNCTION', OFF)
		
		param:set('SERVO13_FUNCTION', THROTTLE_LEFT)
		param:set('SERVO14_FUNCTION', THROTTLE_RIGHT)
		
		gcs_msg(MAV_SEVERITY_INFO, 'skid-steering')
		
    else
		
		param:set('SERVO1_FUNCTION', MOTOR_1)
		param:set('SERVO2_FUNCTION', MOTOR_2)
		param:set('SERVO3_FUNCTION', MOTOR_3)
		
		param:set('SERVO13_FUNCTION', OFF)
		param:set('SERVO14_FUNCTION', OFF)
		gcs_msg(MAV_SEVERITY_INFO, 'Thrusters')
    end

    last_sw_pos = sw_pos

    return update, RUN_INTERVAL_MS
end

gcs_msg(MAV_SEVERITY_INFO, 'Initialized.')

return update()