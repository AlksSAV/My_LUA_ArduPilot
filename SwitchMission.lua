--[[----------------------------------------------------------------------------

SwitchMission.lua - ArduPilot Lua script

HOW IT WORKS:
- get RC input as selected and check, if loading a mission is requested
- if loading is requested, check if change_mission_allowed:
       - the current Mode is non-AUTO and the ModeReason is RC, GCS or INITIALIZED
    or - the current Mode is non-AUTO and the vehicle isn't armed
    or - the Mission is completed
- if allowed, load corresponding Mission:
  Mission#0.waypoints or
  Mission#1.waypoints or
  etc.

HOW TO USE:
- Store the Missions in the subdir /missions where the lua-script has to be placed (e.g. on SD-Card in '/APM/scripts/missions')
- put the Scripting-Option-Switch you decided to use (300..307) into the SCR_USER_PARAM_OPTION (standard: SCR_USER1) parameter
- put the selected Scripting-Option-Switch (300..307) into the Parameter RCx_OPTION of the RC-Channel of your choice
- put info about the selection-method you want to use for the missions into the SCR_USER_PARAM_NUMBER (standard: SCR_USER2) parameter
  - if <1 : no action (disable switching of mission)
  - if  1 : the selection is done by a pushbutton: short-push will go through the missions, long-push will load the mission
  - if >1 : the selection is done by a multi-position-switch with the amount of positions are given here
    - the whole way of the corresponding RC-Channel is devided in regions e.g. for 6 missions: 
                 1/10    1/5    1/5    1/5    1/5   1/10
       i.e. -100%...-80%...-40%.....0%....40%....80%...100%
- restart the script (e.g. by restarting the FC)

CAUTION: Use this script AT YOUR OWN RISK.

------------------------------------------------------------------------------]]

-------- USER 'CONSTANTS' - change here, if necessary --------
local SCR_USER_PARAM_OPTION      = 'SCR_USER1'  -- the SCR_USERx parameter that defines the selected RCx_Option (300..307)
local SCR_USER_PARAM_NUMBER      = 'SCR_USER2'  -- the SCR_USERx parameter that defines the number of missions you will select by the switch

-------- SCRIPT/ARDUPILOT 'CONSTANTS' --------
local SCRIPT_NAME = 'SwitchMission'

local REPEATER_MS         = 500

local WARN                =   4 -- MAV_SEVERITY_WARNING
local INFO                =   6 -- MAV_SEVERITY_INFO

local MODE_AUTO           =  10
local REASON_RC           =   1
local REASON_GCS          =   2
local REASON_INITIALIZED  =  26

-------- SCRIPT VARIABLES --------
local sub_dir            -- that's where to store the Missions - for SITL: './scripts/missions' - for SD-Card: '/APM/scripts/missions'
local number_of_missions -- number of missions to select
local scripting_rc       -- RC-input-channel for OPTION_SWITCH
local last_pos     = nil -- last position of RC-switch


local function send_msg(msg_type, msg)
-- wrapper for sending messages to the GCS
  gcs:send_text(msg_type, string.format('%s: %s', SCRIPT_NAME, msg))
end

local function read_mission(file_name)
-- try to read the mission into the FC
  local file = io.open(file_name)

  -- check header
  local headline = assert(file:read('l'), string.format('%s: %s not existing or empty', SCRIPT_NAME, file_name))
  assert(string.find(headline, 'QGC WPL 110') == 1, string.format('%s: %s incorrect format', SCRIPT_NAME, file_name))

  -- clear any existing mission
  assert(mission:clear(), string.format('%s: Could not clear current mission', SCRIPT_NAME))

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          send_msg(INFO, string.format('Mission %s with %d items loaded', file_name, index-1))
          return -- got to the end of the file
        else
          mission:clear() -- clear part loaded mission
          error(string.format('%s: failed to read file', SCRIPT_NAME))
        end
      end
    end

    item:seq(data[1])
    item:frame(data[3])
    item:command(data[4])
    item:param1(data[5])
    item:param2(data[6])
    item:param3(data[7])
    item:param4(data[8])
    item:x(data[9]*10^7)
    item:y(data[10]*10^7)
    item:z(data[11])

    if not mission:set_item(index,item) then
      mission:clear() -- clear partly loaded mission
      error(string.format('%s failed to set mission item %i', SCRIPT_NAME, index))
    end
    index = index + 1
  end

end

local function change_mission_allowed()
-- test if loading a new Mission is allowed
  local mode = vehicle:get_mode()
  local mode_reason = vehicle:get_control_mode_reason()

  if ((mode ~= MODE_AUTO)
   and
   ((mode_reason == REASON_RC) or (mode_reason == REASON_GCS) or (mode_reason == REASON_INITIALIZED) or (not arming:is_armed())))
   or
   (mission:state() == mission.MISSION_COMPLETE)
  then
    return true
  else
    send_msg(WARN, string.format('Changing Mission not allowed (mode: %d reason: %d)',mode, mode_reason))
    return false
  end
end

local function switch_has_changed(new_sw_pos)
-- action when switch has changed
  if change_mission_allowed() then
      local file_name = string.format('%s/Mission#%d.waypoints', sub_dir, new_sw_pos)
      send_msg(INFO, string.format('try to read %s ...', file_name))
      read_mission(file_name)
      last_pos = new_sw_pos
  end
end

function update_switch()
-- get switch position
  local percentage = (scripting_rc:norm_input() * 100 + 100) / 2
  local percentage_step = 100.0 / (number_of_missions - 1)  
  local percentage_compare = -0.5 * percentage_step
  local sw_pos = -1

  repeat
    percentage_compare = percentage_compare + percentage_step
    sw_pos = sw_pos + 1
  until percentage < percentage_compare

  -- check if sw_pos has changed
  if sw_pos ~= last_pos then
    switch_has_changed(sw_pos)
  end
  return update_switch, REPEATER_MS -- reschedules the loop
end

function init()
-- finding correct sub_dir for SITL or SD-card
  sub_dir = '/APM/scripts/missions'
  local file_name = string.format('%s/Mission#%d.waypoints', sub_dir, 0)
  local file = io.open(file_name)
  if file:read('l') ~= nil then
    file:close()
  else
    send_msg(INFO, string.format('No file %s found', file_name))
    sub_dir = './scripts/missions'
    local file_name = string.format('%s/Mission#%d.waypoints', sub_dir, 0)
    send_msg(INFO, string.format('Try to find %s', file_name))
    local file = io.open(file_name)
    if file:read('l') ~= nil then
      file:close()
    else
      send_msg(WARN, string.format('No file %s found', file_name))
      return init, 10000
    end
  end

  -- reading number of switch positions (missions) in SCR_USERx parameter
  number_of_missions = param:get(SCR_USER_PARAM_NUMBER)
  if number_of_missions then
    if number_of_missions >= 2 then
      send_msg(INFO, string.format('selected number of missions: %i',number_of_missions))
    else
      send_msg(WARN, string.format('selected number of missions: %i out of range (<2)',number_of_missions))
      return init, 10000
    end
  else
    send_msg(WARN, string.format('get parameter %s failed', SCR_USER_PARAM_NUMBER))
    return init, 10000
  end

  -- finding the selected option in SCR_USERx parameter and corresponding rc-channel for the switch
  local option_value = param:get(SCR_USER_PARAM_OPTION)
  if option_value then
    if (option_value >= 300) and (option_value <= 307) then
      send_msg(INFO, string.format('selected RC_Option: %i',option_value))
    else
      send_msg(WARN, string.format('selected RC_Option: %i out of range (300..307)',option_value))
      return init, 10000
    end
  else
    send_msg(WARN, string.format('get parameter %s failed', SCR_USER_PARAM_OPTION))
    return init, 10000
  end


  scripting_rc = rc:find_channel_for_option(option_value)
  if scripting_rc then
    return update_switch()
  else
    send_msg(WARN, string.format('RCx_OPTION = %d not found', option_value))
    return init, 10000
  end
end

return init()
