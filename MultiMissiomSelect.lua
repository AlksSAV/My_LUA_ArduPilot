--[[----------------------------------------------------------------------------

ThreeMissions.lua - ArduPilot Lua script

How to use:
- Store the three Missions in the same subdir where the lua-script has to be placed
- select the correct SUB_DIR ('/APM/scripts/' for real flight)
- select the OPTION_SWITCH you decided to use (300..307) and put the same in the Parameter RCx_OPTION of the RC-Channel of your choice

CAUTION: Use this script AT YOUR OWN RISK.
------------------------------------------------------------------------------]]

local SCRIPT_NAME = 'ThreeMissions'

-------- USER 'CONSTANTS' - change here, if necessary --------
local SUB_DIR               = '/APM/scripts/' -- that's where to store the Missions - for SD-Card
local OPTION_SWITCH         = 300          -- the SCR-switch you selected (300..307)

-------- SCRIPT/ARDUPILOT 'CONSTANTS' --------
local REPEATER_MS         = 500

local WARN                =   4 --MAV_SEVERITY_WARNING
local INFO                =   6 --MAV_SEVERITY_INFO

local MODE_AUTO           =  10
local REASON_RC           =   1
local REASON_GCS          =   2
local REASON_INITIALIZED  =  26
local MISSION_COMPLETE    =   2

-------- SCRIPT VARIABLES --------
local scripting_rc       -- RC-input-channel for OPTION_SWITCH
local last_pos     = nil -- last position of RC-switch


local function send_msg(msg_type, msg)
  gcs:send_text(msg_type, string.format('%s: %s', SCRIPT_NAME, msg))
end

local function read_mission(file_name)

  -- Open file
  local file = io.open(file_name)

  -- check header
  assert(string.find(assert(file:read('l'), string.format('%s: %s not existing or empty', SCRIPT_NAME, file_name)), 'QGC WPL 110') == 1, string.format('%s: %s incorrect format', SCRIPT_NAME, file_name))

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

local function change_mission_allowed() -- test if loading a new Mission is allowed

local mode = vehicle:get_mode()
local mode_reason = vehicle:get_control_mode_reason()

  if ((mode ~= MODE_AUTO) 
   and
   ((mode_reason == REASON_RC) or (mode_reason == REASON_GCS) or (mode_reason == REASON_INITIALIZED) or (not arming:is_armed())))
   or
   (mission:state() == MISSION_COMPLETE)
  then
    return true
  else
    send_msg(WARN, string.format('Changing Mission not allowed (mode: %d reason: %d)',mode, mode_reason))
    return false
  end
end

local function switch_has_changed(new_sw_pos) -- action when switch has changed
  if change_mission_allowed() then
      local file_name = string.format('%s/Mission#%d.waypoints', SUB_DIR, new_sw_pos)
      send_msg(INFO, string.format('try to read %s ...', file_name))
      read_mission(file_name)
      last_pos = new_sw_pos
  end
end

function update_switch()
-- read switch input (0, 1, 2) from designated scripting RCx_OPTION and check if it has changed
  local sw_pos = scripting_rc:get_aux_switch_pos()
  if sw_pos ~= last_pos then
    switch_has_changed(sw_pos)
  end
  return update_switch, REPEATER_MS -- reschedules the loop
end

function init()
  scripting_rc = rc:find_channel_for_option(OPTION_SWITCH)
  if scripting_rc then
    return update_switch()
  else
    send_msg(WARN, string.format('RCx_OPTION = %d not found', OPTION_SWITCH))
    return init, 10000
  end
end

return init()
