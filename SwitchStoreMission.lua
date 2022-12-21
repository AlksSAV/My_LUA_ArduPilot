--[[
SwitchStoreMission.lua - ArduPilot Lua script


]]
-------- SCRIPT 'CONSTANTS' --------
local SCRIPT_NAME           = 'SSM'          -- abbreviation of scriptname for messages on GCS
local SUB_DIR_APM           = '/APM/scripts'-- path to scripts on SD-card
local SUB_DIR_SITL          = './scripts'   -- path to scripts when using SITL
local SUB_DIR_MISSIONS      = 'missions'    -- subdirectory below scripts-directory where the mission-files are stored
local FILE_PREFIX           = 'Mission#'    -- filename-prefix of all mission files
local FILE_EXTENSION        = 'waypoints'   -- file-extension of all mission files
local LONG_PRESS_MS         = 1000          -- time to long press pushbutton to start loading the mission
local SHORT_INTERV_MS       = 20            -- interval to find param_table_key
local RUN_INTERVAL_MS       = 100           -- interval to check input-changes
local SBY_INTERVAL_MS       = 1000          -- interval if script is inactive i.e. num_sw_positions is lower than 1
local INTERRUPTED_MS        = 10000         -- interval if script is paused/interrupted by a problem

local WARN                  =   4           -- MAV_SEVERITY_WARNING
local INFO                  =   6           -- MAV_SEVERITY_INFO

local MODE_AUTO             =  10
local REASON_RC             =   1
local REASON_GCS            =   2
local REASON_INITIALIZED    =  26

-------- PARAMETER TABLE --------
local PARAM_PREFIX          = 'SSM_'
local PARAM_TABLE           = {
--  {       name , default value },
    { 'RC_SELECT',           301 },         -- defines the selected RCx_Option (300..307)
    { 'POSITIONS',             1 },         -- defines the selection-method and/or amount of switch-positions
    { 'RC_STORE' ,           300 },         -- defines the selected RCx_Option (300..307) of the store-input-channel
}
local param_table_key       = 0             -- start-value for searching free or still used table-key between 0 and 200

-------- SCRIPT VARIABLES --------
local sub_dir                               -- where to store the Missions
local file_name                             -- filename of the current mission
local num_files                             -- number of detected mission-files
local file_index            = 0

local num_select_positions                  -- number of select-switch positions
local select_rc                             -- RC-input-channel of selectet RCx_OPTION
local last_select_position  = nil           -- last position of RC-switch
local last_select_change    = uint32_t(0)
local wait_finger_off       = false

local store_rc                              -- RC-input-channel of selectet RCx_OPTION
local last_store_position   = nil           -- last position of RC-switch
local store_value           = 0             -- value of store-parameter
local basepoint Location()                  -- location of vehicle when Store-Button was pushed

local function send_msg(msg_type, msg)
  -- wrapper for sending messages to the GCS
  gcs:send_text(msg_type, string.format('%s: %s', SCRIPT_NAME, msg))
end
  
local function add_params(key, prefix, tbl)
-- add script-specific parameter table
  if not param:add_table(key, prefix, #tbl) then
    send_msg(INFO, string.format('Could not add table %s at key %d', prefix, key))
    return false
  end
  for num, data in ipairs(tbl) do
      assert(param:add_param(key, num, data[1], data[2]), string.format('%s: Could not add %s%s.', SCRIPT_NAME, prefix, data[1]))
  end
  return true
end


local function read_mission(file_path)
-- try to read the mission from file and write it into FC
  local file = io.open(file_path)

  -- check file-header
  local headline = file:read('l')
  if headline == nil then
    send_msg(WARN,string.format('%s not existing or empty', file_name))
    return
  end
  if string.find(headline, 'QGC WPL 110') ~= 1 then
    send_msg(WARN,string.format('%s incorrect format', file_name))
    return
  end

  -- clear any existing mission
  assert(mission:clear(), string.format('%s: Could not clear current mission', SCRIPT_NAME))

  -- read each line of file and write item to FC
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          -- reached correct end of file
          send_msg(INFO, string.format('%s (%d items) loaded', file_name, index-1))
          return
        else
          -- in case of problems
          mission:clear() -- clear partly loaded mission
          error(string.format('%s: failed to read file', SCRIPT_NAME))
        end
      end
    end

    -- fill the item-structure
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

    -- write the mission-item to FC
    if not mission:set_item(index,item) then
      mission:clear() -- clear partly loaded mission
      error(string.format('%s failed to set mission item %i', SCRIPT_NAME, index))
    end
    index = index + 1
  end
end

local function store_mission(write_path, write_file, location_point)
  -- write a mission to file
  
  -- check if there is a mission to save
  local num_wp = mission:num_commands()
  if num_wp < 1 then
    send_msg(WARN,'no mission on FC')
    return false
  end

  -- create new file
  file = assert(io.open(string.format('%s/%s', write_path, write_file), 'w+'), string.format('%s: Could not create %s', SCRIPT_NAME, write_file))

  -- header
  file:write('QGC WPL 110\n')

  -- read home and write to file
  local item = mission:get_item(0)
  file:write(string.format('%i\t1\t%i\t%i\t%0.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n',item:seq(),item:frame(),item:command(),item:param1(),item:param2(),item:param3(),item:param4(),item:x()*10^-7,item:y()*10^-7,item:z()))

  -- fill item-structure with loiter unlimited at location_point and write to file
  file:write(string.format('%i\t0\t%i\t%i\t%0.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n',
    1, -- sequence/line-counter
    0, -- frame (abs)
    17, -- command (LOITER_UNLIM)  // 16 - waypoint, 20 - return to launch 
    0, 0, 0, 0, -- params
    location_point:lat()*10^-7, -- lattitude
    location_point:lng()*10^-7, -- longitude
    location_point:alt()/100))  -- altitude

  file:close()
  send_msg(INFO,string.format('%s stored', write_file))
  return true
end
  
local function change_mission_allowed()
-- check, if loading a new Mission is allowed
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
    send_msg(WARN, string.format('Mission change not allowed (mode:%d reason:%d)',mode, mode_reason))
    return false
  end
end


-- ### selection and loading of mission by momentary-pushbutton ###
local function do_momentary_select()
  -- check if pushbutton changed and react
  local previous_switch_position = last_select_position
  local switch_position          = select_rc:get_aux_switch_pos()
  last_select_position = switch_position
  -- unpressed and no change
  if (switch_position == 0) and (previous_switch_position == 0) then 
    wait_finger_off = false
    return
  end
  if wait_finger_off then return end
  -- just pressed
  if (switch_position >= 1) and (previous_switch_position == 0) then
    last_select_change = millis()
    return
  end
  -- check if long-pressed
  local long_press = (millis() - last_select_change) > LONG_PRESS_MS
  if long_press then
    if change_mission_allowed() then
      file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, file_index, FILE_EXTENSION)
      read_mission(string.format('%s/%s', sub_dir, file_name))
      wait_finger_off = true
    end
    return
  -- just released
  elseif (switch_position == 0) and (previous_switch_position >= 1) then
    file_index = (file_index + 1) % num_files
    send_msg(INFO, string.format('Selected %s%d', FILE_PREFIX, file_index))
  end
  return
end


-- ### selection and loading of mission by multi-position-switch ###
local function switch_has_changed(new_select_pos)
-- action if switch has changed
  if change_mission_allowed() then
    file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, new_select_pos, FILE_EXTENSION)
    read_mission(string.format('%s/%s', sub_dir, file_name))
    last_select_position = new_select_pos
  end
end


local function do_multi_select()
-- check, if select-switch position has changed
  local select_pos = (select_rc:norm_input_ignore_trim() + 1) * (num_select_positions - 1) / 2
  select_pos = tonumber(string.format('%.0f', select_pos)) or -1

  if select_pos ~= last_select_position then
    switch_has_changed(select_pos)
  end
  return
end

local function count_missions()
  -- count available missions
  file_count = 0
  while true do
    file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, file_count, FILE_EXTENSION)
    file = io.open(string.format('%s/%s', sub_dir, file_name))
    if file:read('l') ~= nil then
      file_count = file_count + 1
      file:close()
    else
      send_msg(INFO, string.format('%d missions found', file_count))
      break
    end
  end
  return file_count
end


local function get_store()  
-- check, if store-button position has changed
  local store_pos = store_rc:get_aux_switch_pos()
  if store_pos ~= last_store_position then
    last_store_position = store_pos
    if store_pos == 2 then
      -- get location
      basepoint = assert(ahrs:get_location(), string.format('%s: no location available', SCRIPT_NAME)) -- memorize current location
      -- search first not existing mission-number
      num_files = count_missions()
      -- store location as new mission
      local store_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, num_files, FILE_EXTENSION)
      if not store_mission(sub_dir, store_name, basepoint) then
        send_msg(WARN, string.format('not able to write %s', store_name))
      else
        num_files = num_files + 1
      end
    end
  end
end

local function scan_inputs()
  -- check inputs
  if (num_select_positions < 1) and (store_value < 300) then
    return scan_inputs, SBY_INTERVAL_MS
  end
  if store_value >= 300 then get_store() end
  if num_select_positions > 1 then
    do_multi_select()
  end
  if num_select_positions == 1 then
    do_momentary_select()
  end
  return scan_inputs, RUN_INTERVAL_MS
end
  

-- ### initialisation of all necessary environment ###
function init()

  -- add script-specific parameter-table
  if not add_params(param_table_key, PARAM_PREFIX, PARAM_TABLE) then
    if param_table_key < 200 then
      param_table_key = param_table_key + 1
      return init, SHORT_INTERV_MS
    else
      send_msg(WARN, string.format('Could not add table %s', PARAM_PREFIX))
      return init, INTERRUPTED_MS
    end
  else
    send_msg(INFO, string.format('Param table %s at key %d', PARAM_PREFIX, param_table_key))
  end

  -- reading  parameter: number of switch positions
  num_select_positions = param:get(PARAM_PREFIX .. 'POSITIONS')
  if num_select_positions then
    if num_select_positions == 1 then
      send_msg(INFO, string.format('pushbutton-multiselect'))
    elseif num_select_positions > 1 then
      send_msg(INFO, string.format('multiposition-switch: %i positions',num_select_positions))
    end
  else
    send_msg(WARN, string.format('get parameter %s%s failed', PARAM_PREFIX, 'POSITIONS'))
    return init, INTERRUPTED_MS
  end

  -- reading the selected option in RC_SELECT parameter and finding corresponding rc-channel for the switch
  local select_value = param:get(PARAM_PREFIX .. 'RC_SELECT')
  if select_value then
    if (select_value >= 300) and (select_value <= 307) then
      send_msg(INFO, string.format('selected RC_Option: %i',select_value))
    else
      send_msg(WARN, string.format('selected RC_Option: %i out of range (300..307)',select_value))
      return init, INTERRUPTED_MS
    end
  else
    send_msg(WARN, string.format('get parameter %s%s failed',PARAM_PREFIX, 'RC_SELECT'))
    return init, INTERRUPTED_MS
  end

  -- reading the selected option in RC_STORE parameter and finding corresponding rc-channel for the switch
  store_value = param:get(PARAM_PREFIX .. 'RC_STORE')
  if store_value then
    if (store_value >= 300) and (store_value <= 307) then
      send_msg(INFO, string.format('selected RC_Option: %i',store_value))
    else
      send_msg(WARN, string.format('selected RC_Option: %i out of range (300..307)',store_value))
      return init, INTERRUPTED_MS
    end
  else
    send_msg(WARN, string.format('get parameter %s%s failed',PARAM_PREFIX, 'RC_STORE'))
    return init, INTERRUPTED_MS
  end

  -- finding correct sub_dir for SITL or SD-card
  sub_dir = string.format('%s/%s', SUB_DIR_APM, SUB_DIR_MISSIONS)
  file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, 0, FILE_EXTENSION)
  local file = io.open(string.format('%s/%s', sub_dir, file_name))
  if file:read('l') ~= nil then
    file:close()
  else
    sub_dir = string.format('%s/%s', SUB_DIR_SITL, SUB_DIR_MISSIONS)
    file = io.open(string.format('%s/%s', sub_dir, file_name))
    if file:read('l') ~= nil then
      file:close()
    else
      send_msg(WARN, string.format('No file %s found ...', file_name))
      send_msg(WARN, string.format('...in SubDir /%s', SUB_DIR_MISSIONS))
      return init, INTERRUPTED_MS
    end
  end
  send_msg(INFO, string.format('sub_dir: %s', sub_dir))

  num_files = count_missions()

  file_name = string.format('%s_%s%d.%s', SCRIPT_NAME, FILE_PREFIX, 0, FILE_EXTENSION)

  if (num_select_positions > 1) and (num_files < num_select_positions) then
    send_msg(WARN, string.format('only %d files found', num_files))
    return init, INTERRUPTED_MS
  end  

  -- find channel for mission-storing
  store_rc = rc:find_channel_for_option(store_value)
  if not store_rc then
    send_msg(WARN, string.format('RCx_OPTION = %d not found', store_value))
    return init, INTERRUPTED_MS
  end

  -- find channel for mission-switching
  select_rc = rc:find_channel_for_option(select_value)
  if select_rc then
    send_msg(INFO, 'Script is running')
    return scan_inputs, SBY_INTERVAL_MS -- init done -> regular loop start
  else
    send_msg(WARN, string.format('RCx_OPTION = %d not found', select_value))
    return init, INTERRUPTED_MS
  end
end

return init()
