--[[---------------------------------------------------------------------------
This script is useful to select an existing mission on SD-card via RC-channel, relocate (translate/rotate) it and switch to mode-AUTO

The selection can be done by multi-position-switch or pushbutton.
The relocation can be started by long-press of a pushbutton
The kind of relocation can be selected by a 3-pos-switch or directly via parameter

ATTENTION!!!  - At start of the script, the current Mission in FC is saved in SRM_Backup#x.waypoints in a cyclic manner
                The last Backup# you can find in SRM_Count.txt.

]]-----------------------------------------------------------------------------

-------- SCRIPT/ARDUPILOT 'CONSTANTS' --------
local SCRIPT_NAME           = 'SRM'           -- abbreviation of scriptname for messages to GCS
local SUB_DIR_APM           = '/APM/scripts'  -- path to scripts on SD-card
local SUB_DIR_SITL          = './scripts'     -- path to scripts on SITL
local SUB_DIR_MISSIONS      = './missions'    -- subdirectory below scripts-directory where all the files are stored
local MISSION_FILE_PREFIX   = 'Mission#'      -- filename-prefix of mission files
local BACKUP_FILE_PREFIX    = 'Backup#'       -- filename-prefix of backup files
local FILE_EXTENSION        = 'waypoints'     -- file-extension of all mission files
local BACKUP_FILE_MAX       = 5               -- amount of backup files (circle)
local COUNT_FILE            = 'Count.txt'     -- filename of count file for backups

local LONG_PRESS_MS         = 1000            -- time to long press pushbutton to start loading/relocating the mission
local SHORT_INTERV_MS       = 20              -- interval to find param_table_key
local RUN_INTERVAL_MS       = 100             -- interval to check input-changes
local SBY_INTERVAL_MS       = 30000           -- interval if script is inactive i.e. SRM_POSITIONS is lower than 1
local INTERRUPTED_MS        = 10000           -- interval if script is paused/interrupted by a problem

local WARN                  = 4               -- MAV_SEVERITY_WARNING
local INFO                  = 6               -- MAV_SEVERITY_INFO

local MODE_AUTO             = 10
local REASON_RC             = 1
local REASON_GCS            = 2
local REASON_INITIALIZED    = 26

local MAV_WAYPOINT          = 16              -- MAV_CMD_NAV_WAYPOINT
local MAV_SPLINE_WAYPOINT   = 82              -- MAV_CMD_NAV_SPLINE_WAYPOINT
local MAV_DO_LAND_START     = 189             -- MAV_CMD_DO_LAND_START

local NO_RELOCATION         = 0
local ONLY_TRANSLATE        = 1
local TRANSLATE_ROTATE      = 2 

-------- SCRIPT PARAMETERS --------
local PARAM_PREFIX          = 'SRM_'
local PARAM_TABLE           = {
--  {          name  ,          default },
    { 'POSITIONS'   ,                1 },    -- defines the selection-method and/or amount of switch-positions
    { 'RC_SELECT'   ,                0 },    -- the selected RCx_Option (300..307) for selecting the mission or 0 to use start-button RC_START
    { 'RC_START'    ,              300 },    -- the selected RCx_Option (300..307) for starting the relocation
    { 'RELOCATION'  , TRANSLATE_ROTATE },    -- the selected RCx_Option (300..307) for definition of the kind of relocation or 0=no relocation / 1=translation only / 2=translation+rotation
    { 'NO_RELOC_M'  ,               50 },    -- no relocation of Mission within a radius of XX m around Homepoint (recommended: Plane:50m, Copter+Heli:10..20m, others:5m)
}
local POSITIONS  = Parameter()               -- create Parameter() objects for frequently read paramters
local RELOCATION = Parameter()
local NO_RELOC_M = Parameter()

-------- SCRIPT VARIABLES --------
local sub_dir
local num_sw_positions                        -- number of select-switch positions
local num_files                               -- number of detected mission-files

local last_sw_change        = uint32_t(0)
local file_index            = 0
local wait_finger_off       = false

local scr_rc_select         = nil             -- RC-channel of multipos-switch for selection of mission (if any)
local scr_rc_start          = nil             -- RC-channel of start-button
local scr_rc_relocation     = nil             -- RC-channel of 3-pos-switch for selection of kind of relocation
local no_relocation_radius  = -1
local last_switch_position  = -1              -- last position of RC-switch
local last_button_position  = 0               -- last position of RC-pushbutton
local kind_of_relocation    = 0

local basepoint Location()                    -- location of the vehicle when the Start-Button was pushed
local rotation              = 0.0             -- heading of the vehicle when the Start-Button was pushed
local altitude_translation  = 0               -- in [cm]
local param_table_key       = 0               -- start-value for searching free or still used table-key between 0 and 200


local function send_msg(msg_type, msg)
-- helper to have the SCRIPT_NAME in front of each message to GCS
  gcs:send_text(msg_type, string.format('%s: %s', SCRIPT_NAME, msg))
end

local function longitude_scale(lat)
-- helper to correct the longitude distance according to lattitude movement
  local scale = math.cos(lat * (1.0e-7 * 3.141592653589793 / 180.0))
  return math.max(scale, 0.01)
end

local function add_params(key, prefix, tbl)
-- add script-specific parameter table
  if not param:add_table(key, prefix, #tbl) then
--    send_msg(INFO, string.format('key %d occupied', key))
    return false
  end
  for num, data in ipairs(tbl) do
      assert(param:add_param(key, num, data[1], data[2]), string.format('%s: Could not add %s%s.', SCRIPT_NAME, prefix, data[1]))
  end
  return true
end

      
local function read_mission(read_path, read_file, relocation)
-- try to read the mission from file and write it into the FC

  local first_wp          = Location() -- first waypoint of the mission
  local untranslated_loc  = Location()
  local current_wp        = Location()


  local file = io.open(string.format('%s/%s', read_path, read_file), "r")
  if file then
    local headline = file:read('l')
  -- check header
    if not headline then
      send_msg(WARN,string.format('%s is empty', read_file))
      return false
    end
    if string.find(headline, 'QGC WPL 110') ~= 1 then
      send_msg(WARN,string.format('%s incorrect format', read_file))
      return false
    end
  else
    send_msg(WARN,string.format('%s not existing', read_file))
    return false
  end

  -- clear any existing mission
  assert(mission:clear(), 'Could not clear current mission')

    -- read each line and write mission to FC
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if not data[i] then
        if i == 1 then
          -- reached correct end of file
          send_msg(INFO, string.format('%s (%d items) loaded', read_file, index-1))
          return true
        else
          mission:clear() -- clear partly loaded mission
          error(string.format('%s: failed to read file - mission cleared', SCRIPT_NAME))
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
    if (item:command() == MAV_DO_LAND_START) then
      relocation = NO_RELOCATION
    end

    if (item:seq() > 0) and (relocation > NO_RELOCATION) then  -- at least translation
      if (first_wp:lat() == 0) and (first_wp:lng() == 0) then
        if (item:seq() > 0) and 
           ((item:command() == MAV_WAYPOINT) or (item:command() == MAV_SPLINE_WAYPOINT)) and 
           (item:x() ~= 0) and (item:y() ~= 0) then -- look for first waypoint
          first_wp:lat(item:x())
          first_wp:lng(item:y())
          first_wp:alt(item:z()*100)
          if     item:frame() == 3 then
            first_wp:relative_alt(1)
          elseif item:frame() == 10 then
            first_wp:terrain_alt(1)
          end

          -- calculate altitude-translation
          assert(basepoint:change_alt_frame(first_wp:get_alt_frame()), string.format('%s: alt_frame not comparable', SCRIPT_NAME))
          altitude_translation = basepoint:alt() - first_wp:alt()
          if altitude_translation < 0 then -- allow just positive offsets for altitude
            altitude_translation = 0
          end
          send_msg(INFO, string.format('altitude translation: %+.1f m', altitude_translation/100))
        end
      end

      if ((item:command() == MAV_WAYPOINT) or (item:command() == MAV_SPLINE_WAYPOINT)) then -- translate waypoints
        untranslated_loc:lat(item:x())
        item:x(basepoint:lat() + (item:x() - first_wp:lat()))
        current_wp:lat(item:x())
        item:y(basepoint:lng() + (item:y() - first_wp:lng() ) / longitude_scale(untranslated_loc:lat()) * longitude_scale(current_wp:lat()) )
        current_wp:lng(item:y())
        item:z(item:z()+altitude_translation/100)
      end
      if (relocation == TRANSLATE_ROTATE) and ((item:command() == MAV_WAYPOINT) or (item:command() == MAV_SPLINE_WAYPOINT)) then -- additional rotation of waypoints
        local rot = Vector2f()
        rot:y(  current_wp:lat() - basepoint:lat() )
        rot:x( (current_wp:lng() - basepoint:lng()) * longitude_scale(untranslated_loc:lat()) )
        rot:rotate((-1.0)*rotation)
        item:x(basepoint:lat() + rot:y())
        item:y(basepoint:lng() + rot:x() / longitude_scale(current_wp:lat()) )
      end
    end

    if not mission:set_item(index,item) then
      mission:clear() -- clear partly loaded mission
      error(string.format('%s: failed to set mission item %i', SCRIPT_NAME, index))
    end
    index = index + 1
  end
end


local function write_mission(write_path, write_file)
-- writing a mission from FC to file

  -- check if there is a mission to save
  local num_wp = mission:num_commands()
  if num_wp <= 1 then
    send_msg(WARN,'no mission on FC')
    return false
  end

  -- create new file
  file = assert(io.open(string.format('%s/%s', write_path, write_file), 'w+'), string.format('%s: Could not create %s', SCRIPT_NAME, write_file))

  -- header
  file:write('QGC WPL 110\n')

  -- read each item and write to file
  for i = 0, num_wp - 1 do
    local item = mission:get_item(i)
    file:write(string.format('%i\t0\t%i\t%i\t%0.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1\n',item:seq(),item:frame(),item:command(),item:param1(),item:param2(),item:param3(),item:param4(),item:x()*10^-7,item:y()*10^-7,item:z()))
  end
  file:close()
  send_msg(INFO,string.format('%s saved (%d items)', write_file, num_wp-1))
  return true
end


local function change_mission_allowed()
-- check, if loading/starting a new Mission is allowed
  local mode = vehicle:get_mode()
  local mode_reason = vehicle:get_control_mode_reason()
  local armed = arming:is_armed()
  if ((mode ~= MODE_AUTO) and armed
    and
    ((mode_reason == REASON_RC) or
    (mode_reason == REASON_GCS) or
    (mode_reason == REASON_INITIALIZED)))
  then
    return true
  else
    if mode == MODE_AUTO then
      send_msg(WARN, 'new Mission forbidden (mode = AUTO)')
    elseif not armed then
      send_msg(WARN, 'new Mission forbidden (not armed)')
    else
      send_msg(WARN, string.format('new Mission forbidden (mode:%d reason:%d)',mode, mode_reason))
    end
    return false
  end
end
  
local function update_params()
  -- read parameter and if necessary find corresponding rc-channel for the switch
  local option_value = RELOCATION:get()
  if (option_value >= NO_RELOCATION) and (option_value <= TRANSLATE_ROTATE) then -- kind of relocation directly via parameter
    kind_of_relocation = option_value
    scr_rc_relocation = nil
  elseif (option_value >= 300) and (option_value <= 307) then -- kind of relocation via switch
    -- find channel for kind-of-relocation-switch
    scr_rc_relocation = rc:find_channel_for_option(option_value)
    if not scr_rc_relocation then
      send_msg(WARN, string.format('RCx_OPTION(RELOCATION) = %d not found', option_value))
      return false
    end
  else
    send_msg(WARN, string.format('RELOCATION: %i out of range (0..2, 300..307)', option_value))
    return false
  end

  -- read parameter and set the no_relocation_radius
  option_value = NO_RELOC_M:get()
  if (option_value >= 0) then
    if no_relocation_radius ~= option_value then
      send_msg(INFO, string.format('safety radius %im', option_value))
      no_relocation_radius = option_value
    end
  else
    if no_relocation_radius ~= 50 then
      send_msg(INFO, 'safety radius forced to 50m')
      no_relocation_radius = 50
    end
  end

  -- read parameter number of switch positions
  option_value = POSITIONS:get()
  if option_value then
    if option_value ~= num_sw_positions then
      if option_value == 1 then
        send_msg(INFO, string.format('select mission by pushbutton'))
      elseif option_value > 1 then
        send_msg(INFO, string.format('select mission by switch: %i pos',option_value))
      end
    end
    if option_value < 1 then
      send_msg(INFO, string.format('script is pausing (%sPOSITIONS<1)', PARAM_PREFIX))
    end
    num_sw_positions = option_value
  end
  return true
end
  
local function start_relocation(file_name)
-- check switch for kind of relocation, start relocation accordingly and switch to MODE_AUTO
  local mode = vehicle:get_mode()
  local mode_reason = vehicle:get_control_mode_reason()
  local relocation = kind_of_relocation
  if change_mission_allowed() then
    basepoint = assert(ahrs:get_location(), string.format('%s: no location available', SCRIPT_NAME)) -- memorize current location
    local home = assert(ahrs:get_home(), string.format('%s: home not available', SCRIPT_NAME))
    if not update_params() then return end
    -- test, if out of safety-radius
    if  (basepoint:get_distance(home) <= no_relocation_radius) and (no_relocation_radius > 0) then
      relocation = NO_RELOCATION
      send_msg(WARN, string.format('no Relocation within safety radius (dist:%.1fm)', basepoint:get_distance(home)))
    elseif scr_rc_relocation then
      relocation = scr_rc_relocation:get_aux_switch_pos()
    end
    if relocation == TRANSLATE_ROTATE then
      send_msg(INFO, 'translation & rotation')
      rotation = assert(ahrs:get_yaw(), 'no yaw available') -- memorize current heading
      send_msg(INFO, string.format('rotation %.1f °', math.deg(rotation)) )
    elseif relocation == ONLY_TRANSLATE then
      send_msg(INFO, 'translation only')
    else
      send_msg(INFO, 'no relocation')
    end
    if read_mission(sub_dir, file_name, relocation) then
      vehicle:set_mode(MODE_AUTO) -- switch to mode-AUTO
    end
  end
end

local function check_button()
  -- check if pushbutton changed and react
  local previous_button_position = last_button_position
  local button_position          = scr_rc_start:get_aux_switch_pos()
  last_button_position = button_position
  -- no change
  if (button_position == 0) and (previous_button_position == 0) then 
    wait_finger_off = false
    return
  end
  if wait_finger_off then return end
  -- pressed
  if (button_position >= 1) and (previous_button_position == 0) then
    last_sw_change = millis()
    return
  end
  -- check if long-pressed
  local long_press = (millis() - last_sw_change) > LONG_PRESS_MS
  if long_press then
      start_relocation(string.format('%s_%s%d.%s', SCRIPT_NAME, MISSION_FILE_PREFIX, file_index, FILE_EXTENSION))
      wait_finger_off = true
      return
  elseif (button_position == 0) and (previous_button_position >= 1) and (num_sw_positions ==1) then
    file_index = (file_index + 1) % num_files
    send_msg(INFO, string.format('Selected %s_%s%d', SCRIPT_NAME, MISSION_FILE_PREFIX, file_index))
  end
  return
end
  
local function check_switch()
-- check, if switch position has changed
  local sw_pos = (scr_rc_select:norm_input_ignore_trim() + 1) * (num_sw_positions - 1) / 2
  sw_pos = tonumber(string.format('%.0f', sw_pos)) or -1
  if sw_pos ~= last_switch_position then
  -- action if switch has changed
    file_index = sw_pos
    send_msg(INFO, string.format('Selected %s_%s%d', SCRIPT_NAME, MISSION_FILE_PREFIX, file_index))
    last_switch_position = sw_pos
  end
end

local function standby()
-- standby or switch to the preferred selection-variant
  if not update_params() then return standby, INTERRUPTED_MS end
  if num_sw_positions > 1 then check_switch() end
  if num_sw_positions > 0 then check_button() end
  if num_sw_positions < 1 then 
    return standby, SBY_INTERVAL_MS 
  end
  return standby, RUN_INTERVAL_MS
end



local function initialize()
-- initializing parameters, filepath, options, switches, files and mission

-- add script-specific parameter-table
  if not add_params(param_table_key, PARAM_PREFIX, PARAM_TABLE) then
    if param_table_key < 200 then
      param_table_key = param_table_key + 1
      return initialize, SHORT_INTERV_MS
    else
      send_msg(WARN, string.format('Could not add table %s', PARAM_PREFIX))
      return initialize, INTERRUPTED_MS
    end
  end

-- get the path to the scripts directory. This will be scripts/ on SITL and APM/scripts on a ChibiOS board
  local dlist = dirlist(SUB_DIR_APM)
  if dlist and #dlist > 0 then
    sub_dir = string.format('%s/%s', SUB_DIR_APM, SUB_DIR_MISSIONS)
  else
    -- otherwise assume scripts/
    sub_dir = string.format('%s/%s', SUB_DIR_SITL, SUB_DIR_MISSIONS)
  end

  local option_name = string.format('%s%s', PARAM_PREFIX, 'POSITIONS')
  if not POSITIONS:init(option_name) then
    send_msg(WARN, string.format('get parameter %s failed', option_name))
    return initialize, INTERRUPTED_MS
  end

  option_name = string.format('%s%s', PARAM_PREFIX, 'RELOCATION')
  if not RELOCATION:init(option_name) then
    send_msg(WARN, string.format('get parameter %s failed', option_name))
    return initialize, INTERRUPTED_MS
  end

  option_name = string.format('%s%s', PARAM_PREFIX, 'NO_RELOC_M')
  if not NO_RELOC_M:init(option_name) then
    send_msg(WARN, string.format('get parameter %s failed', option_name))
    return initialize, INTERRUPTED_MS
  end

  if not update_params() then return initialize, INTERRUPTED_MS end

  -- read the selected option in SRM_ parameter and find corresponding rc-channel for the start-button  
  option_name = string.format('%s%s', PARAM_PREFIX, 'RC_START')
  local option_value = param:get(option_name)
  if option_value then
    if (option_value >= 300) and (option_value <= 307) then
      -- find channel for start-button
      scr_rc_start = rc:find_channel_for_option(option_value)
      if not scr_rc_start then
        send_msg(WARN, string.format('RCx_OPTION(%s) = %d not found', option_name, option_value))
        return initialize, INTERRUPTED_MS
      end
    else
      send_msg(WARN, string.format('%s: %i out of range (300..307)', option_name, option_value))
      return initialize, INTERRUPTED_MS
    end
  else
    send_msg(WARN, string.format('get parameter %s failed', option_name))
    return initialize, INTERRUPTED_MS
  end

  -- read the selected option in SRM_ parameter and find corresponding rc-channel for select-switch (if any)
  option_name = string.format('%s%s', PARAM_PREFIX, 'RC_SELECT')
  option_value = param:get(option_name)
  if option_value then
    if (option_value == 0) then
      scr_rc_select = scr_rc_start
      send_msg(INFO, 'selection via startbutton')
    elseif (option_value >= 300) and (option_value <= 307) then
      scr_rc_select = rc:find_channel_for_option(option_value)
      if not scr_rc_select then
        send_msg(WARN, string.format('RCx_OPTION(%s) = %d not found', option_name, option_value))
        return initialize, INTERRUPTED_MS
      end
    else
      send_msg(WARN, string.format('%s: %i out of range (300..307)', option_name, option_value))
      return initialize, INTERRUPTED_MS
    end
  else
    send_msg(WARN, string.format('get parameter %s failed', option_name))
    return initialize, INTERRUPTED_MS
  end

  -- backup mission on FC to file
  if mission:num_commands() <= 1 then
    send_msg(INFO,'no mission on FC to backup')
  else
    -- try to read count-file
    local count_file = io.open(string.format('%s/%s_%s', sub_dir, SCRIPT_NAME, COUNT_FILE), 'r')
    local file_counter = 0
    if count_file then
      file_counter = count_file:read('*number')
      count_file:close()
      -- update count
      if file_counter then
        if file_counter >= BACKUP_FILE_MAX then
          file_counter = 1
        else
          file_counter = file_counter + 1
        end
      else
        file_counter = 1
      end
    end
    -- write backup
    local backup_name = string.format('%s_%s%d.%s', SCRIPT_NAME, BACKUP_FILE_PREFIX, file_counter, FILE_EXTENSION)
    if not write_mission(sub_dir, backup_name) then
      send_msg(WARN, string.format('not able to write %s', backup_name))
      return initialize, INTERRUPTED_MS
    end
    -- write count-file
    count_file = io.open(string.format('%s/%s_%s', sub_dir, SCRIPT_NAME, COUNT_FILE), 'w')
    if count_file then
      count_file:write(string.format('%d', file_counter))
      count_file:close()
    else
      send_msg(WARN, string.format('not able to write %s', count_file))
      return initialize, INTERRUPTED_MS
    end
  end

  -- count available missions Mission#x.waypoints
  num_files = 0
  while true do
    file = io.open(string.format('%s/%s', sub_dir, string.format('%s_%s%d.%s', SCRIPT_NAME, MISSION_FILE_PREFIX, num_files, FILE_EXTENSION)))
    if file then
      num_files = num_files + 1
      file:close()
    else
      send_msg(INFO, string.format('%d missions found', num_files))
      break
    end
  end

  if (num_sw_positions > 1) then
    if  (num_files < num_sw_positions) then
      send_msg(WARN, string.format('only %d files found', num_files))
      return init, INTERRUPTED_MS
    end
    if num_sw_positions > 1 then check_switch() end --to preselect the switched mission#
  end  

  if not read_mission(sub_dir, string.format('%s_%s%d.%s', SCRIPT_NAME, MISSION_FILE_PREFIX, file_index, FILE_EXTENSION), NO_RELOCATION) then
    return initialize, INTERRUPTED_MS
  end

  send_msg(INFO, 'Script running')
  return standby, RUN_INTERVAL_MS -- init done -> regular loop start

end

return initialize()
