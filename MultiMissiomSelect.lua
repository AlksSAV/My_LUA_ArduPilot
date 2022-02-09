--[[----------------------------------------------------------------------------

------------------------------------------------------------------------------]]
local mission_loaded = false
local rc_switch = rc:find_channel_for_option(24)  --AUX FUNC sw for mission restart
local ENABLE_CHANNEL   = 15 -- 15 CH for 6 position sw
local lastPwm = rc:get_pwm(ENABLE_CHANNEL)
if not rc_switch then  -- requires the switch to be assigned in order to run script
  return
end
local FREQ = 500
local last_pwm = 0

local function read_mission(file_name)

   -- Open file try and read header
  local file = io.open(file_name,"r")
  local header = file:read('l')
  if not header then
    return update, 1000 --could not read, file probably does not exist
  end

  -- check header
  assert(string.find(header,'QGC WPL 110') == 1, file_name .. ': incorrect format')

  -- clear any existing mission
  assert(mission:clear(), 'Could not clear current mission')

  -- read each line and write to mission
  local item = mavlink_mission_item_int_t()
  local index = 0
  while true do
    local data = {}
    for i = 1, 12 do
      data[i] = file:read('n')
      if data[i] == nil then
        if i == 1 then
          gcs:send_text(6, 'loaded mission: ' .. file_name)
          file:close()
          return -- got to the end of the file
        else
          mission:clear() -- clear part loaded mission
          error('failed to read file')
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
      mission:clear() -- clear part loaded mission
      error(string.format('failed to set mission item %i',index))
    end
    index = index + 1
  end
  file:close()
end

function update()
  local somethingChanged = false
  if not arming:is_armed() and vehicle:get_mode() ~= ROVER_MODE_HOLD  then --if disarmed and no HOLD mode, wait until armed and HOLD
    mission_loaded = false
    return update,1000
  end
  if (pwm ~= lastPwm) then
    lastPwm = pwm
    somethingChanged = true
end
  if not mission_loaded then -- if first time after arm and switch is valid then try to load based on switch position
    local read_mission
    local pwm = RC:get_pwm(15) -- chanel 15 (6 pos switch)
    if pwm < 983 then
      read_mission('mission1.waypoints')
    elseif pwm < 1186 then
      read_mission('mission2.waypoints')
    elseif pwm < 1391 then
      read_mission('mission3.waypoints')
    elseif pwm < 1596 then
      read_mission('mission4.waypoints')
    elseif pwm < 1800 then
      read_mission('mission5.waypoints')
    else
      read_mission('mission6.waypoints')
     end
    mission_loaded = true
    read_mission(read_mission)
    end
    if (not somethingChanged) then return update, FREQ 
    end -- early return
  return update, FREQ
end 

return update, FREQ





