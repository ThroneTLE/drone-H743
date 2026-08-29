--
-- Home Assistant MQTT auto-discovery + state publish.
--
-- Assumes the device emits comma-separated values, one per dataset, in
-- the order configured in the project. Adjust the FIELDS table to match
-- your dataset titles and units.
--
-- Topic Base should be set to "homeassistant/sensor/<id>" so this
-- script publishes to:
--   homeassistant/sensor/<id>/config  -- discovery (retained)
--   homeassistant/sensor/<id>/state   -- state payload
--

local DEVICE_ID   = "serial_studio_device"
local DEVICE_NAME = "Serial Studio Device"
local FIELDS = { "temperature", "humidity", "pressure" }

local discovery_sent = false

local function split(s, sep)
  local out, i = {}, 1
  for part in string.gmatch(s, "([^" .. sep .. "]+)") do
    out[i] = part
    i = i + 1
  end
  return out
end

local function escape(s)
  return string.format("%q", tostring(s))
end

function mqtt(frame)
  local parts = split(tostring(frame), ",")
  local body  = {}

  for i, key in ipairs(FIELDS) do
    local v = tonumber(parts[i])
    table.insert(body, escape(key) .. ":" .. tostring(v or "null"))
  end

  local state = "{" .. table.concat(body, ",") .. "}"

  if not discovery_sent then
    discovery_sent = true
    local d = '{"device":{"identifiers":["' .. DEVICE_ID
            .. '"],"name":"' .. DEVICE_NAME
            .. '"},"state_topic":"homeassistant/sensor/' .. DEVICE_ID
            .. '/state","unique_id":"' .. DEVICE_ID
            .. '","value_template":"{{ value_json.temperature }}"}'
    return d .. "\n" .. state
  end

  return state
end
