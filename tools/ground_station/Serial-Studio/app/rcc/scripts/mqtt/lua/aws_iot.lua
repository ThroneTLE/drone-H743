--
-- AWS IoT Core / Azure IoT Hub device-shadow update.
--
-- Both platforms accept identical JSON via MQTT:
--   AWS:    $aws/things/<thingName>/shadow/update
--   Azure:  $iothub/twin/PATCH/properties/reported/?$rid=<requestId>
--
-- The payload shape {"state":{"reported":{...}}} works for both.
--

local FIELDS = { "temperature", "humidity", "pressure" }

local function split(s, sep)
  local out, i = {}, 1
  for part in string.gmatch(s, "([^" .. sep .. "]+)") do
    out[i] = part
    i = i + 1
  end
  return out
end

function mqtt(frame)
  local parts = split(tostring(frame), ",")
  local body  = {}

  for i, key in ipairs(FIELDS) do
    local v = tonumber(parts[i]) or "null"
    table.insert(body, string.format('"%s":%s', key, tostring(v)))
  end

  return '{"state":{"reported":{' .. table.concat(body, ",") .. '}}}\n'
end
