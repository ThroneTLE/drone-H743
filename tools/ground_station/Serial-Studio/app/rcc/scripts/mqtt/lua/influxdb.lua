--
-- InfluxDB line-protocol publisher.
--
-- Line protocol:
--   measurement[,tag=value...] field=value[,field=value...] [timestamp]
--
-- Pipe this topic into a Telegraf mqtt_consumer input with the
-- influx_v2 data_format and it lands directly in your bucket.
--

local MEASUREMENT = "serial_studio"
local TAGS        = "device=dev01"
local FIELDS      = { "temperature", "humidity", "pressure" }

local function split(s, sep)
  local out, i = {}, 1
  for part in string.gmatch(s, "([^" .. sep .. "]+)") do
    out[i] = part
    i = i + 1
  end
  return out
end

function mqtt(frame)
  local parts  = split(tostring(frame), ",")
  local fields = {}

  for i, key in ipairs(FIELDS) do
    local v = tonumber(parts[i])
    if v then
      table.insert(fields, key .. "=" .. v)
    end
  end

  if #fields == 0 then
    return nil
  end

  -- Nanosecond timestamp; os.time() is seconds so multiply by 1e9
  local ts = string.format("%d", os.time() * 1000000000)
  return MEASUREMENT .. "," .. TAGS .. " "
       .. table.concat(fields, ",") .. " " .. ts .. "\n"
end
