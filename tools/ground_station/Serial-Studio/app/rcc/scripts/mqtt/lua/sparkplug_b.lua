--
-- Eclipse Sparkplug B "NDATA" message (simplified text-encoded preview).
--
-- Real Sparkplug B uses Google Protocol Buffers. This template emits a
-- human-readable surrogate payload that mirrors the Sparkplug B field
-- layout so you can validate routing before swapping in a proto encoder.
--
-- Recommended topic base: spBv1.0/<group_id>/NDATA/<edge_node_id>
--

local METRICS = { "temperature", "humidity", "pressure" }
local seq = 0

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
  local ts    = os.time() * 1000
  local items = {}

  for i, name in ipairs(METRICS) do
    local v = tonumber(parts[i]) or "null"
    table.insert(items, string.format(
      '{"name":"%s","timestamp":%d,"dataType":"Double","value":%s}',
      name, ts, tostring(v)))
  end

  seq = (seq + 1) % 256
  return string.format('{"timestamp":%d,"metrics":[%s],"seq":%d}\n',
                       ts, table.concat(items, ","), seq)
end
