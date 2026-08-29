/*
 * Home Assistant MQTT auto-discovery + state publish.
 *
 * Assumes the device emits comma-separated values, one per dataset, in the
 * order configured in the project. Adjust the FIELDS array below to match
 * your dataset titles and units.
 *
 * Topic Base in the MQTT view should be set to "homeassistant/sensor/<id>"
 * so this script publishes to:
 *   homeassistant/sensor/<id>/config  -- discovery payload (retained)
 *   homeassistant/sensor/<id>/state   -- state payload
 *
 * Most brokers (Mosquitto + Home Assistant) honor the retained discovery
 * message and surface the device automatically.
 */

var DEVICE_ID  = "serial_studio_device";
var DEVICE_NAME = "Serial Studio Device";
var FIELDS = [
  { key: "temperature", unit: "C"   },
  { key: "humidity",    unit: "%"   },
  { key: "pressure",    unit: "hPa" }
];

var discoverySent = false;

function mqtt(frame) {
  var parts = String(frame).split(",");
  var payload = {};
  for (var i = 0; i < FIELDS.length && i < parts.length; ++i) {
    var v = parseFloat(parts[i]);
    payload[FIELDS[i].key] = isFinite(v) ? v : parts[i];
  }

  // Publish discovery once; the Topic Base subscription handles the rest
  if (!discoverySent) {
    discoverySent = true;
    var discovery = {
      device: { identifiers: [DEVICE_ID], name: DEVICE_NAME },
      state_topic: "homeassistant/sensor/" + DEVICE_ID + "/state",
      unique_id: DEVICE_ID,
      value_template: "{{ value_json.temperature }}",
      unit_of_measurement: FIELDS[0].unit
    };
    // Discovery + state are concatenated on a single publish since the
    // MQTT worker aggregates script returns per tick.
    return JSON.stringify(discovery) + "\n" + JSON.stringify(payload);
  }

  return JSON.stringify(payload);
}
