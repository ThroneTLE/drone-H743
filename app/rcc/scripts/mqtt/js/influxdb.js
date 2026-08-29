/*
 * InfluxDB line-protocol publisher.
 *
 * Line protocol:
 *   measurement[,tagKey=tagValue...] fieldKey=fieldValue[,...] [timestamp]
 *
 * Pipe this MQTT topic into a Telegraf MQTT consumer with the
 * `mqtt_consumer` input + `influx_v2` data_format and it lands directly
 * in your bucket.
 */

var MEASUREMENT = "serial_studio";
var TAGS = "device=dev01";
var FIELDS = ["temperature", "humidity", "pressure"];

function mqtt(frame) {
  var parts = String(frame).split(",");
  var fields = [];

  for (var i = 0; i < FIELDS.length && i < parts.length; ++i) {
    var v = parseFloat(parts[i]);
    if (!isFinite(v))
      continue;

    fields.push(FIELDS[i] + "=" + v);
  }

  if (fields.length === 0)
    return null;

  // Nanosecond timestamp; Date.now() is ms so multiply by 1e6
  var ts = (Date.now() * 1e6).toFixed(0);
  return MEASUREMENT + "," + TAGS + " " + fields.join(",") + " " + ts + "\n";
}
