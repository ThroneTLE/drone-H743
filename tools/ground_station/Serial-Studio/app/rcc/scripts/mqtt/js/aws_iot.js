/*
 * AWS IoT Core / Azure IoT Hub device-shadow update.
 *
 * AWS IoT Core listens for shadow updates at:
 *   $aws/things/<thingName>/shadow/update
 * Azure IoT Hub equivalent:
 *   $iothub/twin/PATCH/properties/reported/?$rid=<requestId>
 *
 * The payload shape ({"state":{"reported":{...}}}) is identical for both
 * platforms when used via MQTT, so this single template covers both.
 */

var FIELDS = ["temperature", "humidity", "pressure"];

function mqtt(frame) {
  var parts = String(frame).split(",");
  var reported = {};
  for (var i = 0; i < FIELDS.length && i < parts.length; ++i) {
    var v = parseFloat(parts[i]);
    reported[FIELDS[i]] = isFinite(v) ? v : parts[i];
  }

  return JSON.stringify({ state: { reported: reported } }) + "\n";
}
