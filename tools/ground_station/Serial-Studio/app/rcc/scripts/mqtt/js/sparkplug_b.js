/*
 * Eclipse Sparkplug B "NDATA" message (simplified text-encoded preview).
 *
 * Real Sparkplug B uses Google Protocol Buffers. This template emits a
 * human-readable surrogate payload that mirrors the Sparkplug B field
 * layout so you can validate dataset routing before swapping in a proto
 * encoder. Replace the body with a binary encoder (e.g. protobuf.js) for
 * production use.
 *
 * Recommended topic base: spBv1.0/<group_id>/NDATA/<edge_node_id>
 */

var METRICS = ["temperature", "humidity", "pressure"];
var seq = 0;

function mqtt(frame) {
  var parts = String(frame).split(",");
  var metrics = [];
  for (var i = 0; i < METRICS.length && i < parts.length; ++i) {
    var v = parseFloat(parts[i]);
    metrics.push({
      name: METRICS[i],
      timestamp: Date.now(),
      dataType: "Double",
      value: isFinite(v) ? v : null
    });
  }

  var ndata = {
    timestamp: Date.now(),
    metrics: metrics,
    seq: (seq++) & 0xff
  };

  return JSON.stringify(ndata) + "\n";
}
