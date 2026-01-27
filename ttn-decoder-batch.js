/**
 * TTN / The Things Stack custom payload decoder for M0 batched log uplinks.
 * Payload (big-endian): [vbat_hi, vbat_lo] (V×100), [n], [timeTick_hi, timeTick_lo, temp_hi, temp_lo] × n.
 * Temperature: 0–3000 = centidegrees; 0xFFFD => >30°C, 0xFFFE => <0°C, 0xFFFF = error.
 * TimeTick = 30-min ticks since 2026-01-01 00:00:00 UTC.
 *
 * In Console → Application → Payload Formats → Custom: paste this as decoder.
 * Accepts input.bytes (byte array) or input.frm_payload (base64 string) for compatibility.
 */
function decodeUplink(input) {
  var b = input.bytes;
  if (!b || b.length < 3) {
    if (input.frm_payload && typeof atob === 'function') {
      var raw = atob(input.frm_payload);
      b = [];
      for (var i = 0; i < raw.length; i++) b.push(raw.charCodeAt(i));
    }
  }
  if (!b || b.length < 3) return { data: {} };
  if (typeof b.slice === 'function') b = Array.from(b);

  var battery_v = ((b[0] << 8) | b[1]) / 100;
  var n = b[2] & 0xFF;
  var maxN = (b.length - 3) >> 2;
  if (n > maxN) n = maxN;

  var entries = [];
  var customEpoch = new Date('2026-01-01T00:00:00Z').getTime();
  for (var i = 0, j = 3; i < n && j + 4 <= b.length; i++, j += 4) {
    var ticks = (b[j] << 8) | b[j + 1];
    var temp = (b[j + 2] << 8) | b[j + 3];
    var ts = new Date(customEpoch + ticks * 1800 * 1000).toISOString();
    if (temp === 0xFFFF) {
      entries.push({ timestamp: ts, temperature_state: 'error' });
    } else if (temp === 0xFFFE) {
      entries.push({ timestamp: ts, temperature_state: 'under_range' });
    } else if (temp === 0xFFFD) {
      entries.push({ timestamp: ts, temperature_state: 'over_range' });
    } else {
      entries.push({ timestamp: ts, temperature_c: temp / 100 });
    }
  }

  return { data: { battery_v: battery_v, entries: entries } };
}
