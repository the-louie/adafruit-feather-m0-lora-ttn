/**
 * TTN / The Things Stack custom payload decoder for M0 batched log uplinks.
 * Payload (big-endian): [vbat_hi, vbat_lo] (V×100), [n], [timeTick_hi, timeTick_mid, timeTick_lo, temp_hi, temp_lo] × n (5 bytes per entry).
 * Temperature: 0–3000 = centidegrees; 0xFFFD => >30°C, 0xFFFE => <0°C, 0xFFFF = error.
 * TimeTick = 1-min ticks since 2026-01-01 00:00:00 UTC (24-bit).
 * When tick is 0 (device RTC not yet synced), timestamp is derived from received_at and 5-min spacing.
 *
 * In Console → Application → Payload Formats → Custom: paste this as decoder.
 * Accepts input.bytes (byte array) or input.frm_payload (base64 string) for compatibility.
 */
function decodeUplink(input) {
  var b = input.bytes;
  if (!b || b.length === 0) {
    if (input.frm_payload && typeof atob === 'function') {
      var raw = atob(input.frm_payload);
      b = [];
      for (var i = 0; i < raw.length; i++) b.push(raw.charCodeAt(i));
    }
  }
  if (!b || b.length === 0) return { data: {} };
  if (typeof b.slice === 'function') b = Array.from(b);

  // FPort 2: simple HELLO WORLD string uplink
  if (input.fPort === 2) {
    var text = '';
    for (var t = 0; t < b.length; t++) {
      text += String.fromCharCode(b[t] & 0xFF);
    }
    return { data: { message: text } };
  }

  // Default (FPort 1): batched log payload
  if (b.length < 3) return { data: {} };

  var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
  var n = b[2] & 0xFF;
  var maxN = ((b.length - 3) / 5) | 0;
  if (n > maxN) n = maxN;

  var customEpoch = new Date('2026-01-01T00:00:00Z').getTime();
  var receivedAtMs = null;
  if (input.recvTime && typeof input.recvTime.getTime === 'function') {
    receivedAtMs = input.recvTime.getTime();
  } else if (input.uplink_message && input.uplink_message.received_at) {
    receivedAtMs = new Date(input.uplink_message.received_at).getTime();
  }

  var entries = [];
  for (var i = 0, j = 3; i < n && j + 5 <= b.length; i++, j += 5) {
    var ticks = ((b[j] << 16) | (b[j + 1] << 8) | b[j + 2]) >>> 0;
    var temp = ((b[j + 3] << 8) | b[j + 4]) >>> 0;
    var ts;
    if (ticks === 0 && receivedAtMs !== null && n > 0) {
      var offsetMs = (n - 1 - i) * 300 * 1000;
      ts = new Date(receivedAtMs - offsetMs).toISOString();
    } else {
      ts = new Date(customEpoch + ticks * 60 * 1000).toISOString();
    }
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
