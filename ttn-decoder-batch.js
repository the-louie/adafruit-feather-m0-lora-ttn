/**
 * TTN / The Things Stack custom payload decoder for M0 batched log uplinks (v2.6 implicit-time format).
 * Payload (big-endian): [vbat_hi, vbat_lo] (V×100), [n], [baseTick_hi, baseTick_mid, baseTick_lo], [temp_hi, temp_lo] × n (2 bytes per entry).
 * Entry i timestamp = baseTick + i*5 minutes since custom epoch (5-min spacing). baseTick 24-bit.
 * Temperature: 0–3000 = centidegrees (÷100 → °C); 0xFFFD => >30°C, 0xFFFE => <0°C, 0xFFFF = error.
 * When baseTick is 0 (RTC not synced), timestamps derived from received_at and 5-min spacing.
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

  // FPort 1: batched log (v2.6 implicit time: 6-byte header + 2 bytes per entry)
  if (b.length < 6) return { data: {} };
  if ((b.length - 6) % 2 !== 0) return { data: {} };

  var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
  var n = b[2] & 0xFF;
  var baseTick = ((b[3] << 16) | (b[4] << 8) | b[5]) >>> 0;
  var maxN = (b.length - 6) >> 1;
  if (n > maxN) n = maxN;

  var customEpoch = 1735689600 * 1000;
  var receivedAtMs = null;
  if (input.recvTime && typeof input.recvTime.getTime === 'function') {
    receivedAtMs = input.recvTime.getTime();
  } else if (input.uplink_message && input.uplink_message.received_at) {
    receivedAtMs = new Date(input.uplink_message.received_at).getTime();
  }

  var entries = [];
  for (var i = 0; i < n; i++) {
    var j = 6 + i * 2;
    var temp = ((b[j] << 8) | b[j + 1]) >>> 0;
    var ts;
    if (baseTick === 0 && receivedAtMs !== null && n > 0) {
      var offsetMs = (n - 1 - i) * 300 * 1000;
      ts = new Date(receivedAtMs - offsetMs).toISOString();
    } else {
      var tickMinutes = baseTick + i * 5;
      ts = new Date(customEpoch + tickMinutes * 60 * 1000).toISOString();
    }
    if (temp === 0xFFFF) {
      entries.push({ timestamp: ts, temperature_state: 'error' });
    } else if (temp === 0xFFFE) {
      entries.push({ timestamp: ts, temperature_state: 'under_range' });
    } else if (temp === 0xFFFD) {
      entries.push({ timestamp: ts, temperature_state: 'over_range' });
    } else {
      entries.push({ timestamp: ts, temperature_c: Number((temp / 100).toFixed(2)) });
    }
  }

  return { data: { battery_v: battery_v, entries: entries } };
}
