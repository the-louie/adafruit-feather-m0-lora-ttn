/**
 * TTN / The Things Stack custom payload decoder for M0 batched log uplinks (v2.8).
 * FPort 1: batch 7-byte header [vbat, flags, n, baseTick_3B] + 2×n; flags bit0 = watchdog_triggered.
 * FPort 2: HELLO WORLD string. FPort 4: 3-byte time request [vbat_hi, vbat_lo, 0x00] (time_request: true).
 * v2.6 legacy: 6-byte header (no flags). Entry i timestamp = baseTick + i*5 min since custom epoch.
 *
 * In Console → Application → Payload Formats → Custom: paste this as decoder.
 */
function decodeUplink(input) {
  var b = input.bytes;
  if (!b || b.length === 0) return { data: {} };

  // FPort 2: simple HELLO WORLD string uplink
  if (input.fPort === 2) {
    var text = '';
    for (var t = 0; t < b.length; t++) text += String.fromCharCode(b[t] & 0xFF);
    return { data: { message: text } };
  }

  // FPort 4: 3-byte time request when RTC not synced
  if (input.fPort === 4 && b.length >= 2) {
    var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
    return { data: { battery_v: battery_v, time_request: true } };
  }

  // FPort 1: batch with 7-byte header [vbat(2)] [flags(1)] [n(1)] [baseTick(3)] = 7 bytes
  if (b.length < 7) return { data: {} };

  var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
  var flags = b[2] & 0xFF;
  var n = b[3] & 0xFF;
  var baseTick = ((b[4] << 16) | (b[5] << 8) | b[6]) >>> 0;

  var watchdog_triggered = (flags & 0x01) !== 0;

  var customEpoch = 1735689600 * 1000; // 2026-01-01
  var receivedAtMs = new Date(input.recvTime || Date.now()).getTime();

  var entries = [];
  for (var i = 0; i < n; i++) {
    var j = 7 + i * 2; // Offset by 7-byte header
    if (j + 1 >= b.length) break;

    var temp = ((b[j] << 8) | b[j + 1]) >>> 0;
    var ts;

    // Implicit timing: entry i is at baseTick + (i * 5 minutes)
    if (baseTick === 0) {
      // RTC not synced: backfill from receipt time
      ts = new Date(receivedAtMs - (n - 1 - i) * 300000).toISOString();
    } else {
      var tickMinutes = baseTick + (i * 5);
      ts = new Date(customEpoch + tickMinutes * 60000).toISOString();
    }

    var result = { timestamp: ts };
    if (temp === 0xFFFF) result.temperature_state = 'error';
    else if (temp === 0xFFFE) result.temperature_state = 'under_range';
    else if (temp === 0xFFFD) result.temperature_state = 'over_range';
    else result.temperature_c = Number((temp / 100).toFixed(2));

    entries.push(result);
  }

  return {
    data: {
      battery_v: battery_v,
      watchdog_triggered: watchdog_triggered,
      entries: entries
    }
  };
}