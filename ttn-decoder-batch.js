/**
 * TTN / The Things Stack custom payload decoder for M0 batched log uplinks.
 * v3.1 Timeless: FPort 10 = 300 s, FPort 20 = 10800 s. 4-byte header [vbat(2), flags(1), sequence(1)] + 2×n. Time from gateway received_at; entry i = anchor - i*interval.
 * Flags bit 1 = cold boot (boot_event); firmware v3.4+ ensures at least one entry when boot_event is set, for backend verification of the hardware stack.
 * FPort 2: HELLO WORLD. FPort 4: legacy 3-byte time request. FPort 1: legacy v2.8 batch (7-byte header + baseTick).
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

  // v3.1 Timeless: FPort 10 (300 s) or FPort 20 (10800 s), 4-byte header + 2×n entries, newest first
  if ((input.fPort === 10 || input.fPort === 20) && b.length >= 4) {
    var interval = (input.fPort === 20) ? 10800 : 300;
    var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
    var flags = b[2] & 0xFF;
    var seq = b[3] & 0xFF;
    var anchorTimeMs = new Date(input.recvTime || input.uplink_message?.received_at || Date.now()).getTime();
    var n = Math.floor((b.length - 4) / 2);
    var entries = [];
    for (var i = 0; i < n; i++) {
      var ptr = 4 + i * 2;
      var rawTemp = ((b[ptr] << 8) | b[ptr + 1]) >>> 0;
      var offsetMs = i * interval * 1000;
      var timestamp = new Date(anchorTimeMs - offsetMs).toISOString();
      var entry = { timestamp: timestamp };
      if (rawTemp === 0xFFFF) entry.temperature_state = 'error';
      else if (rawTemp === 0xFFFE) entry.temperature_state = 'under_range';
      else if (rawTemp === 0xFFFD) entry.temperature_state = 'over_range';
      else entry.temperature_c = Number((rawTemp / 100).toFixed(2));
      entries.push(entry);
    }
    return {
      data: {
        battery_v: battery_v,
        sequence: seq,
        watchdog_triggered: (flags & 0x01) !== 0,
        boot_event: (flags & 0x02) !== 0,
        interval_seconds: interval,
        entries: entries
      }
    };
  }

  // FPort 4: legacy 3-byte time request
  if (input.fPort === 4 && b.length >= 2) {
    var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
    return { data: { battery_v: battery_v, time_request: true } };
  }

  // FPort 1: legacy v2.8 batch with 7-byte header [vbat(2)] [flags(1)] [n(1)] [baseTick(3)]
  if (b.length < 7) return { data: {} };

  var battery_v = (((b[0] << 8) | b[1]) >>> 0) / 100;
  var flags = b[2] & 0xFF;
  var n = b[3] & 0xFF;
  var baseTick = ((b[4] << 16) | (b[5] << 8) | b[6]) >>> 0;
  var watchdog_triggered = (flags & 0x01) !== 0;
  var customEpoch = 1735689600 * 1000;
  var receivedAtMs = new Date(input.recvTime || Date.now()).getTime();
  var entries = [];
  for (var i = 0; i < n; i++) {
    var j = 7 + i * 2;
    if (j + 1 >= b.length) break;
    var temp = ((b[j] << 8) | b[j + 1]) >>> 0;
    var ts;
    if (baseTick === 0) {
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
