import { describe, it, expect } from 'vitest';
import { parseBatteryMessage } from '../hooks';

describe('parseBatteryMessage', () => {
  it('parses UInt16 millivolts', () => {
    const msg = { data: 8365 };
    const out = parseBatteryMessage(msg);
    expect(out).not.toBeNull();
    expect(out?.millivolts).toBe(8365);
    expect(out?.volts).toBeCloseTo(8.365, 3);
    expect(out?.percent).toBeGreaterThan(80);
  });

  it('parses BatteryState with voltage', () => {
    const msg = { voltage: 7.4 };
    const out = parseBatteryMessage(msg);
    expect(out).not.toBeNull();
    expect(out?.volts).toBeCloseTo(7.4, 3);
    expect(out?.millivolts).toBe(7400);
  });

  it('parses percent field', () => {
    const msg = { percentage: 0.5 };
    const out = parseBatteryMessage(msg);
    expect(out).not.toBeNull();
    expect(out?.percent).toBe(50);
  });
});