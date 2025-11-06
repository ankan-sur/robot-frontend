import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

export function useRosStringJSON(topic: ROSLIB.Topic) {
  const [value, setValue] = useState<any>(null);
  useEffect(() => {
    const cb = (m: any) => {
      try {
        setValue(JSON.parse(m?.data));
      } catch {
        setValue(m?.data ?? null);
      }
    };
    topic.subscribe(cb);
    return () => topic.unsubscribe(cb);
  }, [topic]);
  return value;
}

export function useRosFloat(topic: ROSLIB.Topic) {
  const [value, set] = useState<number | null>(null);
  useEffect(() => {
    const cb = (m: any) => {
      const d = typeof m?.data === 'number' ? m.data : Number(m?.data);
      set(Number.isFinite(d) ? d : null);
    };
    topic.subscribe(cb);
    return () => topic.unsubscribe(cb);
  }, [topic]);
  return value;
}

