import { useEffect, useState } from 'react';
import { ros } from '../ros/ros';
import { useRosConnection } from '../ros/hooks';

export default function DiagnosticsPanel() {
  const { connected } = useRosConnection();
  const [topics, setTopics] = useState<string[]>([]);
  const [services, setServices] = useState<string[]>([]);
  const [lastError, setLastError] = useState<string | null>(null);

  const refresh = () => {
    setLastError(null);
    try {
      (ros as any).getTopics((res: any) => {
        setTopics(res?.topics || []);
      });
    } catch (e: any) {
      setLastError(String(e));
      setTopics([]);
    }

    try {
      (ros as any).getServices((res: any) => {
        setServices(res || []);
      });
    } catch (e: any) {
      setLastError(prev => prev ? prev + '; ' + String(e) : String(e));
      setServices([]);
    }
  };

  useEffect(() => {
    if (!connected) {
      setTopics([]);
      setServices([]);
      return;
    }
    refresh();
  }, [connected]);

  // Small list of important items to highlight
  const important = [
    '/robot/state',
    '/available_maps',
    '/map',
    '/ascamera/camera_publisher/rgb0/image',
    '/ui/cmd_vel',
    '/cmd_vel',
    '/navigate_to_pose',
    '/navigate_to_pose/status',
  ];

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      <div className="flex justify-between items-start">
        <h2 className="text-lg font-semibold mb-2 text-slate-800">Diagnostics</h2>
        <div className="text-sm text-slate-500">Status: {connected ? 'connected' : 'disconnected'}</div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
        <div>
          <div className="text-sm text-slate-600 mb-1">Topics ({topics.length})</div>
          <div className="max-h-40 overflow-auto text-xs font-mono bg-slate-50 p-2 rounded">
            {topics.length === 0 ? <div className="text-slate-400">(none)</div> : topics.map(t => <div key={t}>{t}</div>)}
          </div>
        </div>

        <div>
          <div className="text-sm text-slate-600 mb-1">Services ({services.length})</div>
          <div className="max-h-40 overflow-auto text-xs font-mono bg-slate-50 p-2 rounded">
            {services.length === 0 ? <div className="text-slate-400">(none)</div> : services.map(s => <div key={s}>{s}</div>)}
          </div>
        </div>
      </div>

      <div className="mt-3 text-sm">
        <div className="mb-2 font-medium">Important checks</div>
        <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 text-xs">
          {important.map(i => {
            const ok = topics.includes(i) || services.includes(i) || false;
            return (
              <div key={i} className="flex items-center gap-2">
                <div className={ok ? 'text-emerald-600' : 'text-amber-600'}>{ok ? '✓' : '·'}</div>
                <div className="truncate">{i}</div>
              </div>
            );
          })}
        </div>
      </div>

      <div className="mt-3 flex items-center justify-between">
        <div className="text-xs text-slate-500">{lastError ? `Error: ${lastError}` : ''}</div>
        <div>
          <button onClick={refresh} className="px-2 py-1 text-xs bg-slate-100 rounded border">Refresh</button>
        </div>
      </div>
    </section>
  );
}
