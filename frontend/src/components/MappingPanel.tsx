import { useState } from 'react';
import { useModeAndMaps, useRobotState, usePointsOfInterest, PointOfInterest } from '../ros/hooks';
import { setMode, loadMap, stopSlamAndSave } from '../ros/services';
import { TeleopBlock } from './TeleopBlock';

type Props = {
  goToLab: (poi: PointOfInterest) => void;
  onStop: () => void;
  disabledMove?: boolean;
}

export function MappingPanel({ goToLab, onStop, disabledMove }: Props) {
  const { mode, activeMap, maps, loading, error, refresh } = useModeAndMaps();
  const robotState = useRobotState();
  const pois = usePointsOfInterest();
  const [operating, setOperating] = useState(false);
  const [statusMsg, setStatusMsg] = useState<string | null>(null);
  const [selectedMap, setSelectedMap] = useState<string>('');
  const [selectedPoi, setSelectedPoi] = useState<string>('');
  const selectedPoiData = pois.find(p => `${p.x || p.pose?.x},${p.y || p.pose?.y}` === selectedPoi);
  const hasPois = pois.length > 0;
  const inSlamMode = mode === 'slam';
  const canNavigate = !(disabledMove) && !inSlamMode;
  const stopEnabled = canNavigate && (robotState ? robotState !== 'idle' : true);

  const handleStartMapping = async () => {
    setOperating(true);
    setStatusMsg(null);
    try {
      await setMode('slam');
      setStatusMsg('✓ Mapping mode started');
      refresh();
    } catch (e: any) {
      setStatusMsg(`✗ Failed to start mapping: ${e?.message || 'Unknown error'}`);
    } finally {
      setOperating(false);
    }
  };

  const handleStopAndSave = async () => {
    const mapName = prompt('Enter map name to save:');
    if (!mapName || !mapName.trim()) {
      setStatusMsg('✗ Map name required');
      return;
    }

    setOperating(true);
    setStatusMsg(null);
    try {
      await stopSlamAndSave(mapName.trim());
      setStatusMsg(`✓ Map "${mapName}" saved successfully`);
      refresh();
    } catch (e: any) {
      setStatusMsg(`✗ Failed to save map: ${e?.message || 'Unknown error'}`);
    } finally {
      setOperating(false);
    }
  };

  const handleLoadMap = async () => {
    if (!selectedMap) {
      setStatusMsg('✗ Select a map first');
      return;
    }

    setOperating(true);
    setStatusMsg(null);
    try {
      await loadMap(selectedMap);
      setStatusMsg(`✓ Loaded map "${selectedMap}"`);
      refresh();
    } catch (e: any) {
      setStatusMsg(`✗ Failed to load map: ${e?.message || 'Unknown error'}`);
    } finally {
      setOperating(false);
    }
  };

  const handleSetLocalization = async () => {
    setOperating(true);
    setStatusMsg(null);
    try {
      await setMode('localization');
      setStatusMsg('✓ Switched to localization mode');
      refresh();
    } catch (e: any) {
      setStatusMsg(`✗ Failed to switch mode: ${e?.message || 'Unknown error'}`);
    } finally {
      setOperating(false);
    }
  };

  const handleSetIdle = async () => {
    setOperating(true);
    setStatusMsg(null);
    try {
      await setMode('idle');
      setStatusMsg('✓ Switched to idle mode');
      refresh();
    } catch (e: any) {
      setStatusMsg(`✗ Failed to switch mode: ${e?.message || 'Unknown error'}`);
    } finally {
      setOperating(false);
    }
  };

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      {/* Header */}
      <div className="flex items-center justify-between mb-3 pb-3 border-b border-slate-200">
        <h2 className="text-lg font-semibold text-slate-800">Control Center</h2>
        <button
          onClick={refresh}
          disabled={operating || loading}
          className="px-3 py-1 text-sm rounded bg-slate-100 hover:bg-slate-200 text-slate-700 border border-slate-300 disabled:opacity-40 disabled:cursor-not-allowed transition-colors"
          title="Refresh"
        >
          <span className={loading ? 'inline-block animate-spin' : ''}>↻</span>
        </button>
      </div>

      {/* Status Messages */}
      {error && (
        <div className="bg-red-50 rounded px-3 py-2 text-red-800 text-sm mb-3 border-l-4 border-red-500">
          <span className="font-semibold">⚠</span> {error}
        </div>
      )}
      {statusMsg && (
        <div className={`rounded px-3 py-2 text-sm mb-3 border-l-4 ${
          statusMsg.startsWith('✓')
            ? 'bg-green-50 text-green-800 border-green-500'
            : 'bg-red-50 text-red-800 border-red-500'
        }`}>
          {statusMsg}
        </div>
      )}

      <div className="space-y-4">
        {/* Current Status */}
        <div className={`rounded p-3 border ${
          mode === 'slam' ? 'bg-amber-50 border-amber-300' :
          mode === 'localization' ? 'bg-blue-50 border-blue-300' :
          'bg-slate-50 border-slate-300'
        }`}>
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-slate-700">Mode</span>
            <span className={`px-3 py-1 rounded-full text-xs font-bold ${
              mode === 'slam' ? 'bg-amber-500 text-white' :
              mode === 'localization' ? 'bg-blue-500 text-white' :
              'bg-slate-500 text-white'
            }`}>
              {loading ? '...' : mode?.toUpperCase() || 'UNKNOWN'}
            </span>
          </div>
          <div className="flex items-center justify-between pt-2 border-t border-slate-200">
            <span className="text-sm font-medium text-slate-700">Map</span>
            <span className="text-sm font-mono text-slate-900">
              {loading ? '...' : activeMap || 'None'}
            </span>
          </div>
        </div>

        {/* Mode Buttons */}
        <div>
          <div className="flex gap-2">
            <button
              onClick={handleStartMapping}
              disabled={operating || loading || mode === 'slam'}
              className="flex-1 px-3 py-2 bg-amber-500 hover:bg-amber-600 disabled:bg-slate-200 text-white disabled:text-slate-400 rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
            >
              SLAM
            </button>
            <button
              onClick={handleSetLocalization}
              disabled={operating || loading || mode === 'localization'}
              className="flex-1 px-3 py-2 bg-blue-500 hover:bg-blue-600 disabled:bg-slate-200 text-white disabled:text-slate-400 rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
            >
              Localize
            </button>
            <button
              onClick={handleSetIdle}
              disabled={operating || loading || mode === 'idle'}
              className="flex-1 px-3 py-2 bg-slate-500 hover:bg-slate-600 disabled:bg-slate-200 text-white disabled:text-slate-400 rounded font-semibold text-sm transition-colors disabled:cursor-not-allowed"
            >
              Idle
            </button>
          </div>
        </div>

        {/* SLAM Save */}
        {mode === 'slam' && (
          <div className="bg-amber-50 border border-amber-300 rounded p-3">
            <p className="text-xs text-amber-800 mb-2 font-medium">⚡ Mapping Active</p>
            <button
              onClick={handleStopAndSave}
              disabled={operating || loading}
              className="w-full px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-slate-300 text-white disabled:text-slate-500 rounded font-semibold transition-colors disabled:cursor-not-allowed"
            >
              💾 Stop & Save Map
            </button>
          </div>
        )}

        {/* Map Controls */}
        <div>
          <label className="block text-xs font-semibold text-slate-600 mb-1 uppercase">Load Map</label>
          <select
            value={selectedMap}
            onChange={(e) => setSelectedMap(e.target.value)}
            disabled={operating || loading}
            className="w-full px-3 py-2 mb-2 bg-white text-slate-900 rounded border border-slate-300 focus:border-blue-500 focus:ring-1 focus:ring-blue-500 focus:outline-none disabled:bg-slate-100 disabled:text-slate-400"
          >
            <option value="">Select a map...</option>
            {maps.map(m => (
              <option key={m} value={m}>{m}</option>
            ))}
          </select>
          <button
            onClick={handleLoadMap}
            disabled={operating || loading || !selectedMap || mode === 'slam'}
            className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-slate-200 text-white disabled:text-slate-400 rounded font-semibold transition-colors disabled:cursor-not-allowed"
          >
            📍 Load
          </button>
        </div>

        {/* Navigation */}
        {inSlamMode && (
          <div className="bg-yellow-50 border border-yellow-300 rounded px-3 py-2 text-sm text-yellow-900">
            ⚠️ Navigation disabled in mapping mode
          </div>
        )}
        <div>
          <label className="block text-xs font-semibold text-slate-600 mb-1 uppercase">Navigate To</label>
          <select
            value={selectedPoi}
            onChange={(e) => setSelectedPoi(e.target.value)}
            disabled={!hasPois}
            className={`w-full px-3 py-2 mb-2 rounded border focus:outline-none focus:ring-1 ${
              hasPois
                ? 'border-slate-300 bg-white text-slate-900 focus:border-blue-500 focus:ring-blue-500'
                : 'border-slate-300 bg-slate-100 text-slate-400'
            }`}
          >
            <option value="">{hasPois ? 'Select destination…' : 'No destinations'}</option>
            {pois.map((poi, idx) => {
              const x = poi.pose?.x ?? poi.x ?? 0;
              const y = poi.pose?.y ?? poi.y ?? 0;
              return (
                <option key={idx} value={`${x},${y}`}>
                  {poi.name} ({x.toFixed(1)}, {y.toFixed(1)})
                </option>
              );
            })}
          </select>
          <div className="grid grid-cols-2 gap-2">
            <button
              onClick={() => { if (selectedPoiData) goToLab(selectedPoiData) }}
              disabled={!canNavigate || !selectedPoiData || !hasPois}
              className="px-4 py-2 rounded bg-blue-600 hover:bg-blue-700 text-white font-semibold disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
            >
              Go
            </button>
            <button
              onClick={onStop}
              disabled={!stopEnabled}
              className="px-4 py-2 rounded bg-red-600 hover:bg-red-700 text-white font-semibold disabled:opacity-40 disabled:cursor-not-allowed transition-colors"
            >
              Stop
            </button>
          </div>
        </div>

        {/* Teleop */}
        <div>
          <label className="block text-xs font-semibold text-slate-600 mb-2 uppercase">Teleop</label>
          <TeleopBlock />
        </div>
      </div>
    </section>
  );
}
