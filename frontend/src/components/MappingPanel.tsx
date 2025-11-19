import { useState } from 'react';
import { useModeAndMaps } from '../ros/hooks';
import { setMode, loadMap, stopSlamAndSave } from '../ros/services';

export function MappingPanel() {
  const { mode, activeMap, maps, loading, error, refresh } = useModeAndMaps();
  const [operating, setOperating] = useState(false);
  const [statusMsg, setStatusMsg] = useState<string | null>(null);
  const [selectedMap, setSelectedMap] = useState<string>('');

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

  // Get mode display styling
  const getModeStyle = () => {
    switch (mode) {
      case 'slam':
        return {
          bg: 'bg-gradient-to-r from-amber-50 to-yellow-50',
          border: 'border-amber-300',
          text: 'text-amber-800',
          badge: 'bg-amber-500 text-amber-50',
          pulse: 'bg-amber-400'
        };
      case 'localization':
        return {
          bg: 'bg-gradient-to-r from-blue-50 to-indigo-50',
          border: 'border-blue-300',
          text: 'text-blue-800',
          badge: 'bg-blue-500 text-blue-50',
          pulse: 'bg-blue-400'
        };
      case 'idle':
        return {
          bg: 'bg-gradient-to-r from-slate-50 to-gray-50',
          border: 'border-slate-300',
          text: 'text-slate-700',
          badge: 'bg-slate-500 text-slate-50',
          pulse: 'bg-slate-400'
        };
      default:
        return {
          bg: 'bg-gradient-to-r from-gray-50 to-slate-50',
          border: 'border-gray-300',
          text: 'text-gray-700',
          badge: 'bg-gray-500 text-gray-50',
          pulse: 'bg-gray-400'
        };
    }
  };

  const modeStyle = getModeStyle();

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">
          Mapping & Mode
        </h2>
        <button
          onClick={refresh}
          disabled={operating || loading}
          className="px-3 py-1.5 text-sm rounded-lg bg-gradient-to-r from-blue-100 to-indigo-100 hover:from-blue-200 hover:to-indigo-200 text-blue-700 border-2 border-blue-300 font-medium disabled:opacity-40 disabled:cursor-not-allowed transition-all shadow-sm hover:shadow-md"
          title="Refresh mode and maps"
        >
          <span className={loading ? 'inline-block animate-spin' : ''}>↻</span> Refresh
        </button>
      </div>

      {/* Current Status Card */}
      <div className={`${modeStyle.bg} rounded-lg p-3 border-2 ${modeStyle.border} mb-3 shadow-sm transition-all duration-300`}>
        <div className="flex items-center justify-between mb-2">
          <span className={`text-base font-medium ${modeStyle.text}`}>Current Mode</span>
          <div className="flex items-center gap-2">
            <span className={`w-3 h-3 rounded-full ${modeStyle.pulse} ${mode ? 'animate-pulse' : 'opacity-40'}`}></span>
            <span className={`px-3 py-1 rounded-full text-sm font-bold shadow-sm ${modeStyle.badge}`}>
              {loading ? '...' : mode?.toUpperCase() || 'UNKNOWN'}
            </span>
          </div>
        </div>
        <div className="flex items-center justify-between pt-2 border-t-2 border-blue-200/50">
          <span className={`text-sm font-medium ${modeStyle.text}`}>Active Map</span>
          <span className="text-sm font-mono font-semibold text-blue-900 bg-white/60 px-2 py-0.5 rounded border border-blue-200">
            {loading ? '...' : activeMap || 'None'}
          </span>
        </div>
      </div>

      {/* Error/Status Messages */}
      {error && (
        <div className="bg-gradient-to-r from-red-50 to-rose-50 rounded-lg px-3 py-2 text-red-800 text-sm mb-2 shadow-sm animate-in fade-in slide-in-from-top-2 duration-300">
          <span className="font-semibold">⚠ Error:</span> {error}
        </div>
      )}
      {statusMsg && (
        <div className={`rounded-lg px-3 py-2 text-sm mb-2 shadow-sm animate-in fade-in slide-in-from-top-2 duration-300 ${
          statusMsg.startsWith('✓')
            ? 'bg-gradient-to-r from-green-50 to-emerald-50 text-green-800'
            : 'bg-gradient-to-r from-red-50 to-rose-50 text-red-800'
        }`}>
          <span className="font-semibold">{statusMsg}</span>
        </div>
      )}

      <div className="space-y-3">
        {/* Mode Controls */}
        <div>
          <h3 className="text-sm font-semibold text-blue-800 mb-2 uppercase tracking-wide">Mode Controls</h3>
          <div className="grid grid-cols-3 gap-2">
            <button
              onClick={handleStartMapping}
              disabled={operating || loading || mode === 'slam'}
              className="px-3 py-2.5 bg-gradient-to-r from-amber-500 to-yellow-600 hover:from-amber-600 hover:to-yellow-700 disabled:from-gray-200 disabled:to-gray-300 text-white disabled:text-gray-500 rounded-lg font-bold text-sm transition-all shadow-md hover:shadow-lg disabled:shadow-none transform hover:scale-105 disabled:scale-100 disabled:cursor-not-allowed"
              title="Start SLAM mapping mode"
            >
              SLAM
            </button>
            <button
              onClick={handleSetLocalization}
              disabled={operating || loading || mode === 'localization'}
              className="px-3 py-2.5 bg-gradient-to-r from-blue-500 to-indigo-600 hover:from-blue-600 hover:to-indigo-700 disabled:from-gray-200 disabled:to-gray-300 text-white disabled:text-gray-500 rounded-lg font-bold text-sm transition-all shadow-md hover:shadow-lg disabled:shadow-none transform hover:scale-105 disabled:scale-100 disabled:cursor-not-allowed"
              title="Switch to localization mode"
            >
              Localize
            </button>
            <button
              onClick={handleSetIdle}
              disabled={operating || loading || mode === 'idle'}
              className="px-3 py-2.5 bg-gradient-to-r from-slate-500 to-gray-600 hover:from-slate-600 hover:to-gray-700 disabled:from-gray-200 disabled:to-gray-300 text-white disabled:text-gray-500 rounded-lg font-bold text-sm transition-all shadow-md hover:shadow-lg disabled:shadow-none transform hover:scale-105 disabled:scale-100 disabled:cursor-not-allowed"
              title="Set robot to idle mode"
            >
              Idle
            </button>
          </div>
        </div>

        {/* Mapping Actions */}
        {mode === 'slam' && (
          <div className="bg-gradient-to-r from-amber-50 to-yellow-50 border-2 border-amber-300 rounded-lg p-3 shadow-sm animate-in fade-in slide-in-from-top-2 duration-300">
            <h3 className="text-sm font-semibold text-amber-800 mb-2 uppercase tracking-wide flex items-center gap-2">
              <span className="w-2 h-2 rounded-full bg-amber-500 animate-pulse"></span>
              Mapping Active
            </h3>
            <button
              onClick={handleStopAndSave}
              disabled={operating || loading}
              className="w-full px-4 py-2.5 bg-gradient-to-r from-green-500 to-emerald-600 hover:from-green-600 hover:to-emerald-700 disabled:from-gray-300 disabled:to-gray-400 text-white disabled:text-gray-500 rounded-lg font-bold transition-all shadow-md hover:shadow-lg disabled:shadow-none transform hover:scale-[1.02] disabled:scale-100 disabled:cursor-not-allowed"
              title="Stop mapping and save the map"
            >
              💾 Stop & Save Map
            </button>
          </div>
        )}

        {/* Map Loading */}
        <div className="bg-gradient-to-r from-blue-50 to-indigo-50 border-2 border-blue-300 rounded-lg p-3 shadow-sm">
          <h3 className="text-sm font-semibold text-blue-800 mb-2 uppercase tracking-wide">Load Map</h3>
          <select
            value={selectedMap}
            onChange={(e) => setSelectedMap(e.target.value)}
            disabled={operating || loading}
            className="w-full px-3 py-2 mb-2 bg-white text-blue-900 rounded-lg border-2 border-blue-300 focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 focus:outline-none disabled:bg-gray-100 disabled:text-gray-500 disabled:border-gray-300 font-medium transition-all shadow-sm"
          >
            <option value="">Select a map...</option>
            {maps.map(m => (
              <option key={m} value={m}>{m}</option>
            ))}
          </select>
          <button
            onClick={handleLoadMap}
            disabled={operating || loading || !selectedMap || mode === 'slam'}
            className="w-full px-4 py-2.5 bg-gradient-to-r from-blue-500 to-indigo-600 hover:from-blue-600 hover:to-indigo-700 disabled:from-gray-200 disabled:to-gray-300 text-white disabled:text-gray-500 rounded-lg font-bold transition-all shadow-md hover:shadow-lg disabled:shadow-none transform hover:scale-[1.02] disabled:scale-100 disabled:cursor-not-allowed"
            title="Load the selected map"
          >
            📍 Load Selected Map
          </button>
        </div>

        {/* Available Maps List */}
        {maps.length > 0 && (
          <div className="bg-gradient-to-r from-slate-50 to-gray-50 border-2 border-slate-300 rounded-lg p-3 shadow-sm">
            <h3 className="text-xs font-bold text-slate-700 mb-2 uppercase tracking-wider flex items-center justify-between">
              <span>Available Maps</span>
              <span className="px-2 py-0.5 bg-slate-200 text-slate-700 rounded-full text-xs font-bold">{maps.length}</span>
            </h3>
            <div className="max-h-32 overflow-y-auto space-y-1.5 pr-1 scrollbar-thin scrollbar-thumb-blue-300 scrollbar-track-blue-50">
              {maps.map(m => (
                <div
                  key={m}
                  className={`px-3 py-1.5 rounded-lg text-xs font-mono font-medium transition-all ${
                    m === activeMap
                      ? 'bg-gradient-to-r from-blue-100 to-indigo-100 text-blue-900 border-2 border-blue-400 shadow-sm'
                      : 'bg-white text-slate-700 border border-slate-300 hover:border-blue-300 hover:bg-blue-50'
                  }`}
                >
                  <span className={m === activeMap ? 'font-bold' : ''}>{m}</span>
                  {m === activeMap && <span className="ml-2 text-blue-600 font-bold">← Active</span>}
                </div>
              ))}
            </div>
          </div>
        )}
      </div>
    </section>
  );
}
