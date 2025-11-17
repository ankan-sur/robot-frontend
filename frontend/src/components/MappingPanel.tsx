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

  return (
    <div className="bg-gray-900 rounded-lg p-4 space-y-4">
      <div className="flex items-center justify-between">
        <h2 className="text-lg font-bold text-white">Mapping & Mode</h2>
        <button
          onClick={refresh}
          disabled={operating || loading}
          className="text-sm px-2 py-1 bg-gray-700 hover:bg-gray-600 disabled:bg-gray-800 disabled:text-gray-500 rounded transition-colors"
        >
          ↻ Refresh
        </button>
      </div>

      {/* Status Display */}
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <span className="text-gray-400 text-sm">Current Mode:</span>
          <span className={`px-3 py-1 rounded-full text-sm font-semibold ${
            mode === 'slam' ? 'bg-yellow-600 text-yellow-100' :
            mode === 'localization' ? 'bg-blue-600 text-blue-100' :
            mode === 'idle' ? 'bg-gray-600 text-gray-200' :
            'bg-gray-700 text-gray-300'
          }`}>
            {loading ? '...' : mode?.toUpperCase() || 'UNKNOWN'}
          </span>
        </div>
        
        <div className="flex items-center justify-between">
          <span className="text-gray-400 text-sm">Active Map:</span>
          <span className="text-white text-sm font-mono">
            {loading ? '...' : activeMap || '—'}
          </span>
        </div>
      </div>

      {/* Error/Status Messages */}
      {error && (
        <div className="bg-red-900/30 border border-red-700 rounded px-3 py-2 text-red-200 text-sm">
          {error}
        </div>
      )}
      {statusMsg && (
        <div className={`rounded px-3 py-2 text-sm ${
          statusMsg.startsWith('✓') 
            ? 'bg-green-900/30 border border-green-700 text-green-200'
            : 'bg-red-900/30 border border-red-700 text-red-200'
        }`}>
          {statusMsg}
        </div>
      )}

      {/* Mode Controls */}
      <div className="space-y-2">
        <h3 className="text-sm font-semibold text-gray-300 uppercase tracking-wide">Mode Controls</h3>
        <div className="grid grid-cols-3 gap-2">
          <button
            onClick={handleStartMapping}
            disabled={operating || loading || mode === 'slam'}
            className="px-3 py-2 bg-yellow-600 hover:bg-yellow-700 disabled:bg-gray-700 disabled:text-gray-500 text-white rounded font-semibold text-sm transition-colors"
          >
            SLAM
          </button>
          <button
            onClick={handleSetLocalization}
            disabled={operating || loading || mode === 'localization'}
            className="px-3 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:text-gray-500 text-white rounded font-semibold text-sm transition-colors"
          >
            Localize
          </button>
          <button
            onClick={handleSetIdle}
            disabled={operating || loading || mode === 'idle'}
            className="px-3 py-2 bg-gray-600 hover:bg-gray-500 disabled:bg-gray-700 disabled:text-gray-500 text-white rounded font-semibold text-sm transition-colors"
          >
            Idle
          </button>
        </div>
      </div>

      {/* Mapping Actions */}
      <div className="space-y-2">
        <h3 className="text-sm font-semibold text-gray-300 uppercase tracking-wide">Mapping Actions</h3>
        <button
          onClick={handleStopAndSave}
          disabled={operating || loading || mode !== 'slam'}
          className="w-full px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:text-gray-500 text-white rounded font-semibold transition-colors"
        >
          Stop & Save Map
        </button>
      </div>

      {/* Map Loading */}
      <div className="space-y-2">
        <h3 className="text-sm font-semibold text-gray-300 uppercase tracking-wide">Load Map</h3>
        <select
          value={selectedMap}
          onChange={(e) => setSelectedMap(e.target.value)}
          disabled={operating || loading}
          className="w-full px-3 py-2 bg-gray-800 text-white rounded border border-gray-700 focus:border-blue-500 focus:outline-none disabled:bg-gray-700 disabled:text-gray-500"
        >
          <option value="">Select a map...</option>
          {maps.map(m => (
            <option key={m} value={m}>{m}</option>
          ))}
        </select>
        <button
          onClick={handleLoadMap}
          disabled={operating || loading || !selectedMap || mode === 'slam'}
          className="w-full px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:text-gray-500 text-white rounded font-semibold transition-colors"
        >
          Load Selected Map
        </button>
      </div>

      {/* Available Maps List */}
      {maps.length > 0 && (
        <div className="space-y-1">
          <h3 className="text-xs font-semibold text-gray-400 uppercase tracking-wide">Available Maps ({maps.length})</h3>
          <div className="max-h-32 overflow-y-auto space-y-1">
            {maps.map(m => (
              <div
                key={m}
                className={`px-2 py-1 rounded text-xs font-mono ${
                  m === activeMap
                    ? 'bg-blue-900/40 text-blue-200 border border-blue-700'
                    : 'bg-gray-800 text-gray-300'
                }`}
              >
                {m} {m === activeMap && '← active'}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
