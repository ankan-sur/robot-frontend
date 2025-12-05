/**
 * Demo Day UI - Navigation Mode Only
 * 
 * Features:
 * - Map dropdown to load maps
 * - POI dropdown for navigation
 * - Camera feed
 * - Teleop controller
 * - Telemetry and battery bar
 * - Blue/white theme
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { DEMO_CONFIG } from './config';
import { DEMO_POIS, POI } from './pois';

// Import components and hooks
import { MapView } from '../components/MapView';
import VideoFeed from '../components/VideoFeed';
import { TeleopBlock } from '../components/TeleopBlock';
import { DebugLog } from '../components/DebugLog';
import { useRosConnection, useRobotPose, useBattery, useModeAndMaps } from '../ros/hooks';
import { ros } from '../ros/ros';
import { loadMap } from '../ros/services';

// Battery Bar Component
function BatteryBar({ percent, volts }: { percent: number; volts?: number }) {
  const fillWidth = Math.max(0, Math.min(100, percent));
  const isLow = percent < 20;
  const isMed = percent < 40;
  
  return (
    <div className="flex items-center gap-2">
      <div className="relative w-20 h-6 bg-slate-700 rounded border border-slate-500 overflow-hidden">
        <div 
          className={'h-full transition-all duration-300 ' + (isLow ? 'bg-red-500' : isMed ? 'bg-yellow-500' : 'bg-green-500')}
          style={{ width: fillWidth + '%' }}
        />
        <div className="absolute inset-0 flex items-center justify-center text-xs font-bold text-white drop-shadow">
          {Math.round(percent)}%
        </div>
      </div>
      {volts && <span className="text-xs text-slate-400">{volts.toFixed(1)}V</span>}
    </div>
  );
}

// Helpers
function quaternionFromYaw(yaw: number): { x: number; y: number; z: number; w: number } {
  const halfYaw = yaw / 2;
  return { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) };
}

// Main Demo App Component
export default function AppDemo() {
  const { connected } = useRosConnection();
  const robotPose = useRobotPose();
  const battery = useBattery();
  const { maps, loading: mapsLoading, refresh: refreshMaps } = useModeAndMaps();
  
  // Extract position from pose
  const posX = robotPose?.pose?.pose?.position?.x ?? robotPose?.pose?.position?.x ?? 0;
  const posY = robotPose?.pose?.pose?.position?.y ?? robotPose?.pose?.position?.y ?? 0;
  const q = robotPose?.pose?.pose?.orientation ?? robotPose?.pose?.orientation;
  const theta = q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : 0;

  // State
  const [selectedMap, setSelectedMap] = useState<string>(DEMO_CONFIG.mapName);
  const [selectedPOI, setSelectedPOI] = useState<string>('');
  const [isNavigating, setIsNavigating] = useState(false);
  const [currentPOI, setCurrentPOI] = useState<POI | null>(null);
  const [statusMsg, setStatusMsg] = useState('');
  const [mapLoading, setMapLoading] = useState(false);
  
  const actionClientRef = useRef<any>(null);
  const goalHandleRef = useRef<any>(null);

  // Clear status after delay
  const clearStatus = useCallback(() => {
    setTimeout(() => setStatusMsg(''), 4000);
  }, []);

  // Load maps on mount
  useEffect(() => {
    if (connected) {
      refreshMaps();
    }
  }, [connected, refreshMaps]);

  // Initialize action client when connected
  useEffect(() => {
    if (!ros || !connected) return;

    const ROSLIB = (window as any).ROSLIB;
    if (!ROSLIB) return;

    actionClientRef.current = new ROSLIB.ActionClient({
      ros,
      serverName: '/navigate_to_pose',
      actionName: 'nav2_msgs/action/NavigateToPose',
    });

    return () => {
      if (goalHandleRef.current) {
        goalHandleRef.current.cancel();
      }
    };
  }, [connected]);

  // Handle map load
  const handleLoadMap = async () => {
    if (!selectedMap || mapLoading) return;
    
    setMapLoading(true);
    setStatusMsg('Loading map: ' + selectedMap + '...');
    
    try {
      await loadMap(selectedMap);
      setStatusMsg('Map loaded: ' + selectedMap);
    } catch (e: any) {
      setStatusMsg('Failed to load map: ' + (e?.message || 'Unknown error'));
    } finally {
      setMapLoading(false);
      clearStatus();
    }
  };

  // Handle POI navigation
  const handleNavigate = useCallback(() => {
    if (!selectedPOI || !actionClientRef.current || !connected) return;
    
    const poi = DEMO_POIS.find(p => p.id === selectedPOI);
    if (!poi) return;

    // Cancel any current goal
    if (goalHandleRef.current) {
      goalHandleRef.current.cancel();
    }

    setCurrentPOI(poi);
    setIsNavigating(true);
    setStatusMsg('Navigating to ' + poi.label + '...');

    const ROSLIB = (window as any).ROSLIB;
    
    const goal = new ROSLIB.Goal({
      actionClient: actionClientRef.current,
      goalMessage: {
        pose: {
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x: poi.x, y: poi.y, z: 0 },
            orientation: quaternionFromYaw(poi.theta),
          },
        },
      },
    });

    goalHandleRef.current = goal;

    goal.on('result', (result: any) => {
      if (result) {
        setStatusMsg('Arrived at ' + poi.label);
      } else {
        setStatusMsg('Navigation failed');
      }
      setIsNavigating(false);
      setCurrentPOI(null);
      goalHandleRef.current = null;
      clearStatus();
    });

    goal.on('status', (status: any) => {
      if (status.status === 4) {
        setStatusMsg('Arrived at ' + poi.label);
        setIsNavigating(false);
        setCurrentPOI(null);
      } else if (status.status >= 3 && status.status !== 4) {
        setStatusMsg('Navigation cancelled or failed');
        setIsNavigating(false);
        setCurrentPOI(null);
      }
    });

    goal.send();
  }, [selectedPOI, connected, clearStatus]);

  // Cancel navigation
  const handleCancel = useCallback(() => {
    if (goalHandleRef.current) {
      goalHandleRef.current.cancel();
      goalHandleRef.current = null;
    }
    setIsNavigating(false);
    setCurrentPOI(null);
    setStatusMsg('Navigation cancelled');
    clearStatus();
  }, [clearStatus]);

  const batteryPercent = battery?.percent ?? 0;
  const batteryVolts = battery?.volts ?? undefined;

  return (
    <div className="min-h-screen bg-slate-900 text-white">
      {/* Header */}
      <div className="bg-slate-800 border-b border-slate-600 px-4 py-3">
        <div className="max-w-7xl mx-auto">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-xl font-bold text-white">
                {DEMO_CONFIG.robotName} - Navigation
              </h1>
              <div className="flex items-center gap-3 text-sm mt-1">
                <div className="flex items-center gap-1.5">
                  <div className={'w-2 h-2 rounded-full ' + (connected ? 'bg-blue-400' : 'bg-red-400')} />
                  <span className={connected ? 'text-blue-400' : 'text-red-400'}>
                    {connected ? 'Connected' : 'Disconnected'}
                  </span>
                </div>
                <span className="text-slate-500">|</span>
                <span className="text-slate-300">Map: {selectedMap || 'None'}</span>
              </div>
            </div>
            <div className="flex items-center gap-4">
              <BatteryBar percent={batteryPercent} volts={batteryVolts} />
              <div className="text-right">
                <div className={'text-lg font-bold ' + (isNavigating ? 'text-blue-400' : 'text-white')}>
                  {isNavigating ? 'Going to ' + currentPOI?.label : 'Ready'}
                </div>
                <div className="text-xs text-slate-400">
                  ({posX.toFixed(2)}, {posY.toFixed(2)})
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Status Message */}
      {statusMsg && (
        <div className="max-w-7xl mx-auto px-4 mt-3">
          <div className="p-3 rounded-lg text-center font-semibold bg-blue-900/50 text-blue-300 border border-blue-700">
            {statusMsg}
          </div>
        </div>
      )}

      <main className="max-w-7xl mx-auto p-4">
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
          
          {/* Left Column: Map + Controls */}
          <div className="lg:col-span-2 space-y-4">
            {/* Map Selection + POI Navigation */}
            <div className="bg-slate-800 rounded-lg border border-slate-600 p-4">
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                {/* Map Dropdown */}
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-2">
                    Select Map
                  </label>
                  <div className="flex gap-2">
                    <select
                      value={selectedMap}
                      onChange={(e) => setSelectedMap(e.target.value)}
                      disabled={mapLoading}
                      className="flex-1 px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:outline-none focus:border-blue-500"
                    >
                      <option value="">-- Select Map --</option>
                      {maps.map((m) => (
                        <option key={m} value={m}>{m}</option>
                      ))}
                    </select>
                    <button
                      onClick={handleLoadMap}
                      disabled={!selectedMap || mapLoading || !connected}
                      className="px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-700 disabled:text-slate-500 text-white rounded-lg font-semibold transition-colors"
                    >
                      {mapLoading ? '...' : 'Load'}
                    </button>
                  </div>
                </div>

                {/* POI Dropdown */}
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-2">
                    Navigate to POI
                  </label>
                  <div className="flex gap-2">
                    <select
                      value={selectedPOI}
                      onChange={(e) => setSelectedPOI(e.target.value)}
                      disabled={isNavigating || !connected}
                      className="flex-1 px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:outline-none focus:border-blue-500"
                    >
                      <option value="">-- Select POI --</option>
                      {DEMO_POIS.map((poi) => (
                        <option key={poi.id} value={poi.id}>
                          {poi.icon} {poi.label}
                        </option>
                      ))}
                    </select>
                    {isNavigating ? (
                      <button
                        onClick={handleCancel}
                        className="px-4 py-2 bg-red-600 hover:bg-red-500 text-white rounded-lg font-semibold transition-colors"
                      >
                        Cancel
                      </button>
                    ) : (
                      <button
                        onClick={handleNavigate}
                        disabled={!selectedPOI || !connected}
                        className="px-4 py-2 bg-blue-600 hover:bg-blue-500 disabled:bg-slate-700 disabled:text-slate-500 text-white rounded-lg font-semibold transition-colors"
                      >
                        Go
                      </button>
                    )}
                  </div>
                </div>
              </div>
            </div>

            {/* Map View */}
            <div className="bg-slate-800 rounded-lg border border-slate-600 overflow-hidden">
              <div className="px-4 py-2 border-b border-slate-600 bg-slate-800/50">
                <span className="text-sm font-medium text-slate-300">Map View</span>
              </div>
              <div className="p-2">
                <MapView embedded mode="localization" />
              </div>
            </div>

            {/* Debug Log */}
            <DebugLog />
          </div>

          {/* Right Column: Camera + Teleop + Telemetry */}
          <div className="space-y-4">
            {/* Camera */}
            <div className="bg-slate-800 rounded-lg border border-slate-600 overflow-hidden">
              <div className="px-4 py-2 border-b border-slate-600 bg-slate-800/50">
                <span className="text-sm font-medium text-slate-300">Camera</span>
              </div>
              <div className="p-2">
                <VideoFeed embedded />
              </div>
            </div>

            {/* Teleop Control */}
            <div className="bg-slate-800 rounded-lg border border-slate-600 p-4">
              <div className="text-sm font-semibold text-slate-300 mb-3">Manual Control</div>
              <TeleopBlock />
            </div>

            {/* Telemetry Panel */}
            <div className="bg-slate-800 rounded-lg border border-slate-600 p-4">
              <div className="text-sm font-semibold mb-3 text-slate-300">Telemetry</div>
              <div className="space-y-2 text-sm font-mono">
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Status:</span>
                  <span className={isNavigating ? 'text-blue-400' : 'text-white'}>
                    {isNavigating ? 'NAVIGATING' : 'READY'}
                  </span>
                </div>
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Connection:</span>
                  <span className={connected ? 'text-blue-400' : 'text-red-400'}>
                    {connected ? 'Connected' : 'Offline'}
                  </span>
                </div>
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Position:</span>
                  <span className="text-white">({posX.toFixed(2)}, {posY.toFixed(2)})</span>
                </div>
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Heading:</span>
                  <span className="text-white">{(theta * 180 / Math.PI).toFixed(1)} deg</span>
                </div>
                {currentPOI && (
                  <div className="flex justify-between p-2 bg-blue-900/30 rounded border border-blue-700">
                    <span className="text-blue-400">Target:</span>
                    <span className="text-blue-300">{currentPOI.icon} {currentPOI.label}</span>
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
}
