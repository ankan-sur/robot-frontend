/**
 * Demo Day UI - AppDemo.tsx
 * 
 * LAN-only POI navigation interface for Demo Day.
 * Features:
 * - Map AND Camera visible together (split view)
 * - POI navigation buttons
 * - Teleop controller
 * - Debug logs
 * - Telemetry and battery status
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { DEMO_CONFIG } from './config';
import { DEMO_POIS, POI } from './pois';

// Import the real components and hooks
import { MapView } from '../components/MapView';
import VideoFeed from '../components/VideoFeed';
import { TeleopBlock } from '../components/TeleopBlock';
import { DebugLog } from '../components/DebugLog';
import { useRosConnection, useRobotPose, useBattery } from '../ros/hooks';
import { ros } from '../ros/ros';

// Types
interface LogEntry {
  id: number;
  timestamp: Date;
  message: string;
  type: 'info' | 'success' | 'error' | 'nav';
}

type NavState = 'idle' | 'navigating' | 'succeeded' | 'failed';

// Helpers
function quaternionFromYaw(yaw: number): { x: number; y: number; z: number; w: number } {
  const halfYaw = yaw / 2;
  return {
    x: 0,
    y: 0,
    z: Math.sin(halfYaw),
    w: Math.cos(halfYaw),
  };
}

// Main Demo App Component
export default function AppDemo() {
  const { connected } = useRosConnection();
  const robotPose = useRobotPose();
  const battery = useBattery();
  
  // Extract position from pose
  const posX = robotPose?.pose?.pose?.position?.x ?? robotPose?.pose?.position?.x ?? 0;
  const posY = robotPose?.pose?.pose?.position?.y ?? robotPose?.pose?.position?.y ?? 0;
  // Extract yaw from quaternion
  const q = robotPose?.pose?.pose?.orientation ?? robotPose?.pose?.orientation;
  const theta = q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : 0;

  // Navigation state
  const [navState, setNavState] = useState<NavState>('idle');
  const [currentPOI, setCurrentPOI] = useState<POI | null>(null);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [statusMsg, setStatusMsg] = useState('');
  
  // Action client ref
  const actionClientRef = useRef<any>(null);
  const goalHandleRef = useRef<any>(null);
  const logIdRef = useRef(0);

  // Add log entry
  const addLog = useCallback((message: string, type: LogEntry['type'] = 'info') => {
    const entry: LogEntry = {
      id: logIdRef.current++,
      timestamp: new Date(),
      message,
      type,
    };
    setLogs(prev => [entry, ...prev].slice(0, 50));
  }, []);

  // Clear status after delay
  const clearStatus = useCallback(() => {
    setTimeout(() => setStatusMsg(''), 4000);
  }, []);

  // Initialize action client when connected
  useEffect(() => {
    if (!ros || !connected) return;

    const ROSLIB = (window as any).ROSLIB;
    if (!ROSLIB) {
      console.warn('ROSLIB not available');
      return;
    }

    actionClientRef.current = new ROSLIB.ActionClient({
      ros,
      serverName: '/navigate_to_pose',
      actionName: 'nav2_msgs/action/NavigateToPose',
    });

    addLog('Connected to Nav2 action server', 'success');

    return () => {
      if (goalHandleRef.current) {
        goalHandleRef.current.cancel();
      }
    };
  }, [ros, connected, addLog]);

  // Navigate to POI
  const handleNavigateTo = useCallback(async (poi: POI) => {
    if (!actionClientRef.current || !connected) {
      setStatusMsg('Not connected');
      clearStatus();
      return;
    }

    if (goalHandleRef.current) {
      goalHandleRef.current.cancel();
    }

    setCurrentPOI(poi);
    setNavState('navigating');
    setStatusMsg('Navigating to ' + poi.label + '...');
    addLog('Starting navigation to ' + poi.label + ' (' + poi.x.toFixed(2) + ', ' + poi.y.toFixed(2) + ')', 'nav');

    const ROSLIB = (window as any).ROSLIB;
    
    const goal = new ROSLIB.Goal({
      actionClient: actionClientRef.current,
      goalMessage: {
        pose: {
          header: {
            frame_id: 'map',
            stamp: { sec: 0, nanosec: 0 },
          },
          pose: {
            position: { x: poi.x, y: poi.y, z: 0 },
            orientation: quaternionFromYaw(poi.theta),
          },
        },
      },
    });

    goalHandleRef.current = goal;

    goal.on('feedback', (feedback: any) => {
      const remaining = feedback.distance_remaining?.toFixed(2) || '?';
      addLog('Distance remaining: ' + remaining + 'm', 'info');
    });

    goal.on('result', (result: any) => {
      if (result) {
        setNavState('succeeded');
        setStatusMsg('Arrived at ' + poi.label + '!');
        addLog('Successfully arrived at ' + poi.label, 'success');
      } else {
        setNavState('failed');
        setStatusMsg('Navigation failed');
        addLog('Navigation to ' + poi.label + ' failed', 'error');
      }
      setCurrentPOI(null);
      goalHandleRef.current = null;
      clearStatus();
    });

    goal.on('status', (status: any) => {
      if (status.status === 4) {
        setNavState('succeeded');
      } else if (status.status >= 3 && status.status !== 4) {
        setNavState('failed');
      }
    });

    goal.send();
  }, [connected, addLog, clearStatus]);

  // Cancel navigation
  const handleCancel = useCallback(() => {
    if (goalHandleRef.current) {
      goalHandleRef.current.cancel();
      goalHandleRef.current = null;
    }
    setNavState('idle');
    setCurrentPOI(null);
    setStatusMsg('Navigation cancelled');
    addLog('Navigation cancelled by user', 'info');
    clearStatus();
  }, [addLog, clearStatus]);

  const isNavigating = navState === 'navigating';

  return (
    <div className="min-h-screen bg-slate-900 text-white">
      {/* Header */}
      <div className="bg-emerald-900 border-b border-emerald-700 px-4 py-3">
        <div className="max-w-7xl mx-auto">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-xl font-bold text-emerald-100">
                {DEMO_CONFIG.robotName} - Demo Day
              </h1>
              <div className="flex items-center gap-3 text-sm mt-1">
                <div className="flex items-center gap-1.5">
                  <div className={'w-2 h-2 rounded-full ' + (connected ? 'bg-green-400' : 'bg-red-400')} />
                  <span className={connected ? 'text-green-400' : 'text-red-400'}>
                    {connected ? 'Connected' : 'Disconnected'}
                  </span>
                </div>
                <span className="text-emerald-600">|</span>
                <span className="text-emerald-300">Map: {DEMO_CONFIG.mapName}</span>
                {battery?.percent !== undefined && battery.percent !== null && (
                  <>
                    <span className="text-emerald-600">|</span>
                    <span className={(battery.percent ?? 0) < 20 ? 'text-red-400' : (battery.percent ?? 0) < 40 ? 'text-yellow-400' : 'text-green-400'}>
                      Battery: {Math.round(battery.percent ?? 0)}%
                    </span>
                  </>
                )}
              </div>
            </div>
            <div className="text-right">
              <div className={'text-lg font-bold ' + (isNavigating ? 'text-amber-400' : 'text-emerald-400')}>
                {isNavigating ? 'Going to ' + currentPOI?.label : 'Ready'}
              </div>
              <div className="text-xs text-emerald-400/70">
                ({posX.toFixed(2)}, {posY.toFixed(2)})
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Status Message */}
      {statusMsg && (
        <div className="max-w-7xl mx-auto px-4 mt-3">
          <div className="p-3 rounded-lg text-center font-semibold bg-emerald-900/50 text-emerald-300 border border-emerald-700">
            {statusMsg}
          </div>
        </div>
      )}

      <main className="max-w-7xl mx-auto p-4">
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-4">
          
          {/* Left Column: Map + Camera (both visible) */}
          <div className="lg:col-span-2 space-y-4">
            {/* Map and Camera side by side */}
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              {/* Map */}
              <div className="bg-slate-800 rounded-lg border border-slate-700 overflow-hidden">
                <div className="px-4 py-2 border-b border-slate-700 bg-slate-800/50">
                  <span className="text-sm font-medium text-slate-300">Map View</span>
                </div>
                <div className="p-2">
                  <MapView embedded mode="localization" />
                </div>
              </div>
              
              {/* Camera */}
              <div className="bg-slate-800 rounded-lg border border-slate-700 overflow-hidden">
                <div className="px-4 py-2 border-b border-slate-700 bg-slate-800/50">
                  <span className="text-sm font-medium text-slate-300">Camera</span>
                </div>
                <div className="p-2">
                  <VideoFeed embedded />
                </div>
              </div>
            </div>

            {/* POI Navigation Buttons */}
            <div className="bg-slate-800 rounded-lg border border-emerald-700 p-4">
              <div className="text-sm font-semibold text-emerald-400 mb-3">
                Navigate to Point of Interest
              </div>
              <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
                {DEMO_POIS.map((poi) => (
                  <button
                    key={poi.id}
                    onClick={() => handleNavigateTo(poi)}
                    disabled={!connected || isNavigating}
                    className={
                      'flex flex-col items-center justify-center p-4 rounded-lg font-semibold transition-all ' +
                      (currentPOI?.id === poi.id 
                        ? 'bg-emerald-600 text-white ring-2 ring-emerald-400 ring-offset-2 ring-offset-slate-800' 
                        : 'bg-slate-700 hover:bg-emerald-700 text-slate-200 border border-slate-600 hover:border-emerald-500') +
                      ((!connected || isNavigating) && currentPOI?.id !== poi.id ? ' opacity-50 cursor-not-allowed' : '')
                    }
                  >
                    <span className="text-2xl mb-1">{poi.icon}</span>
                    <span className="text-sm">{poi.label}</span>
                  </button>
                ))}
              </div>
              
              {/* Cancel Button */}
              {isNavigating && (
                <div className="mt-4 flex justify-center">
                  <button
                    onClick={handleCancel}
                    className="px-6 py-2 bg-red-600 hover:bg-red-500 text-white rounded-lg font-semibold transition-colors"
                  >
                    Cancel Navigation
                  </button>
                </div>
              )}
            </div>

            {/* Debug Log */}
            <DebugLog />
          </div>

          {/* Right Column: Teleop + Telemetry */}
          <div className="space-y-4">
            {/* Teleop Control */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <div className="text-sm font-semibold text-slate-300 mb-3">Manual Control</div>
              <TeleopBlock />
            </div>

            {/* Telemetry Panel */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <div className="text-sm font-semibold mb-3 text-slate-300">Telemetry</div>
              <div className="space-y-2 text-sm font-mono">
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Status:</span>
                  <span className={isNavigating ? 'text-amber-400' : 'text-emerald-400'}>
                    {isNavigating ? 'NAVIGATING' : 'READY'}
                  </span>
                </div>
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Connection:</span>
                  <span className={connected ? 'text-green-400' : 'text-red-400'}>
                    {connected ? 'Connected' : 'Offline'}
                  </span>
                </div>
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Position:</span>
                  <span className="text-white">
                    ({posX.toFixed(2)}, {posY.toFixed(2)})
                  </span>
                </div>
                <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                  <span className="text-slate-400">Heading:</span>
                  <span className="text-white">
                    {(theta * 180 / Math.PI).toFixed(1)} deg
                  </span>
                </div>
                {battery?.percent !== undefined && battery.percent !== null && (
                  <div className="flex justify-between p-2 bg-slate-700/50 rounded">
                    <span className="text-slate-400">Battery:</span>
                    <span className={(battery.percent ?? 0) < 20 ? 'text-red-400' : (battery.percent ?? 0) < 40 ? 'text-yellow-400' : 'text-green-400'}>
                      {Math.round(battery.percent ?? 0)}% {battery.volts ? '(' + battery.volts.toFixed(1) + 'V)' : ''}
                    </span>
                  </div>
                )}
                {currentPOI && (
                  <div className="flex justify-between p-2 bg-emerald-900/30 rounded border border-emerald-700">
                    <span className="text-emerald-400">Target:</span>
                    <span className="text-emerald-300">
                      {currentPOI.icon} {currentPOI.label}
                    </span>
                  </div>
                )}
              </div>
            </div>

            {/* Activity Log (compact) */}
            <div className="bg-slate-800 rounded-lg border border-slate-700 p-4">
              <div className="text-sm font-semibold mb-2 text-slate-300">Activity</div>
              <div className="space-y-1 max-h-40 overflow-y-auto text-xs font-mono">
                {logs.slice(0, 10).map((log) => (
                  <div
                    key={log.id}
                    className={'p-1.5 rounded ' + (
                      log.type === 'success' ? 'bg-green-900/30 text-green-400' :
                      log.type === 'error' ? 'bg-red-900/30 text-red-400' :
                      log.type === 'nav' ? 'bg-emerald-900/30 text-emerald-400' :
                      'bg-slate-700/30 text-slate-400'
                    )}
                  >
                    <span className="opacity-60">
                      {log.timestamp.toLocaleTimeString()}
                    </span>
                    {' '}{log.message}
                  </div>
                ))}
                {logs.length === 0 && (
                  <div className="text-slate-500 text-center py-2">No activity yet</div>
                )}
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
}
