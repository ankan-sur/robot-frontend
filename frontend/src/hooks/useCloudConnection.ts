/**
 * Cloud Mode Hook
 * 
 * React hook for using cloud WebSocket connection instead of direct rosbridge.
 * Provides the same interface as the existing ROS hooks for easy migration.
 */

import { useState, useEffect, useCallback } from 'react';
import { getCloudClient, CloudTelemetry, CloudCommandResult } from '../ros/cloudClient';

export interface CloudRobotState {
  connected: boolean;
  robotId: string;
  pose: { x: number; y: number; z: number; theta: number } | null;
  battery: number | null;
  batteryVoltage: number | null;
  state: string;
  map: string | null;
  availableMaps: string[];
  navActive: boolean;
  connectedClients: number;
  lastUpdate: Date | null;
}

export function useCloudConnection() {
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [robotState, setRobotState] = useState<CloudRobotState>({
    connected: false,
    robotId: 'mentorpi',
    pose: null,
    battery: null,
    batteryVoltage: null,
    state: 'UNKNOWN',
    map: null,
    availableMaps: [],
    navActive: false,
    connectedClients: 0,
    lastUpdate: null,
  });

  useEffect(() => {
    const client = getCloudClient();

    // Connection events
    client.on('connection', () => {
      console.log('[Cloud Hook] Connected');
      setConnected(true);
      setError(null);
    });

    client.on('close', () => {
      console.log('[Cloud Hook] Disconnected');
      setConnected(false);
    });

    client.on('error', (err: any) => {
      console.error('[Cloud Hook] Error:', err);
      setError(err.error || 'Connection error');
    });

    // Telemetry updates
    client.on('telemetry', (data: CloudTelemetry) => {
      setRobotState(prev => ({
        ...prev,
        connected: true,
        robotId: data.robot_id,
        pose: data.pose || prev.pose,
        battery: data.battery ?? prev.battery,
        batteryVoltage: data.battery_voltage ?? prev.batteryVoltage,
        state: data.state,
        map: data.map || prev.map,
        availableMaps: data.available_maps || prev.availableMaps,
        navActive: data.nav_active ?? prev.navActive,
        connectedClients: data.connected_clients ?? prev.connectedClients,
        lastUpdate: new Date(data.timestamp),
      }));
    });

    // Initial connection
    client.connect().catch((err: any) => {
      console.error('[Cloud Hook] Failed to connect:', err);
      setError('Failed to connect to cloud backend');
    });

    return () => {
      client.disconnect();
    };
  }, []);

  const sendCommand = useCallback((command: string, params?: Record<string, any>) => {
    const client = getCloudClient();
    client.sendCommand(command, params);
  }, []);

  const startSlam = useCallback(() => {
    getCloudClient().startSlam();
  }, []);

  const stopSlam = useCallback((mapName: string) => {
    getCloudClient().stopSlam(mapName);
  }, []);

  const loadMap = useCallback((mapName: string) => {
    getCloudClient().loadMap(mapName);
  }, []);

  const setMode = useCallback((mode: 'slam' | 'localization' | 'idle') => {
    getCloudClient().setMode(mode);
  }, []);

  const cancelNavigation = useCallback(() => {
    getCloudClient().cancelNavigation();
  }, []);

  const emergencyStop = useCallback(() => {
    getCloudClient().emergencyStop();
  }, []);

  const goToPoi = useCallback((poiId: string) => {
    getCloudClient().goToPoi(poiId);
  }, []);

  const restart = useCallback(() => {
    getCloudClient().restart();
  }, []);

  return {
    connected,
    error,
    robotState,
    sendCommand,
    startSlam,
    stopSlam,
    loadMap,
    setMode,
    cancelNavigation,
    emergencyStop,
    goToPoi,
    restart,
  };
}

// Command result listener hook
export function useCommandResults(callback: (result: CloudCommandResult) => void) {
  useEffect(() => {
    const client = getCloudClient();
    
    client.on('command_result', callback);
    
    return () => {
      client.off('command_result', callback);
    };
  }, [callback]);
}
