/**
 * Cloud Mode Hooks
 * 
 * React hooks for cloud mode functionality - backend connection status,
 * UI client count, robot state, etc.
 */

import { useState, useEffect, useCallback } from 'react';
import { getCloudClient, RobotState } from './cloudClient';
import { isCloudModeEnabled } from './ros';

export interface CloudStatus {
  backendConnected: boolean;
  robotOnline: boolean;
  uiClientCount: number;
  controlHolder: string | null;
  hasControl: boolean;
  lastCommand: string | null;
  robotState: RobotState | null;
}

/**
 * Hook to get cloud backend connection status and related info
 */
export function useCloudStatus(): CloudStatus {
  const [status, setStatus] = useState<CloudStatus>({
    backendConnected: false,
    robotOnline: false,
    uiClientCount: 0,
    controlHolder: null,
    hasControl: false,
    lastCommand: null,
    robotState: null,
  });

  useEffect(() => {
    if (!isCloudModeEnabled()) {
      return;
    }

    const client = getCloudClient();

    const updateStatus = () => {
      setStatus({
        backendConnected: client.isConnected(),
        robotOnline: client.isRobotOnline(),
        uiClientCount: (client as any)._uiClientCount ?? 0,
        controlHolder: client.getControlHolder(),
        hasControl: client.hasControl(),
        lastCommand: (client as any)._lastCommand ?? null,
        robotState: client.getRobotState(),
      });
    };

    // Initial status
    updateStatus();

    // Listen for events
    const onConnection = () => updateStatus();
    const onClose = () => updateStatus();
    const onState = (msg: any) => {
      // Update UI client count from state message if available
      if (msg.uiClientCount !== undefined) {
        (client as any)._uiClientCount = msg.uiClientCount;
      }
      updateStatus();
    };
    const onEvent = (msg: any) => {
      updateStatus();
    };
    const onUiCount = (msg: any) => {
      // UI count event
      if (msg.uiClientCount !== undefined) {
        (client as any)._uiClientCount = msg.uiClientCount;
      }
      updateStatus();
    };
    const onWelcome = (msg: any) => {
      // Welcome may include initial UI count
      if (msg.uiClientCount !== undefined) {
        (client as any)._uiClientCount = msg.uiClientCount;
      }
      updateStatus();
    };
    const onCommandResult = (msg: any) => {
      (client as any)._lastCommand = msg.command;
      updateStatus();
    };
    const onControlResult = () => updateStatus();

    client.on('connection', onConnection);
    client.on('close', onClose);
    client.on('state', onState);
    client.on('event', onEvent);
    client.on('ui_count', onUiCount);
    client.on('welcome', onWelcome);
    client.on('command_result', onCommandResult);
    client.on('control_result', onControlResult);

    return () => {
      client.off('connection', onConnection);
      client.off('close', onClose);
      client.off('state', onState);
      client.off('event', onEvent);
      client.off('ui_count', onUiCount);
      client.off('welcome', onWelcome);
      client.off('command_result', onCommandResult);
      client.off('control_result', onControlResult);
    };
  }, []);

  return status;
}

/**
 * Hook to request/release control of the robot
 */
export function useCloudControl() {
  const [hasControl, setHasControl] = useState(false);
  const [requesting, setRequesting] = useState(false);

  useEffect(() => {
    if (!isCloudModeEnabled()) return;

    const client = getCloudClient();
    
    const onControlResult = (msg: any) => {
      setRequesting(false);
      if (msg.success) {
        setHasControl(msg.action === 'request' || msg.action === 'force');
      }
    };

    client.on('control_result', onControlResult);
    return () => client.off('control_result', onControlResult);
  }, []);

  const requestControl = useCallback(() => {
    if (!isCloudModeEnabled()) return;
    setRequesting(true);
    getCloudClient().requestControl();
  }, []);

  const releaseControl = useCallback(() => {
    if (!isCloudModeEnabled()) return;
    getCloudClient().releaseControl();
    setHasControl(false);
  }, []);

  const forceControl = useCallback(() => {
    if (!isCloudModeEnabled()) return;
    setRequesting(true);
    getCloudClient().forceControl();
  }, []);

  return { hasControl, requesting, requestControl, releaseControl, forceControl };
}

/**
 * Hook to send teleop commands via cloud
 */
export function useCloudTeleop() {
  const sendTeleop = useCallback((linearX: number, angularZ: number) => {
    if (!isCloudModeEnabled()) return;
    getCloudClient().sendTeleop(linearX, angularZ);
  }, []);

  const stop = useCallback(() => {
    if (!isCloudModeEnabled()) return;
    getCloudClient().sendTeleop(0, 0);
  }, []);

  return { sendTeleop, stop };
}
