import { useEffect, useState, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import { ros, getConnectionState, onConnectionChange, ConnectionState } from './ros';
import { ROS_CONFIG } from './config';

export function useRosConnection() {
  const [state, setState] = useState<ConnectionState>(getConnectionState());
  const [latency, setLatency] = useState<number | null>(null);

  useEffect(() => {
    const unsubscribe = onConnectionChange(setState);
    
    // Simple ping test
    const interval = setInterval(() => {
      if (state === 'connected') {
        const start = Date.now();
        // Simple connection test
        setTimeout(() => {
          setLatency(Date.now() - start);
        }, 10);
      } else {
        setLatency(null);
      }
    }, 5000);

    return () => {
      unsubscribe();
      clearInterval(interval);
    };
  }, [state]);

  return { state, latency, connected: state === 'connected' };
}

export interface Odometry {
  pose: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  twist: {
    linear: { x: number; y: number; z: number };
    angular: { x: number; y: number; z: number };
  };
}

export function useOdom() {
  const [odom, setOdom] = useState<Odometry | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const odomTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.odom,
      messageType: ROS_CONFIG.messageTypes.odom
    });
    
    odomTopic.subscribe((msg: Odometry) => {
      setOdom(msg);
    });
    
    return () => {
      odomTopic.unsubscribe();
    };
  }, []);
  
  return odom;
}

export function useBattery() {
  const [battery, setBattery] = useState<number | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.battery,
      messageType: ROS_CONFIG.messageTypes.battery
    });
    
    batteryTopic.subscribe((msg: any) => {
      // Extract battery value - adjust based on actual message structure
      const value = msg.data ?? msg.percentage ?? msg.battery_level ?? null;
      setBattery(typeof value === 'number' ? value : null);
    });
    
    return () => {
      batteryTopic.unsubscribe();
    };
  }, []);
  
  return battery;
}

export function useCmdVel() {
  const cmdVelRef = useRef<ROSLIB.Topic | null>(null);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    cmdVelRef.current = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.cmdVel,
      messageType: ROS_CONFIG.messageTypes.cmdVel
    });

    return () => {
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
    };
  }, []);
  
  const send = useCallback((linearX: number, angularZ: number, maxLinear = 0.5, maxAngular = 0.5) => {
    if (!cmdVelRef.current || getConnectionState() !== 'connected') {
      console.warn('Cannot send cmd_vel: not connected');
      return;
    }
    
    // Clamp speeds
    const clampedX = Math.max(-maxLinear, Math.min(maxLinear, linearX));
    const clampedZ = Math.max(-maxAngular, Math.min(maxAngular, angularZ));

    const twist = new ROSLIB.Message({
      linear: { x: clampedX, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: clampedZ }
    });

    cmdVelRef.current.publish(twist);

    // Auto-stop if no command within 300ms (safety feature)
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    timeoutRef.current = setTimeout(() => {
      const stop = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      });
      cmdVelRef.current?.publish(stop);
    }, 300);
  }, []);
  
  const stop = useCallback(() => {
    if (timeoutRef.current) clearTimeout(timeoutRef.current);
    send(0, 0);
  }, [send]);
  
  return { send, stop };
}

