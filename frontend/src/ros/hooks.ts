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

export interface OccupancyGrid {
  header: {
    frame_id: string;
    stamp: { sec: number; nanosec: number };
  };
  info: {
    resolution: number;
    width: number;
    height: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
  data: number[];
}

export function useMap() {
  const [map, setMap] = useState<OccupancyGrid | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const mapTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.map,
      messageType: ROS_CONFIG.messageTypes.map
    });
    
    mapTopic.subscribe((msg: OccupancyGrid) => {
      setMap(msg);
    });
    
    return () => {
      mapTopic.unsubscribe();
    };
  }, []);
  
  return map;
}

export interface PointOfInterest {
  name: string;
  x: number;
  y: number;
  yaw?: number;
}

// Extract POIs from map metadata or use default labs
export function usePointsOfInterest(): PointOfInterest[] {
  const [pois, setPois] = useState<PointOfInterest[]>([
    { name: 'Lab 1', x: 0.0, y: 0.0 },
    { name: 'Lab 2', x: 10.0, y: 0.0 },
  ]);

  // TODO: In the future, subscribe to a POI topic or read from map metadata
  // For now, return default POIs
  return pois;
}

export function useNavigateToPose() {
  const navigate = useCallback((x: number, y: number, yaw: number = 0) => {
    if (getConnectionState() !== 'connected') {
      console.warn('Cannot navigate: not connected');
      return Promise.reject('Not connected');
    }

    return new Promise<void>((resolve, reject) => {
      try {
        // Create action client
        const actionClient = new ROSLIB.ActionClient({
          ros,
          serverName: ROS_CONFIG.actions.navigateToPose,
          actionName: 'nav2_msgs/NavigateToPose'
        });

        // Create goal
        const goal = new ROSLIB.Goal({
          actionClient,
          goalMessage: {
            pose: {
              header: {
                frame_id: 'map',
                stamp: {
                  sec: 0,
                  nanosec: 0
                }
              },
              pose: {
                position: { x, y, z: 0 },
                orientation: {
                  x: 0,
                  y: 0,
                  z: Math.sin(yaw / 2),
                  w: Math.cos(yaw / 2)
                }
              }
            }
          }
        });

        goal.on('feedback', (feedback: any) => {
          console.log('Navigation feedback:', feedback);
        });

        goal.on('result', (result: any) => {
          // Status 4 = SUCCEEDED in action_msgs/GoalStatus
          if (result.status?.status === 4) {
            resolve();
          } else {
            reject(new Error(`Navigation failed with status: ${result.status?.status}`));
          }
        });

        goal.send();
      } catch (error: any) {
        reject(new Error(`Failed to start navigation: ${error.message}`));
      }
    });
  }, []);

  return { navigate };
}

// Service to change map (for debug panel)
export function changeMap(mapName: string): Promise<void> {
  return new Promise((resolve, reject) => {
    if (getConnectionState() !== 'connected') {
      reject(new Error('Not connected'));
      return;
    }

    const service = new ROSLIB.Service({
      ros,
      name: ROS_CONFIG.services.changeMap,
      serviceType: 'std_srvs/SetString'
    });

    const request = new ROSLIB.ServiceRequest({ data: mapName });

    service.callService(request, (result) => {
      if (result.success) {
        resolve();
      } else {
        reject(new Error('Failed to change map'));
      }
    });
  });
}

