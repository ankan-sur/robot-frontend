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

function useConnectionWatcher() {
  const [connectionState, setConnectionState] = useState<ConnectionState>(getConnectionState());

  useEffect(() => {
    const unsubscribe = onConnectionChange(setConnectionState);
    return unsubscribe;
  }, []);

  return connectionState;
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
  const connectionState = useConnectionWatcher();
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setOdom(null);
      return;
    }

    const odomTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.odom,
      messageType: ROS_CONFIG.messageTypes.odom
    });

    const handleMessage = (msg: Odometry) => setOdom(msg);
    odomTopic.subscribe(handleMessage);
    
    return () => {
      odomTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);
  
  return odom;
}

export interface BatteryReading {
  millivolts: number | null;
  volts: number | null;
  percent: number | null;
}

const BATTERY_MIN_MV = 6000; // 6.0V (2S LiPo lower bound, adjust to your pack)
const BATTERY_MAX_MV = 8400; // 8.4V (2S LiPo full charge)

export function useBattery(): BatteryReading | null {
  const [battery, setBattery] = useState<BatteryReading | null>(null);
  const connectionState = useConnectionWatcher();

  useEffect(() => {
    if (connectionState !== 'connected') {
      setBattery(null);
      return;
    }

    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.battery,
    } as any);

    const handleMessage = (msg: any) => {
      // Try to interpret as BatteryState or simple numeric messages.
      // Possible shapes:
      // - std_msgs/UInt16: { data: <millivolts> }
      // - std_msgs/Float32: { data: <volts or percent> }
      // - sensor_msgs/BatteryState: { voltage: <V>, percentage: <0..1> }
      // - custom: { battery_level, value, percentage }

      let mv: number | null = null;
      let v: number | null = null;
      let pct: number | null = null;

      const dataVal = msg?.data ?? msg?.value ?? null;
      const voltageV = msg?.voltage ?? msg?.voltage_v ?? null;
      const percentageField = msg?.percentage ?? msg?.percent ?? null;

      if (typeof voltageV === 'number' && !isNaN(voltageV)) {
        v = voltageV;
        mv = Math.round(voltageV * 1000);
      }

      if (typeof dataVal === 'number' && !isNaN(dataVal)) {
        if (dataVal > 100) {
          // Likely millivolts
          mv = dataVal;
          v = dataVal / 1000;
        } else {
          // 0..100 percent or volts in small systems; assume percent here
          pct = dataVal;
        }
      }

      if (typeof percentageField === 'number' && !isNaN(percentageField)) {
        // BatteryState uses 0..1
        pct = percentageField <= 1 ? percentageField * 100 : percentageField;
      }

      // Compute missing percent from mV if needed
      if ((pct == null || isNaN(pct)) && typeof mv === 'number') {
        const clamped = Math.max(0, Math.min(100, ((mv - BATTERY_MIN_MV) / (BATTERY_MAX_MV - BATTERY_MIN_MV)) * 100));
        pct = clamped;
      }

      // Round for stable UI
      if (typeof v === 'number') v = Math.round(v * 100) / 100; // 2 decimals
      if (typeof mv === 'number') mv = Math.round(mv);
      if (typeof pct === 'number') pct = Math.max(0, Math.min(100, Math.round(pct)));

      if (v == null && mv == null && pct == null) {
        setBattery(null);
      } else {
        setBattery({ millivolts: mv ?? (v != null ? Math.round(v * 1000) : null), volts: v ?? (mv != null ? mv / 1000 : null), percent: pct ?? null });
      }
    };

    batteryTopic.subscribe(handleMessage);

    return () => {
      batteryTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);

  return battery;
}

export interface IMURPY {
  x: number; // Roll
  y: number; // Pitch
  z: number; // Yaw
}

export function useIMURPY() {
  const [rpy, setRpy] = useState<IMURPY | null>(null);
  const connectionState = useConnectionWatcher();
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setRpy(null);
      return;
    }

    const imuTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.imuRpy,
      messageType: ROS_CONFIG.messageTypes.imuRpy
    });
    
    const handleMessage = (msg: any) => {
      const vector = msg?.vector ?? msg;
      const roll = vector?.x ?? vector?.roll ?? 0;
      const pitch = vector?.y ?? vector?.pitch ?? 0;
      const yaw = vector?.z ?? vector?.yaw ?? 0;
      setRpy({ x: roll, y: pitch, z: yaw });
    };

    imuTopic.subscribe(handleMessage);
    
    return () => {
      imuTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);
  
  return rpy;
}

export interface JointState {
  name: string[];
  position: number[];
  velocity: number[];
  effort: number[];
}

export function useJointStates() {
  const [jointStates, setJointStates] = useState<JointState | null>(null);
  const connectionState = useConnectionWatcher();
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setJointStates(null);
      return;
    }

    const jointTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.jointStates,
      messageType: ROS_CONFIG.messageTypes.jointStates
    });
    
    const handleMessage = (msg: any) => {
      setJointStates({
        name: msg.name || [],
        position: msg.position || [],
        velocity: msg.velocity || [],
        effort: msg.effort || []
      });
    };

    jointTopic.subscribe(handleMessage);
    
    return () => {
      jointTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);
  
  return jointStates;
}

export function useButton() {
  const [buttonPressed, setButtonPressed] = useState<boolean | null>(null);
  const connectionState = useConnectionWatcher();
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setButtonPressed(null);
      return;
    }

    const buttonTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.button,
      messageType: ROS_CONFIG.messageTypes.button
    });
    
    const handleMessage = (msg: any) => {
      const value = msg?.pressed ?? msg?.state ?? msg?.data ?? msg?.value ?? false;
      setButtonPressed(Boolean(value));
    };

    buttonTopic.subscribe(handleMessage);
    
    return () => {
      buttonTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);
  
  return buttonPressed;
}

export type RobotState = 'idle' | 'responding_to_command' | 'heading_to_charger';

export function useRobotState() {
  const [robotState, setRobotState] = useState<RobotState | null>(null);
  const connectionState = useConnectionWatcher();
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setRobotState(null);
      return;
    }

    const stateTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.robotState,
      messageType: ROS_CONFIG.messageTypes.robotState
    });
    
    const handleMessage = (msg: any) => {
      let stateStrRaw: any = msg?.data ?? msg?.state ?? '';
      if (typeof stateStrRaw === 'string' && stateStrRaw.trim().startsWith('{')) {
        try {
          const parsed = JSON.parse(stateStrRaw);
          stateStrRaw = parsed?.state ?? stateStrRaw;
        } catch {}
      }
      const stateStr = String(stateStrRaw).toLowerCase().trim();
      if (stateStr === 'idle' || stateStr === 'idle_state') {
        setRobotState('idle');
      } else if (stateStr === 'responding_to_command' || stateStr === 'responding' || stateStr === 'executing_command') {
        setRobotState('responding_to_command');
      } else if (stateStr === 'heading_to_charger' || stateStr === 'charging' || stateStr === 'going_to_charger') {
        setRobotState('heading_to_charger');
      } else {
        setRobotState(null);
      }
    };

    stateTopic.subscribe(handleMessage);
    
    return () => {
      stateTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);
  
  return robotState;
}

export function useCmdVel() {
  const cmdVelRef = useRef<ROSLIB.Topic | null>(null);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  
  useEffect(() => {
    const handleConnectionChange = (state: ConnectionState) => {
      if (state === 'connected') {
        cmdVelRef.current = new ROSLIB.Topic({
          ros,
          name: ROS_CONFIG.topics.cmdVel,
          messageType: ROS_CONFIG.messageTypes.cmdVel
        });
      } else {
        cmdVelRef.current = null;
        if (timeoutRef.current) {
          clearTimeout(timeoutRef.current);
          timeoutRef.current = null;
        }
      }
    };

    handleConnectionChange(getConnectionState());
    const unsubscribe = onConnectionChange(handleConnectionChange);

    return () => {
      unsubscribe();
      cmdVelRef.current = null;
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
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
  const connectionState = useConnectionWatcher();
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setMap(null);
      return;
    }

    const mapTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.map,
      messageType: ROS_CONFIG.messageTypes.map
    });
    
    const handleMessage = (msg: OccupancyGrid) => {
      setMap(msg);
    };

    mapTopic.subscribe(handleMessage);
    
    return () => {
      mapTopic.unsubscribe(handleMessage);
    };
  }, [connectionState]);
  
  return map;
}

export interface PointOfInterest {
  name: string;
  x: number;
  y: number;
  yaw?: number;
}

// Fetch POIs for a given map via service (fallback to /pois topic if service fails)
export function usePoisForMap(mapName?: string): PointOfInterest[] {
  const [pois, setPois] = useState<PointOfInterest[]>([]);
  const connectionState = useConnectionWatcher();

  useEffect(() => {
    if (connectionState !== 'connected') {
      setPois([]);
      return;
    }

    // Prefer /system/map/info; fallback to /system/map/pois
    const handleList = (list: any) => {
      const out: PointOfInterest[] = [];
      if (Array.isArray(list)) {
        list.forEach((p: any, i: number) => {
          const base = p?.point || p?.position || p;
          const x = Number(base?.x);
          const y = Number(base?.y);
          const yaw = p?.yaw != null ? Number(p.yaw) : (base?.yaw != null ? Number(base.yaw) : undefined);
          if (!Number.isNaN(x) && !Number.isNaN(y)) out.push({ name: p?.name || `POI ${i+1}`, x, y, yaw });
        });
      }
      setPois(out);
    };

    const infoSvc = new ROSLIB.Service({ ros, name: ROS_CONFIG.services.systemMapInfo, serviceType: 'interfaces/SetString' });
    const req = new ROSLIB.ServiceRequest({ data: mapName || '' });
    infoSvc.callService(req, (res: any) => {
      try {
        const s = res?.message || res?.data || res;
        const parsed = typeof s === 'string' ? JSON.parse(s) : s;
        handleList(parsed?.pois || []);
      } catch (e) {
        console.error('map/info parse error', e);
        const poisSvc = new ROSLIB.Service({ ros, name: ROS_CONFIG.services.systemMapPois, serviceType: 'interfaces/SetString' });
        const req2 = new ROSLIB.ServiceRequest({ data: mapName || '' });
        poisSvc.callService(req2, (res2: any) => {
          try {
            const s2 = res2?.message || res2?.data || res2;
            const parsed2 = typeof s2 === 'string' ? JSON.parse(s2) : s2;
            handleList(parsed2?.pois || parsed2 || []);
          } catch {
            setPois([]);
          }
        }, () => setPois([]));
      }
    }, () => {
      const poisSvc = new ROSLIB.Service({ ros, name: ROS_CONFIG.services.systemMapPois, serviceType: 'interfaces/SetString' });
      const req2 = new ROSLIB.ServiceRequest({ data: mapName || '' });
      poisSvc.callService(req2, (res2: any) => {
        try {
          const s2 = res2?.message || res2?.data || res2;
          const parsed2 = typeof s2 === 'string' ? JSON.parse(s2) : s2;
          handleList(parsed2?.pois || parsed2 || []);
        } catch {
          setPois([]);
        }
      }, () => setPois([]));
    });
  }, [connectionState, mapName]);

  return pois;
}

// Backward-compatible name
export const usePointsOfInterest = usePoisForMap;

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
          actionName: 'nav2_msgs/action/NavigateToPose'
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
// map change service removed; handled by bringup/nav stack

// Subscribe to list of available maps from /available_maps (std_msgs/String JSON array)
export function useAvailableMaps(): string[] {
  const [maps, setMaps] = useState<string[]>([]);
  const connectionState = useConnectionWatcher();

  useEffect(() => {
    if (connectionState !== 'connected') {
      setMaps([]);
      return;
    }

    // Prefer service-based listing; fallback to topic if service fails
    (async () => {
      try {
        const { listMaps } = await import('./services')
        const arr = await listMaps()
        if (Array.isArray(arr) && arr.length) { setMaps(arr); return }
      } catch {}
      const topic = new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.availableMaps, messageType: ROS_CONFIG.messageTypes.availableMaps || 'std_msgs/String' });
      const handle = (msg: any) => {
        try {
          const str = typeof msg === 'string' ? msg : (typeof msg?.data === 'string' ? msg.data : '[]');
          const parsed = JSON.parse(str);
          if (Array.isArray(parsed)) setMaps(parsed.filter((x:any) => typeof x === 'string'));
        } catch (e) { console.error('Failed to parse available_maps:', e) }
      };
      topic.subscribe(handle);
      return () => { try { topic.unsubscribe(handle) } catch {} };
    })();
  }, [connectionState]);

  return maps;
}

export type NavGoalStatus = {
  id?: number[];
  status?: number;
  text?: string;
}

export function useNavStatus(): NavGoalStatus | null {
  const [st, setSt] = useState<NavGoalStatus | null>(null)
  const connectionState = useConnectionWatcher();

  useEffect(() => {
    if (connectionState !== 'connected') { setSt(null); return }
    const topic = new ROSLIB.Topic({ ros, name: (ROS_CONFIG.nav?.statusTopic || '/navigate_to_pose/status'), messageType: 'action_msgs/GoalStatusArray' })
    const handle = (msg: any) => {
      try {
        const list = msg?.status_list || msg?.status || []
        if (Array.isArray(list) && list.length > 0) {
          const latest = list[list.length - 1]
          const id = latest?.goal_info?.goal_id?.uuid || latest?.goal_id?.uuid
          const code = latest?.status
          const text = statusCodeToText(code)
          setSt({ id, status: code, text })
        }
      } catch (e) { /* ignore */ }
    }
    topic.subscribe(handle)
    return () => { try { topic.unsubscribe(handle) } catch {} }
  }, [connectionState])

  return st
}

function statusCodeToText(code: number | undefined): string {
  switch (code) {
    case 0: return 'UNKNOWN'
    case 1: return 'ACCEPTED'
    case 2: return 'EXECUTING'
    case 3: return 'CANCELING'
    case 4: return 'SUCCEEDED'
    case 5: return 'CANCELED'
    case 6: return 'ABORTED'
    default: return '—'
  }
}

// Demo initial pose (from RViz /initialpose captured during setup)
export const DEMO_INITIAL_POSE = {
  x: -0.1869187355,
  y: -0.1397080421,
  yaw: 0.045868,
};

// Publish an initial pose to /initialpose (geometry_msgs/PoseWithCovarianceStamped)
export function publishInitialPose(
  x: number,
  y: number,
  yaw: number,
  covariance: { covX?: number; covY?: number; covYaw?: number } = {}
): Promise<void> {
  const { covX = 0.25, covY = 0.25, covYaw = 0.06853891909122467 } = covariance;

  return new Promise((resolve, reject) => {
    if (getConnectionState() !== 'connected') {
      reject(new Error('Not connected'));
      return;
    }

    try {
      const topic = new ROSLIB.Topic({
        ros,
        name: '/initialpose',
        messageType: 'geometry_msgs/PoseWithCovarianceStamped',
      });

      // 6x6 covariance flattened row-major (36 entries)
      const cov = Array(36).fill(0);
      cov[0] = covX;   // x
      cov[7] = covY;   // y
      cov[35] = covYaw; // yaw

      const msg = new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: {
          pose: {
            position: { x, y, z: 0 },
            orientation: {
              x: 0,
              y: 0,
              z: Math.sin(yaw / 2),
              w: Math.cos(yaw / 2),
            },
          },
          covariance: cov,
        },
      });

      topic.publish(msg);
      resolve();
    } catch (e: any) {
      reject(new Error(e?.message || 'Failed to publish initial pose'));
    }
  });
}

// available maps handled via /available_maps directly in UI when needed
