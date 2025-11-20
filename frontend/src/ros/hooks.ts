// Subscribe to robot pose. Prefer /amcl_pose (map frame, PoseWithCovarianceStamped) when available,
// otherwise fall back to /robot/pose (PoseStamped published by system_topics or other nodes).
export function useRobotPose() {
  const [pose, setPose] = useState<any | null>(null);
  const connectionState = useConnectionWatcher();

  useEffect(() => {
    if (connectionState !== 'connected') {
      setPose(null);
      return;
    }

    let activeTopic: ROSLIB.Topic | null = null;
    let legacyTopic: ROSLIB.Topic | null = null;
    const handleMsg = (msg: any) => {
      // Normalize PoseWithCovarianceStamped -> pose, PoseStamped -> pose
      if (msg?.pose) {
        // PoseWithCovarianceStamped has .pose, PoseStamped may also have .pose
        setPose(msg);
      } else if (msg?.position || msg?.pose?.position) {
        setPose(msg);
      } else {
        setPose(msg);
      }
    };

    // Try to query available topics and prefer /amcl_pose
    try {
      (ros as any).getTopics((res: any) => {
        const available: string[] = res?.topics || [];
        const preferred = available.includes('/amcl_pose') ? '/amcl_pose' : '/robot/pose';
        const preferredType = preferred === '/amcl_pose' ? 'geometry_msgs/PoseWithCovarianceStamped' : 'geometry_msgs/PoseStamped';
        try {
          activeTopic = new ROSLIB.Topic({ 
            ros, 
            name: preferred, 
            messageType: preferredType,
            throttle_rate: 100,  // 10 Hz - smooth robot tracking without overload
            queue_length: 1,
          } as any);
          activeTopic.subscribe(handleMsg);
        } catch (e) {
          // fallback to robot/pose topic directly
          try {
            legacyTopic = new ROSLIB.Topic({ 
              ros, 
              name: '/robot/pose', 
              messageType: 'geometry_msgs/PoseStamped',
              throttle_rate: 100,
              queue_length: 1,
            } as any);
            legacyTopic.subscribe(handleMsg);
          } catch (e2) {
            // no-op
          }
        }
      });
    } catch (e) {
      // If getTopics not available, just subscribe to /robot/pose
      try {
        activeTopic = new ROSLIB.Topic({ 
          ros, 
          name: '/robot/pose', 
          messageType: 'geometry_msgs/PoseStamped',
          throttle_rate: 100,
          queue_length: 1,
        } as any);
        activeTopic.subscribe(handleMsg);
      } catch (e) {
        // ignore
      }
    }

    return () => {
      try { if (activeTopic) activeTopic.unsubscribe(handleMsg); } catch {}
      try { if (legacyTopic) legacyTopic.unsubscribe(handleMsg); } catch {}
    };
  }, [connectionState]);

  return pose;
}
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
      messageType: ROS_CONFIG.messageTypes.odom,
      throttle_rate: 200,  // 5 Hz for UI updates
      queue_length: 1,
    } as any);

    const handleMessage = (msg: Odometry) => setOdom(msg);
    odomTopic.subscribe((msg: any) => handleMessage(msg));
    
    return () => {
      odomTopic.unsubscribe((msg: any) => handleMessage(msg));
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

export function parseBatteryMessage(msg: any): BatteryReading | null {
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
    return null;
  }

  return { millivolts: mv ?? (v != null ? Math.round(v * 1000) : null), volts: v ?? (mv != null ? mv / 1000 : null), percent: pct ?? null };
}

export function useBattery(): BatteryReading | null {
  const [battery, setBattery] = useState<BatteryReading | null>(null);
  const connectionState = useConnectionWatcher();

  useEffect(() => {
    if (connectionState !== 'connected') {
      setBattery(null);
      return;
    }

    // Wait for topic to be advertised before subscribing
    let batteryTopic: ROSLIB.Topic | null = null;
    let checkInterval: NodeJS.Timeout | null = null;

    const trySubscribe = () => {
      if (batteryTopic) return; // already subscribed

      (ros as any).getTopics((result: any) => {
        const topics = result?.topics || [];
        if (!topics.includes(ROS_CONFIG.topics.battery)) {
          // Topic not advertised yet, wait
          return;
        }

        // Topic is available, subscribe
        if (checkInterval) {
          clearInterval(checkInterval);
          checkInterval = null;
        }

        batteryTopic = new ROSLIB.Topic({
          ros,
          name: ROS_CONFIG.topics.battery,
          throttle_rate: 2000,  // 0.5 Hz - battery doesn't change quickly
          queue_length: 1,
        } as any);

        batteryTopic.subscribe(handleMessage);
      }, (err: any) => {
        console.error('Failed to query topics:', err);
      });
    };

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

    // Try immediately
    trySubscribe();

    // Poll every 2 seconds until subscribed
    checkInterval = setInterval(trySubscribe, 2000);

    return () => {
      if (checkInterval) clearInterval(checkInterval);
      if (batteryTopic) batteryTopic.unsubscribe(handleMessage);
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
      messageType: ROS_CONFIG.messageTypes.imuRpy,
      throttle_rate: 200,  // 5 Hz
      queue_length: 1,
    } as any);
    
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
      messageType: ROS_CONFIG.messageTypes.jointStates,
      throttle_rate: 500,  // 2 Hz
      queue_length: 1,
    } as any);
    
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
    // Subscribe to canonical '/robot/state' and fall back to legacy '/robot_state'.
    // We create two topics and accept messages from either. This makes the UI
    // robust to small naming differences in bringup or remaps.
    const primary = new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.robotState, messageType: ROS_CONFIG.messageTypes.robotState });
    const legacyName = ROS_CONFIG.topics.robotState === '/robot/state' ? '/robot_state' : undefined;
    const legacy = legacyName ? new ROSLIB.Topic({ ros, name: legacyName, messageType: ROS_CONFIG.messageTypes.robotState }) : null;

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

    primary.subscribe(handleMessage);
    if (legacy) legacy.subscribe(handleMessage);

    return () => {
      try { primary.unsubscribe(handleMessage); } catch {}
      if (legacy) try { legacy.unsubscribe(handleMessage); } catch {}
    };
  }, [connectionState]);
  
  return robotState;
}

export function useCmdVel() {
  const cmdVelRef = useRef<ROSLIB.Topic | null>(null);
  // Removed auto-stop timeout per clarified requirements (instant manual stop only)
  
  useEffect(() => {
    const handleConnectionChange = (state: ConnectionState) => {
      if (state === 'connected') {
        // Query rosbridge for available topics and prefer configured topic,
        // but fall back to '/cmd_vel' if the UI topic isn't present.
        try {
          (ros as any).getTopics((res: any) => {
            const available: string[] = res?.topics || [];
            let chosen = ROS_CONFIG.topics.cmdVel;
            if (!available.includes(chosen) && available.includes('/cmd_vel')) {
              chosen = '/cmd_vel';
            }
            cmdVelRef.current = new ROSLIB.Topic({ ros, name: chosen, messageType: ROS_CONFIG.messageTypes.cmdVel });
            console.info('cmd_vel topic selected:', chosen);
          });
        } catch (e) {
          // Fallback: use configured name
          cmdVelRef.current = new ROSLIB.Topic({ ros, name: ROS_CONFIG.topics.cmdVel, messageType: ROS_CONFIG.messageTypes.cmdVel });
        }
      } else {
        cmdVelRef.current = null;
      }
    };

    handleConnectionChange(getConnectionState());
    const unsubscribe = onConnectionChange(handleConnectionChange);

    return () => {
      unsubscribe();
      cmdVelRef.current = null;
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

  }, []);
  
  const stop = useCallback(() => {
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

    mapTopic.subscribe((msg: any) => handleMessage(msg));
    
    return () => {
      mapTopic.unsubscribe((msg: any) => handleMessage(msg));
    };
  }, [connectionState]);
  
  return map;
}

// POI supports both legacy (x, y, yaw?) and new schema (pose: {x, y, yaw})
export interface PointOfInterest {
  name: string;
  x?: number;
  y?: number;
  yaw?: number;
  pose?: {
    x: number;
    y: number;
    yaw?: number;
  };
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
          // New schema: { name, pose: {x, y, yaw} }
          if (p?.pose && typeof p.pose.x === 'number' && typeof p.pose.y === 'number') {
            out.push({ name: p?.name || `POI ${i+1}`, pose: { x: p.pose.x, y: p.pose.y, yaw: p.pose.yaw } });
          } else {
            // Legacy: { name, x, y, yaw }
            const x = Number(p?.x);
            const y = Number(p?.y);
            const yaw = p?.yaw != null ? Number(p.yaw) : undefined;
            if (!Number.isNaN(x) && !Number.isNaN(y)) out.push({ name: p?.name || `POI ${i+1}`, x, y, yaw });
          }
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

export interface ModeAndMaps {
  mode: string | null; // 'slam', 'localization', 'idle', etc
  activeMap: string | null; // currently loaded map name
  maps: string[]; // list of available maps
  loading: boolean;
  error: string | null;
  refresh: () => void;
}

export function useModeAndMaps(): ModeAndMaps {
  const [mode, setMode] = useState<string | null>(null);
  const [activeMap, setActiveMap] = useState<string | null>(null);
  const [maps, setMaps] = useState<string[]>([]);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  const connectionState = useConnectionWatcher();

  const fetchData = useCallback(async () => {
    if (connectionState !== 'connected') {
      setMode(null);
      setActiveMap(null);
      setMaps([]);
      setLoading(false);
      setError('Not connected to ROS');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      // Dynamically import to avoid circular deps
      const { getMode, listMaps } = await import('./services');
      
      // Fetch mode and map info
      const [modeRes, mapsRes] = await Promise.all([
        getMode().catch(() => null),
        listMaps().catch(() => [])
      ]);

      // Parse mode response (returns { mode, map, msg })
      if (modeRes) {
        setMode(modeRes.mode || null);
        setActiveMap(modeRes.map || null);
      } else {
        setMode(null);
        setActiveMap(null);
      }

      // Parse maps list
      if (Array.isArray(mapsRes)) {
        setMaps(mapsRes.filter((m): m is string => typeof m === 'string'));
      } else {
        setMaps([]);
      }
    } catch (e: any) {
      console.error('Failed to fetch mode/maps:', e);
      setError(e?.message || 'Failed to fetch mode/maps');
    } finally {
      setLoading(false);
    }
  }, [connectionState]);

  useEffect(() => {
    fetchData();
  }, [fetchData]);

  return {
    mode,
    activeMap,
    maps,
    loading,
    error,
    refresh: fetchData
  };
}
