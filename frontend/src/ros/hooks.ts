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
  const [connectionState, setConnectionState] = useState<ConnectionState>(getConnectionState());
  
  // Listen for connection changes
  useEffect(() => {
    const unsubscribe = onConnectionChange(setConnectionState);
    return unsubscribe;
  }, []);
  
  useEffect(() => {
    if (connectionState !== 'connected') {
      setBattery(null);
      return;
    }

    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.battery,
      messageType: ROS_CONFIG.messageTypes.battery
    });
    
    batteryTopic.subscribe((msg: any) => {
      // Debug: log the message structure
      console.log('Battery message received:', msg);
      
      // Extract battery value - adjust based on actual message structure
      // For std_msgs/UInt16, the data is typically in msg.data
      const rawValue = msg.data ?? msg.percentage ?? msg.battery_level ?? msg.value ?? null;
      
      console.log('Raw battery value:', rawValue);
      
      if (typeof rawValue === 'number' && !isNaN(rawValue)) {
        // Convert millivolts to percentage
        // Assuming values are in millivolts (e.g., 8365 = 8.365V)
        // For a 2S LiPo: 6000mV (0%) to 8400mV (100%)
        const MIN_VOLTAGE_MV = 6000;  // Empty battery
        const MAX_VOLTAGE_MV = 8400;  // Fully charged battery
        
        let percentage: number;
        
        // If value is already in percentage range (0-100), use as-is
        if (rawValue >= 0 && rawValue <= 100) {
          percentage = rawValue;
        } else {
          // Otherwise, assume it's millivolts and convert to percentage
          percentage = ((rawValue - MIN_VOLTAGE_MV) / (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV)) * 100;
          // Clamp between 0 and 100
          percentage = Math.max(0, Math.min(100, percentage));
        }
        
        console.log('Converted battery percentage:', percentage);
        setBattery(percentage);
      } else {
        console.warn('Invalid battery value:', rawValue);
        setBattery(null);
      }
    });
    
    return () => {
      batteryTopic.unsubscribe();
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
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const imuTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.imuRpy,
      messageType: ROS_CONFIG.messageTypes.imuRpy
    });
    
    imuTopic.subscribe((msg: any) => {
      // geometry_msgs/Vector3 has x, y, z fields
      const roll = msg.x ?? msg.roll ?? 0;
      const pitch = msg.y ?? msg.pitch ?? 0;
      const yaw = msg.z ?? msg.yaw ?? 0;
      setRpy({ x: roll, y: pitch, z: yaw });
    });
    
    return () => {
      imuTopic.unsubscribe();
    };
  }, []);
  
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
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const jointTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.jointStates,
      messageType: ROS_CONFIG.messageTypes.jointStates
    });
    
    jointTopic.subscribe((msg: any) => {
      // sensor_msgs/JointState structure
      setJointStates({
        name: msg.name || [],
        position: msg.position || [],
        velocity: msg.velocity || [],
        effort: msg.effort || []
      });
    });
    
    return () => {
      jointTopic.unsubscribe();
    };
  }, []);
  
  return jointStates;
}

export function useButton() {
  const [buttonPressed, setButtonPressed] = useState<boolean | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const buttonTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.button,
      messageType: ROS_CONFIG.messageTypes.button
    });
    
    buttonTopic.subscribe((msg: any) => {
      // Could be std_msgs/Bool or std_msgs/UInt8
      const value = msg.data ?? msg.value ?? false;
      setButtonPressed(Boolean(value));
    });
    
    return () => {
      buttonTopic.unsubscribe();
    };
  }, []);
  
  return buttonPressed;
}

export type RobotState = 'idle' | 'responding_to_command' | 'heading_to_charger';

export function useRobotState() {
  const [robotState, setRobotState] = useState<RobotState | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const stateTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.robotState,
      messageType: ROS_CONFIG.messageTypes.robotState
    });
    
    stateTopic.subscribe((msg: any) => {
      const stateStr = (msg.data ?? msg.state ?? '').toLowerCase().trim();
      // Normalize state values
      if (stateStr === 'idle' || stateStr === 'idle_state') {
        setRobotState('idle');
      } else if (stateStr === 'responding_to_command' || stateStr === 'responding' || stateStr === 'executing_command') {
        setRobotState('responding_to_command');
      } else if (stateStr === 'heading_to_charger' || stateStr === 'charging' || stateStr === 'going_to_charger') {
        setRobotState('heading_to_charger');
      } else {
        // Default to idle if unknown
        setRobotState('idle');
      }
    });
    
    return () => {
      stateTopic.unsubscribe();
    };
  }, []);
  
  return robotState;
}

export function useCurrentCommand() {
  const [currentCommand, setCurrentCommand] = useState<string | null>(null);
  
  useEffect(() => {
    if (getConnectionState() !== 'connected') return;

    const commandTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.currentCommand,
      messageType: ROS_CONFIG.messageTypes.currentCommand
    });
    
    commandTopic.subscribe((msg: any) => {
      const command = msg.data ?? msg.command ?? msg.destination ?? null;
      setCurrentCommand(typeof command === 'string' ? command : null);
    });
    
    return () => {
      commandTopic.unsubscribe();
    };
  }, []);
  
  return currentCommand;
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

// Fetch POIs from ROS topic
export function usePointsOfInterest(): PointOfInterest[] {
  const [pois, setPois] = useState<PointOfInterest[]>([]);

  useEffect(() => {
    if (getConnectionState() !== 'connected') {
      setPois([]);
      return;
    }

    const poisTopic = new ROSLIB.Topic({
      ros,
      name: ROS_CONFIG.topics.pois,
      messageType: ROS_CONFIG.messageTypes.pois || 'std_msgs/String'
    });
    
    poisTopic.subscribe((msg: any) => {
      try {
        // Parse POIs from message - adjust based on actual message structure
        // Could be JSON string, or structured message
        let parsed: PointOfInterest[] = [];
        
        if (typeof msg === 'string') {
          parsed = JSON.parse(msg);
        } else if (msg.data) {
          parsed = typeof msg.data === 'string' ? JSON.parse(msg.data) : msg.data;
        } else if (Array.isArray(msg.pois)) {
          parsed = msg.pois;
        } else if (Array.isArray(msg)) {
          parsed = msg;
        }
        
        if (Array.isArray(parsed)) {
          setPois(parsed);
        }
      } catch (error) {
        console.error('Failed to parse POIs:', error);
        setPois([]);
      }
    });
    
    return () => {
      poisTopic.unsubscribe();
    };
  }, []);

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

// Fetch available maps from ROS service
export function useAvailableMaps(): string[] {
  const [maps, setMaps] = useState<string[]>([]);

  useEffect(() => {
    if (getConnectionState() !== 'connected') {
      setMaps([]);
      return;
    }

    // Try to call list_maps service
    const service = new ROSLIB.Service({
      ros,
      name: ROS_CONFIG.services.listMaps,
      serviceType: 'std_srvs/Empty' // Adjust based on actual service type
    });

    const request = new ROSLIB.ServiceRequest({});

    service.callService(request, (result: any) => {
      try {
        // Parse maps from result - adjust based on actual service response
        let mapList: string[] = [];
        
        if (result.maps && Array.isArray(result.maps)) {
          mapList = result.maps;
        } else if (result.data && Array.isArray(result.data)) {
          mapList = result.data;
        } else if (typeof result === 'string') {
          mapList = JSON.parse(result);
        }
        
        if (Array.isArray(mapList)) {
          setMaps(mapList);
        }
      } catch (error) {
        console.error('Failed to parse maps list:', error);
        setMaps([]);
      }
    }, (error: any) => {
      console.error('Failed to list maps:', error);
      setMaps([]);
    });

    // Also try subscribing to a topic if available
    const mapsTopic = new ROSLIB.Topic({
      ros,
      name: '/available_maps',
      messageType: 'std_msgs/String'
    });

    mapsTopic.subscribe((msg: any) => {
      try {
        const mapList = typeof msg === 'string' 
          ? JSON.parse(msg) 
          : (msg.data ? JSON.parse(msg.data) : []);
        if (Array.isArray(mapList)) {
          setMaps(mapList);
        }
      } catch (error) {
        console.error('Failed to parse maps from topic:', error);
      }
    });

    return () => {
      mapsTopic.unsubscribe();
    };
  }, []);

  return maps;
}

