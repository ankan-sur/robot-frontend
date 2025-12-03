/**
 * Cloud WebSocket Client (v1.0.0 Protocol)
 * 
 * Connects to cloud backend instead of direct rosbridge connection.
 * Handles telemetry updates, command forwarding, and control locking.
 * 
 * Protocol:
 * - UI connects to /ui endpoint
 * - Sends 'subscribe' message with robotId and clientName
 * - Receives 'state' with full robot state or 'event' for updates
 * - Uses 'control' messages for request/release/force control lock
 * - Sends 'command' messages when control is held
 */

// ============================================================================
// Type Definitions - New Protocol v1.0.0
// ============================================================================

export interface TelemetryPayload {
  pose?: {
    x: number;
    y: number;
    z: number;
    theta: number;
  };
  battery?: number;
  battery_voltage?: number;
  state?: string;
  mode?: string;
  current_map?: string;
  maps?: string[];
  pois?: Array<{ id: string; name: string; x: number; y: number }>;
  nav?: {
    active: boolean;
    goal_id: string | null;
    status: string;
  };
}

export interface RobotState {
  online: boolean;
  lastSeen: number;
  pose?: TelemetryPayload['pose'];
  battery?: number;
  mode?: string;
  maps?: string[];
  pois?: TelemetryPayload['pois'];
  nav?: TelemetryPayload['nav'];
  controlledBy?: string | null;
}

// Messages FROM server
export interface WelcomeMessage {
  type: 'welcome';
  config: {
    telemetryInterval: number;
    controlLockTimeout: number;
    maxLinearVel: number;
    maxAngularVel: number;
  };
}

export interface StateMessage {
  type: 'state';
  robotId: string;
  state: RobotState;
}

export interface EventMessage {
  type: 'event';
  robotId: string;
  event: 
    | 'robot_connected'
    | 'robot_disconnected'
    | 'control_acquired'
    | 'control_released'
    | 'control_timeout'
    | 'control_forced'
    | 'telemetry';
  data?: any;
}

export interface CommandResultMessage {
  type: 'command_result';
  robotId: string;
  command: string;
  success: boolean;
  message: string;
  timestamp: string;
}

export interface ControlResultMessage {
  type: 'control_result';
  robotId: string;
  action: 'request' | 'release' | 'force';
  success: boolean;
  holder?: string | null;
  reason?: string;
}

export interface ErrorMessage {
  type: 'error';
  error: string;
  code?: string;
}

// Messages TO server
export interface SubscribeMessage {
  type: 'subscribe';
  robotId: string;
  clientName?: string;
}

export interface ControlMessage {
  type: 'control';
  robotId: string;
  action: 'request' | 'release' | 'force';
}

export interface CommandMessage {
  type: 'command';
  robotId: string;
  command: string;
  payload?: Record<string, any>;
}

type IncomingMessage = 
  | WelcomeMessage 
  | StateMessage 
  | EventMessage 
  | CommandResultMessage 
  | ControlResultMessage 
  | ErrorMessage 
  | { type: 'pong' };

// Legacy compatibility types
export interface CloudTelemetry {
  type: 'telemetry';
  robotId: string;
  ts: string;
  payload: TelemetryPayload;
}

export interface CloudCommand {
  type: 'command';
  robotId: string;
  command: string;
  payload?: Record<string, any>;
}

// ============================================================================
// CloudWebSocketClient Class
// ============================================================================

export class CloudWebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private robotId: string;
  private clientName: string;
  private reconnectInterval: number;
  private reconnectTimer?: number;
  private pingInterval?: number;
  private listeners: Map<string, Set<(data: any) => void>>;
  private connected: boolean = false;
  
  // State tracking
  private robotState: RobotState | null = null;
  private controlHolder: string | null = null;
  private serverConfig: WelcomeMessage['config'] | null = null;

  constructor(
    url: string, 
    robotId: string = 'fordward', 
    clientName: string = 'web-ui',
    reconnectInterval: number = 5000
  ) {
    this.url = url;
    this.robotId = robotId;
    this.clientName = clientName;
    this.reconnectInterval = reconnectInterval;
    this.listeners = new Map();
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        console.log(`[Cloud] Connecting to ${this.url}...`);
        this.ws = new WebSocket(this.url);

        this.ws.onopen = () => {
          console.log('[Cloud] Connected to backend');
          this.connected = true;
          this.emit('connection', {});
          
          // Send subscribe message (new protocol)
          this.sendRaw({
            type: 'subscribe',
            robotId: this.robotId,
            clientName: this.clientName
          });
          
          // Start ping interval
          this.startPingInterval();
          resolve();
        };

        this.ws.onmessage = (event) => {
          try {
            const message: IncomingMessage = JSON.parse(event.data);
            this.handleMessage(message);
          } catch (error) {
            console.error('[Cloud] Error parsing message:', error);
          }
        };

        this.ws.onerror = (error) => {
          console.error('[Cloud] WebSocket error:', error);
          this.emit('error', error);
          reject(error);
        };

        this.ws.onclose = () => {
          console.log('[Cloud] Connection closed');
          this.connected = false;
          this.stopPingInterval();
          this.emit('close', {});
          this.scheduleReconnect();
        };
      } catch (error) {
        reject(error);
      }
    });
  }

  private handleMessage(message: IncomingMessage) {
    switch (message.type) {
      case 'welcome':
        this.serverConfig = message.config;
        console.log('[Cloud] Received welcome, config:', message.config);
        this.emit('welcome', message);
        break;
      
      case 'state':
        this.robotState = message.state;
        this.controlHolder = message.state.controlledBy ?? null;
        this.emit('state', message);
        // Also emit legacy telemetry event for compatibility
        this.emit('telemetry', this.convertToLegacyTelemetry(message.state));
        break;
      
      case 'event':
        this.handleEvent(message);
        break;
      
      case 'command_result':
        this.emit('command_result', message);
        break;
      
      case 'control_result':
        this.handleControlResult(message);
        break;
      
      case 'pong':
        // Ping response, no action needed
        break;
      
      case 'error':
        console.error('[Cloud] Backend error:', message.error);
        this.emit('error', message);
        break;
      
      default:
        console.warn('[Cloud] Unknown message type:', (message as any).type);
    }
  }

  private handleEvent(message: EventMessage) {
    const { event, robotId, data } = message;
    console.log(`[Cloud] Event: ${event}`, data);
    
    switch (event) {
      case 'robot_connected':
        if (this.robotState) this.robotState.online = true;
        break;
      case 'robot_disconnected':
        if (this.robotState) this.robotState.online = false;
        break;
      case 'control_acquired':
        this.controlHolder = data?.holder ?? null;
        break;
      case 'control_released':
      case 'control_timeout':
      case 'control_forced':
        this.controlHolder = null;
        break;
      case 'telemetry':
        if (data && this.robotState) {
          Object.assign(this.robotState, data);
        }
        this.emit('telemetry', this.convertToLegacyTelemetry(this.robotState));
        break;
    }
    
    this.emit('event', message);
    this.emit(event, { robotId, data });
  }

  private handleControlResult(message: ControlResultMessage) {
    console.log(`[Cloud] Control result: ${message.action} -> ${message.success}`, message);
    
    if (message.success) {
      if (message.action === 'request' || message.action === 'force') {
        this.controlHolder = message.holder ?? this.clientName;
      } else if (message.action === 'release') {
        this.controlHolder = null;
      }
    }
    
    this.emit('control_result', message);
  }

  private convertToLegacyTelemetry(state: RobotState | null): CloudTelemetry | null {
    if (!state) return null;
    return {
      type: 'telemetry',
      robotId: this.robotId,
      ts: new Date().toISOString(),
      payload: {
        pose: state.pose,
        battery: state.battery,
        mode: state.mode,
        maps: state.maps,
        pois: state.pois,
        nav: state.nav,
        state: state.online ? 'online' : 'offline'
      }
    };
  }

  private sendRaw(message: object): boolean {
    if (!this.connected || !this.ws) {
      console.error('[Cloud] Cannot send: not connected');
      return false;
    }
    this.ws.send(JSON.stringify(message));
    return true;
  }

  // ============================================================================
  // Control Lock Methods (NEW)
  // ============================================================================

  /**
   * Request exclusive control of the robot.
   * Only the control holder can send teleop commands.
   */
  requestControl(): boolean {
    return this.sendRaw({
      type: 'control',
      robotId: this.robotId,
      action: 'request'
    });
  }

  /**
   * Release control of the robot.
   */
  releaseControl(): boolean {
    return this.sendRaw({
      type: 'control',
      robotId: this.robotId,
      action: 'release'
    });
  }

  /**
   * Force-take control from another client.
   */
  forceControl(): boolean {
    return this.sendRaw({
      type: 'control',
      robotId: this.robotId,
      action: 'force'
    });
  }

  /**
   * Check if this client currently holds control.
   */
  hasControl(): boolean {
    return this.controlHolder === this.clientName;
  }

  /**
   * Get the current control holder.
   */
  getControlHolder(): string | null {
    return this.controlHolder;
  }

  // ============================================================================
  // Command Methods
  // ============================================================================

  sendCommand(command: string, payload: Record<string, any> = {}): boolean {
    if (!this.connected || !this.ws) {
      console.error('[Cloud] Cannot send command: not connected');
      return false;
    }

    const message: CommandMessage = {
      type: 'command',
      robotId: this.robotId,
      command,
      payload
    };

    this.ws.send(JSON.stringify(message));
    console.log(`[Cloud] Sent command: ${command}`, payload);
    return true;
  }

  /**
   * Send teleop velocity command (requires control lock).
   */
  sendTeleop(linearX: number, angularZ: number): boolean {
    return this.sendCommand('teleop', {
      kind: 'vel',
      linear_x: linearX,
      angular_z: angularZ
    });
  }

  // Command helpers
  
  startSlam(): boolean {
    return this.sendCommand('start_slam');
  }

  stopSlam(mapName: string): boolean {
    return this.sendCommand('stop_slam', { map_name: mapName });
  }

  loadMap(mapName: string): boolean {
    return this.sendCommand('load_map', { map_name: mapName });
  }

  setMode(mode: 'slam' | 'localization' | 'idle'): boolean {
    return this.sendCommand('set_mode', { mode });
  }

  cancelNavigation(): boolean {
    return this.sendCommand('cancel_nav');
  }

  emergencyStop(): boolean {
    return this.sendCommand('stop');
  }

  goToPoi(poiId: string): boolean {
    return this.sendCommand('go_to_poi', { poi_id: poiId });
  }

  restart(): boolean {
    return this.sendCommand('restart');
  }

  // ============================================================================
  // Event System
  // ============================================================================

  on(event: string, callback: (data: any) => void): void {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set());
    }
    this.listeners.get(event)!.add(callback);
  }

  off(event: string, callback: (data: any) => void): void {
    const listeners = this.listeners.get(event);
    if (listeners) {
      listeners.delete(callback);
    }
  }

  private emit(event: string, data: any): void {
    const listeners = this.listeners.get(event);
    if (listeners) {
      listeners.forEach(callback => callback(data));
    }
  }

  // ============================================================================
  // Connection Management
  // ============================================================================

  private startPingInterval(): void {
    this.stopPingInterval();
    this.pingInterval = window.setInterval(() => {
      if (this.connected && this.ws) {
        this.ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, 30000); // Ping every 30 seconds
  }

  private stopPingInterval(): void {
    if (this.pingInterval) {
      clearInterval(this.pingInterval);
      this.pingInterval = undefined;
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
    }

    this.reconnectTimer = window.setTimeout(() => {
      console.log('[Cloud] Attempting to reconnect...');
      this.connect().catch(error => {
        console.error('[Cloud] Reconnection failed:', error);
      });
    }, this.reconnectInterval);
  }

  disconnect(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
    }
    this.stopPingInterval();

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    this.connected = false;
    this.robotState = null;
    this.controlHolder = null;
  }

  isConnected(): boolean {
    return this.connected;
  }

  isRobotOnline(): boolean {
    return this.robotState?.online ?? false;
  }

  getRobotState(): RobotState | null {
    return this.robotState;
  }

  getServerConfig(): WelcomeMessage['config'] | null {
    return this.serverConfig;
  }

  getUrl(): string {
    return this.url;
  }

  getRobotId(): string {
    return this.robotId;
  }
}

// ============================================================================
// Singleton Factory
// ============================================================================

let cloudClient: CloudWebSocketClient | null = null;

export function getCloudClient(url?: string, robotId?: string, clientName?: string): CloudWebSocketClient {
  if (!cloudClient) {
    const backendUrl = url || import.meta.env.VITE_CLOUD_BACKEND_URL || 'ws://localhost:8080/ui';
    const robot = robotId || import.meta.env.VITE_ROBOT_ID || 'fordward';
    const name = clientName || `web-${Math.random().toString(36).substring(7)}`;
    cloudClient = new CloudWebSocketClient(backendUrl, robot, name);
  }
  return cloudClient;
}

export function resetCloudClient(): void {
  if (cloudClient) {
    cloudClient.disconnect();
    cloudClient = null;
  }
}
