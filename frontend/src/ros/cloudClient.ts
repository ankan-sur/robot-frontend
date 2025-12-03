/**
 * Cloud WebSocket Client
 * 
 * Connects to cloud backend instead of direct rosbridge connection.
 * Handles telemetry updates and command forwarding.
 */

export interface CloudTelemetry {
  type: 'telemetry';
  robot_id: string;
  timestamp: string;
  pose?: {
    x: number;
    y: number;
    z: number;
    theta: number;
  };
  battery?: number;
  battery_voltage?: number;
  state: string;
  map?: string;
  available_maps?: string[];
  nav_active?: boolean;
  connected_clients?: number;
}

export interface CloudCommand {
  type: 'command';
  robot_id?: string;
  command: string;
  [key: string]: any;
}

export interface CloudCommandResult {
  type: 'command_result';
  robot_id: string;
  command: string;
  success: boolean;
  message: string;
  timestamp: string;
}

export interface CloudInitMessage {
  type: 'init';
  robots: Array<{
    robot_id: string;
    connected: boolean;
    lastSeen: number;
    telemetry: CloudTelemetry;
  }>;
  timestamp: string;
}

type CloudMessage = CloudTelemetry | CloudCommandResult | CloudInitMessage | { type: 'pong' } | { type: 'error'; error: string };

export class CloudWebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private robotId: string;
  private reconnectInterval: number;
  private reconnectTimer?: number;
  private listeners: Map<string, Set<(data: any) => void>>;
  private connected: boolean = false;

  constructor(url: string, robotId: string = 'mentorpi', reconnectInterval: number = 5000) {
    this.url = url;
    this.robotId = robotId;
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
          resolve();
          
          // Start ping interval
          this.startPingInterval();
        };

        this.ws.onmessage = (event) => {
          try {
            const message: CloudMessage = JSON.parse(event.data);
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
          this.emit('close', {});
          this.scheduleReconnect();
        };
      } catch (error) {
        reject(error);
      }
    });
  }

  private handleMessage(message: CloudMessage) {
    switch (message.type) {
      case 'telemetry':
        this.emit('telemetry', message);
        break;
      
      case 'command_result':
        this.emit('command_result', message);
        break;
      
      case 'init':
        this.emit('init', message);
        break;
      
      case 'pong':
        // Ping response
        break;
      
      case 'error':
        console.error('[Cloud] Backend error:', message.error);
        this.emit('error', message);
        break;
      
      default:
        console.warn('[Cloud] Unknown message type:', (message as any).type);
    }
  }

  sendCommand(command: string, params: Record<string, any> = {}): void {
    if (!this.connected || !this.ws) {
      console.error('[Cloud] Cannot send command: not connected');
      return;
    }

    const message: CloudCommand = {
      type: 'command',
      robot_id: this.robotId,
      command,
      ...params
    };

    this.ws.send(JSON.stringify(message));
    console.log(`[Cloud] Sent command: ${command}`, params);
  }

  // Command helpers
  
  startSlam(): void {
    this.sendCommand('start_slam');
  }

  stopSlam(mapName: string): void {
    this.sendCommand('stop_slam', { map_name: mapName });
  }

  loadMap(mapName: string): void {
    this.sendCommand('load_map', { map_name: mapName });
  }

  setMode(mode: 'slam' | 'localization' | 'idle'): void {
    this.sendCommand('set_mode', { mode });
  }

  cancelNavigation(): void {
    this.sendCommand('cancel_nav');
  }

  emergencyStop(): void {
    this.sendCommand('stop');
  }

  goToPoi(poiId: string): void {
    this.sendCommand('go_to_poi', { poi_id: poiId });
  }

  restart(): void {
    this.sendCommand('restart');
  }

  // Event system

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

  // Connection management

  private startPingInterval(): void {
    setInterval(() => {
      if (this.connected && this.ws) {
        this.ws.send(JSON.stringify({ type: 'ping' }));
      }
    }, 30000); // Ping every 30 seconds
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

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    this.connected = false;
  }

  isConnected(): boolean {
    return this.connected;
  }

  getUrl(): string {
    return this.url;
  }
}

// Singleton instance
let cloudClient: CloudWebSocketClient | null = null;

export function getCloudClient(url?: string, robotId?: string): CloudWebSocketClient {
  if (!cloudClient) {
    const backendUrl = url || import.meta.env.VITE_CLOUD_BACKEND_URL || 'ws://localhost:8080/ui';
    const robot = robotId || import.meta.env.VITE_ROBOT_ID || 'mentorpi';
    cloudClient = new CloudWebSocketClient(backendUrl, robot);
  }
  return cloudClient;
}
