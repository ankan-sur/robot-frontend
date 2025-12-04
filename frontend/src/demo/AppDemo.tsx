/**
 * Demo Day UI - AppDemo.tsx
 * 
 * Minimal, polished single-robot control panel for Demo Day video filming.
 * Features:
 * - Clean header with robot name and connection status
 * - Map area with POI markers
 * - Big touch-friendly POI navigation buttons
 * - Live status display and activity log
 * - Green color theme
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { getDemoClient, DEMO_CONFIG, DEMO_POIS, DemoNavState, DemoRobotStatus, DemoControlClient, POI } from './index';

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

interface LogEntry {
  id: number;
  timestamp: Date;
  message: string;
  type: 'info' | 'success' | 'error' | 'nav';
}

// ─────────────────────────────────────────────────────────────────────────────
// Color Theme (Green)
// ─────────────────────────────────────────────────────────────────────────────

const THEME = {
  primary: '#10B981',       // Emerald 500
  primaryDark: '#059669',   // Emerald 600
  primaryLight: '#34D399',  // Emerald 400
  background: '#0F172A',    // Slate 900
  surface: '#1E293B',       // Slate 800
  surfaceLight: '#334155',  // Slate 700
  text: '#F8FAFC',          // Slate 50
  textMuted: '#94A3B8',     // Slate 400
  success: '#22C55E',       // Green 500
  warning: '#F59E0B',       // Amber 500
  error: '#EF4444',         // Red 500
  border: '#475569',        // Slate 600
};

// ─────────────────────────────────────────────────────────────────────────────
// Status Badge Component
// ─────────────────────────────────────────────────────────────────────────────

interface StatusBadgeProps {
  label: string;
  value: string;
  status: 'ok' | 'warning' | 'error' | 'neutral';
}

const StatusBadge: React.FC<StatusBadgeProps> = ({ label, value, status }) => {
  const statusColors = {
    ok: THEME.success,
    warning: THEME.warning,
    error: THEME.error,
    neutral: THEME.textMuted,
  };

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      padding: '12px 16px',
      backgroundColor: THEME.surface,
      borderRadius: '8px',
      minWidth: '100px',
    }}>
      <span style={{ color: THEME.textMuted, fontSize: '12px', marginBottom: '4px' }}>
        {label}
      </span>
      <span style={{ 
        color: statusColors[status], 
        fontSize: '16px', 
        fontWeight: 600,
        display: 'flex',
        alignItems: 'center',
        gap: '6px',
      }}>
        <span style={{
          width: '8px',
          height: '8px',
          borderRadius: '50%',
          backgroundColor: statusColors[status],
        }} />
        {value}
      </span>
    </div>
  );
};

// ─────────────────────────────────────────────────────────────────────────────
// POI Button Component
// ─────────────────────────────────────────────────────────────────────────────

interface POIButtonProps {
  poi: POI;
  isActive: boolean;
  isDisabled: boolean;
  onClick: () => void;
}

const POIButton: React.FC<POIButtonProps> = ({ poi, isActive, isDisabled, onClick }) => {
  const baseStyle: React.CSSProperties = {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    padding: '20px 24px',
    minWidth: '140px',
    minHeight: '80px',
    border: 'none',
    borderRadius: '12px',
    cursor: isDisabled ? 'not-allowed' : 'pointer',
    transition: 'all 0.2s ease',
    fontFamily: 'inherit',
  };

  const getStyle = (): React.CSSProperties => {
    if (isActive) {
      return {
        ...baseStyle,
        backgroundColor: THEME.primary,
        color: THEME.text,
        boxShadow: `0 0 20px ${THEME.primary}40`,
      };
    }
    if (isDisabled) {
      return {
        ...baseStyle,
        backgroundColor: THEME.surfaceLight,
        color: THEME.textMuted,
        opacity: 0.6,
      };
    }
    return {
      ...baseStyle,
      backgroundColor: THEME.surface,
      color: THEME.text,
      border: `2px solid ${THEME.primary}`,
    };
  };

  return (
    <button 
      style={getStyle()} 
      onClick={onClick}
      disabled={isDisabled}
    >
      <span style={{ fontSize: '24px', marginBottom: '4px' }}>{poi.icon}</span>
      <span style={{ fontSize: '16px', fontWeight: 600 }}>{poi.label}</span>
    </button>
  );
};

// ─────────────────────────────────────────────────────────────────────────────
// Map Display Component
// ─────────────────────────────────────────────────────────────────────────────

interface MapDisplayProps {
  pois: POI[];
  currentDestination: POI | null;
  navState: DemoNavState;
}

const MapDisplay: React.FC<MapDisplayProps> = ({ pois, currentDestination, navState }) => {
  // Simple placeholder map with POI markers
  return (
    <div style={{
      position: 'relative',
      width: '100%',
      height: '300px',
      backgroundColor: THEME.surfaceLight,
      borderRadius: '12px',
      border: `2px solid ${THEME.border}`,
      overflow: 'hidden',
    }}>
      {/* Map placeholder */}
      <div style={{
        position: 'absolute',
        inset: 0,
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        color: THEME.textMuted,
        fontSize: '14px',
      }}>
        <div style={{ textAlign: 'center' }}>
          <div style={{ fontSize: '48px', marginBottom: '8px' }}>🗺️</div>
          <div>{DEMO_CONFIG.mapName}</div>
        </div>
      </div>

      {/* POI markers */}
      {pois.map((poi, index) => {
        const isDestination = currentDestination?.id === poi.id;
        const isNavigating = isDestination && navState === 'navigating';
        
        // Arrange POIs in a row for visualization
        const xPos = 20 + (index * 30);
        const yPos = 50;
        
        return (
          <div
            key={poi.id}
            style={{
              position: 'absolute',
              left: `${xPos}%`,
              top: `${yPos}%`,
              transform: 'translate(-50%, -50%)',
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              transition: 'all 0.3s ease',
            }}
          >
            <div style={{
              width: '40px',
              height: '40px',
              borderRadius: '50%',
              backgroundColor: isDestination ? THEME.primary : THEME.surface,
              border: `3px solid ${isDestination ? THEME.primaryLight : THEME.border}`,
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: '20px',
              boxShadow: isNavigating ? `0 0 15px ${THEME.primary}` : 'none',
              animation: isNavigating ? 'pulse 1.5s infinite' : 'none',
            }}>
              {poi.icon}
            </div>
            <span style={{
              marginTop: '4px',
              fontSize: '11px',
              color: isDestination ? THEME.primaryLight : THEME.textMuted,
              fontWeight: isDestination ? 600 : 400,
            }}>
              {poi.label}
            </span>
          </div>
        );
      })}

      {/* Robot indicator */}
      <div style={{
        position: 'absolute',
        left: '10%',
        top: '70%',
        transform: 'translate(-50%, -50%)',
      }}>
        <div style={{
          width: '32px',
          height: '32px',
          borderRadius: '50%',
          backgroundColor: THEME.primary,
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          fontSize: '18px',
          boxShadow: `0 0 10px ${THEME.primary}`,
        }}>
          🤖
        </div>
        <span style={{
          display: 'block',
          textAlign: 'center',
          marginTop: '2px',
          fontSize: '10px',
          color: THEME.primaryLight,
        }}>
          {DEMO_CONFIG.robotName}
        </span>
      </div>

      {/* CSS animation */}
      <style>{`
        @keyframes pulse {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.6; }
        }
      `}</style>
    </div>
  );
};

// ─────────────────────────────────────────────────────────────────────────────
// Activity Log Component
// ─────────────────────────────────────────────────────────────────────────────

interface ActivityLogProps {
  logs: LogEntry[];
}

const ActivityLog: React.FC<ActivityLogProps> = ({ logs }) => {
  const logRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (logRef.current) {
      logRef.current.scrollTop = logRef.current.scrollHeight;
    }
  }, [logs]);

  const getLogColor = (type: LogEntry['type']) => {
    switch (type) {
      case 'success': return THEME.success;
      case 'error': return THEME.error;
      case 'nav': return THEME.primary;
      default: return THEME.textMuted;
    }
  };

  return (
    <div style={{
      backgroundColor: THEME.surface,
      borderRadius: '8px',
      padding: '12px',
      height: '150px',
      overflow: 'hidden',
      display: 'flex',
      flexDirection: 'column',
    }}>
      <div style={{ 
        color: THEME.textMuted, 
        fontSize: '12px', 
        marginBottom: '8px',
        fontWeight: 600,
      }}>
        Activity Log
      </div>
      <div 
        ref={logRef}
        style={{
          flex: 1,
          overflowY: 'auto',
          fontFamily: 'monospace',
          fontSize: '12px',
        }}
      >
        {logs.map((log) => (
          <div key={log.id} style={{ 
            marginBottom: '4px',
            color: getLogColor(log.type),
          }}>
            <span style={{ color: THEME.textMuted }}>
              [{log.timestamp.toLocaleTimeString()}]
            </span>{' '}
            {log.message}
          </div>
        ))}
        {logs.length === 0 && (
          <div style={{ color: THEME.textMuted, fontStyle: 'italic' }}>
            No activity yet...
          </div>
        )}
      </div>
    </div>
  );
};

// ─────────────────────────────────────────────────────────────────────────────
// Cancel Button Component
// ─────────────────────────────────────────────────────────────────────────────

interface CancelButtonProps {
  onClick: () => void;
  disabled: boolean;
}

const CancelButton: React.FC<CancelButtonProps> = ({ onClick, disabled }) => (
  <button
    onClick={onClick}
    disabled={disabled}
    style={{
      padding: '12px 32px',
      backgroundColor: disabled ? THEME.surfaceLight : THEME.error,
      color: THEME.text,
      border: 'none',
      borderRadius: '8px',
      fontSize: '14px',
      fontWeight: 600,
      cursor: disabled ? 'not-allowed' : 'pointer',
      opacity: disabled ? 0.5 : 1,
      transition: 'all 0.2s ease',
      fontFamily: 'inherit',
    }}
  >
    ⏹ Cancel Navigation
  </button>
);

// ─────────────────────────────────────────────────────────────────────────────
// Main AppDemo Component
// ─────────────────────────────────────────────────────────────────────────────

const AppDemo: React.FC = () => {
  const [client, setClient] = useState<DemoControlClient | null>(null);
  const [status, setStatus] = useState<DemoRobotStatus | null>(null);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const logIdRef = useRef(0);

  // ── Logging helper ────────────────────────────────────────────────────────
  const addLog = useCallback((message: string, type: LogEntry['type'] = 'info') => {
    logIdRef.current += 1;
    setLogs(prev => [...prev.slice(-50), { // Keep last 50 entries
      id: logIdRef.current,
      timestamp: new Date(),
      message,
      type,
    }]);
  }, []);

  // ── Initialize client ─────────────────────────────────────────────────────
  useEffect(() => {
    const demoClient = getDemoClient();
    setClient(demoClient);

    // Subscribe to status updates
    const unsubscribe = demoClient.onStatusUpdate((newStatus) => {
      setStatus(newStatus);
    });

    // Connect
    addLog('Connecting to robot...', 'info');
    demoClient.connect()
      .then(() => {
        addLog('Connected to robot', 'success');
      })
      .catch((err) => {
        addLog(`Connection failed: ${err.message}`, 'error');
      });

    return () => {
      unsubscribe();
      demoClient.disconnect();
    };
  }, [addLog]);

  // ── Handle navigation ─────────────────────────────────────────────────────
  const handleNavigateToPOI = useCallback(async (poi: POI) => {
    if (!client) return;

    addLog(`Sending robot to ${poi.label}...`, 'nav');
    
    try {
      await client.navigateTo(poi);
      addLog(`Arrived at ${poi.label}!`, 'success');
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Unknown error';
      addLog(`Navigation failed: ${message}`, 'error');
    }
  }, [client, addLog]);

  // ── Handle cancel ─────────────────────────────────────────────────────────
  const handleCancel = useCallback(async () => {
    if (!client) return;

    addLog('Cancelling navigation...', 'info');
    
    try {
      await client.cancelNavigation();
      addLog('Navigation cancelled', 'info');
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Unknown error';
      addLog(`Cancel failed: ${message}`, 'error');
    }
  }, [client, addLog]);

  // ── Derive UI state ───────────────────────────────────────────────────────
  const isConnected = status?.connected ?? false;
  const navState = status?.navState ?? 'idle';
  const currentDestination = status?.currentDestination ?? null;
  const isNavigating = navState === 'navigating' || navState === 'sending';
  const batteryLevel = status?.batteryPercent ?? 100;

  const getNavStateLabel = () => {
    switch (navState) {
      case 'idle': return 'Idle';
      case 'sending': return 'Sending Goal...';
      case 'navigating': return 'Navigating';
      case 'arrived': return 'Arrived';
      case 'cancelled': return 'Cancelled';
      case 'failed': return 'Failed';
      default: return 'Unknown';
    }
  };

  const getNavStateStatus = (): StatusBadgeProps['status'] => {
    switch (navState) {
      case 'arrived': return 'ok';
      case 'failed': return 'error';
      case 'cancelled': return 'warning';
      default: return 'neutral';
    }
  };

  // ── Render ────────────────────────────────────────────────────────────────
  return (
    <div style={{
      minHeight: '100vh',
      backgroundColor: THEME.background,
      color: THEME.text,
      fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
    }}>
      {/* Header */}
      <header style={{
        backgroundColor: THEME.surface,
        borderBottom: `1px solid ${THEME.border}`,
        padding: '16px 24px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
      }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
          <span style={{ fontSize: '28px' }}>🤖</span>
          <div>
            <h1 style={{ 
              margin: 0, 
              fontSize: '20px', 
              fontWeight: 700,
              color: THEME.primary,
            }}>
              {DEMO_CONFIG.robotName}
            </h1>
            <div style={{ 
              fontSize: '12px', 
              color: THEME.textMuted,
              marginTop: '2px',
            }}>
              Demo Control Panel
            </div>
          </div>
        </div>
        <div style={{
          display: 'flex',
          alignItems: 'center',
          gap: '8px',
          padding: '8px 16px',
          backgroundColor: isConnected ? `${THEME.success}20` : `${THEME.error}20`,
          borderRadius: '20px',
        }}>
          <span style={{
            width: '10px',
            height: '10px',
            borderRadius: '50%',
            backgroundColor: isConnected ? THEME.success : THEME.error,
          }} />
          <span style={{ 
            color: isConnected ? THEME.success : THEME.error,
            fontSize: '14px',
            fontWeight: 500,
          }}>
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </header>

      {/* Main Content */}
      <main style={{
        maxWidth: '900px',
        margin: '0 auto',
        padding: '24px',
      }}>
        {/* Status Row */}
        <div style={{
          display: 'flex',
          gap: '12px',
          marginBottom: '24px',
          flexWrap: 'wrap',
          justifyContent: 'center',
        }}>
          <StatusBadge 
            label="Connection" 
            value={isConnected ? 'Online' : 'Offline'}
            status={isConnected ? 'ok' : 'error'}
          />
          <StatusBadge 
            label="Battery" 
            value={`${batteryLevel}%`}
            status={batteryLevel > 20 ? 'ok' : batteryLevel > 10 ? 'warning' : 'error'}
          />
          <StatusBadge 
            label="Nav State" 
            value={getNavStateLabel()}
            status={getNavStateStatus()}
          />
          <StatusBadge 
            label="Destination" 
            value={currentDestination?.label ?? 'None'}
            status={currentDestination ? 'ok' : 'neutral'}
          />
        </div>

        {/* Map Display */}
        <div style={{ marginBottom: '24px' }}>
          <MapDisplay 
            pois={DEMO_POIS}
            currentDestination={currentDestination}
            navState={navState}
          />
        </div>

        {/* POI Navigation Buttons */}
        <div style={{
          backgroundColor: THEME.surface,
          borderRadius: '12px',
          padding: '20px',
          marginBottom: '24px',
        }}>
          <div style={{ 
            color: THEME.textMuted, 
            fontSize: '12px', 
            marginBottom: '16px',
            fontWeight: 600,
            textTransform: 'uppercase',
            letterSpacing: '0.05em',
          }}>
            Navigate To
          </div>
          <div style={{
            display: 'flex',
            gap: '16px',
            flexWrap: 'wrap',
            justifyContent: 'center',
          }}>
            {DEMO_POIS.map((poi) => (
              <POIButton
                key={poi.id}
                poi={poi}
                isActive={currentDestination?.id === poi.id}
                isDisabled={!isConnected || isNavigating}
                onClick={() => handleNavigateToPOI(poi)}
              />
            ))}
          </div>

          {/* Cancel Button */}
          <div style={{ 
            marginTop: '20px', 
            display: 'flex', 
            justifyContent: 'center',
          }}>
            <CancelButton 
              onClick={handleCancel}
              disabled={!isConnected || !isNavigating}
            />
          </div>
        </div>

        {/* Activity Log */}
        <ActivityLog logs={logs} />

        {/* Footer info */}
        <div style={{
          marginTop: '24px',
          textAlign: 'center',
          color: THEME.textMuted,
          fontSize: '12px',
        }}>
          {DEMO_CONFIG.useMockClient ? (
            <span>🧪 Running in mock mode</span>
          ) : (
            <span>📡 Connected to ROS2</span>
          )}
          {' • '}
          Map: {DEMO_CONFIG.mapName}
        </div>
      </main>
    </div>
  );
};

export default AppDemo;
