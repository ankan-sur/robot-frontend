import React from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App'
import AppDemo from './demo/AppDemo'
import { isCloudModeEnabled, setConnectionState } from './ros/ros'
import { getCloudClient } from './ros/cloudClient'

// Check if demo mode is enabled
const isDemoMode = import.meta.env.VITE_DEMO_MODE === 'true'

// Initialize cloud mode connection if enabled (and not in demo mode)
if (isCloudModeEnabled() && !isDemoMode) {
  console.log('[Main] Cloud mode enabled, initializing cloud client...')
  const client = getCloudClient()
  
  // Set up event handlers to bridge cloud state to ROS connection state
  client.on('connection', () => {
    console.log('[Main] Cloud client connected')
  })
  
  client.on('state', (msg: any) => {
    if (msg.state?.online) {
      setConnectionState('connected')
    } else {
      setConnectionState('disconnected')
    }
  })
  
  client.on('robot_connected', () => {
    setConnectionState('connected')
  })
  
  client.on('robot_disconnected', () => {
    setConnectionState('disconnected')
  })
  
  client.on('close', () => {
    setConnectionState('disconnected')
  })
  
  client.on('error', () => {
    setConnectionState('error')
  })
  
  // Connect (don't block render)
  client.connect().catch((err) => {
    console.warn('[Main] Cloud client initial connection failed:', err)
    // Don't crash - UI will show disconnected state
  })
}

// Choose component based on mode
const RootComponent = isDemoMode ? AppDemo : App
console.log('[Main] Mode:', isDemoMode ? 'DEMO' : (isCloudModeEnabled() ? 'CLOUD' : 'LAN'))

createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <RootComponent />
  </React.StrictMode>
)




