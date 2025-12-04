import React from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App'
import { DemoApp, isDemoMode } from './demo/DemoEntry'
import { isCloudModeEnabled, setConnectionState } from './ros/ros'
import { getCloudClient } from './ros/cloudClient'

// Check if we should render demo mode
const demoMode = isDemoMode()

// Initialize cloud mode connection if enabled (skip in demo mode)
if (!demoMode && isCloudModeEnabled()) {
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

// Log which mode we're in
if (demoMode) {
  console.log('[Main] 🎬 Demo mode enabled - rendering Demo UI')
} else {
  console.log('[Main] Standard mode - rendering full App')
}

createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    {demoMode ? <DemoApp /> : <App />}
  </React.StrictMode>
)




