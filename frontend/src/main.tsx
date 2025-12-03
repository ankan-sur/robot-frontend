import React from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App'
import { isCloudModeEnabled, setConnectionState } from './ros/ros'
import { getCloudClient } from './ros/cloudClient'

// Initialize cloud mode connection if enabled
if (isCloudModeEnabled()) {
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

createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
)




