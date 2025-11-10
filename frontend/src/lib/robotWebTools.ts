import EventEmitter2 from 'eventemitter2'
import ROSLIB from 'roslib'

let ros2dPromise: Promise<any> | null = null
let createjsPromise: Promise<void> | null = null

const ensureGlobals = () => {
  const globalAny = globalThis as any
  if (!globalAny.EventEmitter2) {
    globalAny.EventEmitter2 = EventEmitter2
  }
  if (!globalAny.ROSLIB) {
    globalAny.ROSLIB = ROSLIB
  }
}

const loadScript = (url: string) =>
  new Promise<void>((resolve, reject) => {
    const script = document.createElement('script')
    script.src = url
    script.async = true
    script.onload = () => resolve()
    script.onerror = () => reject(new Error(`Failed to load ${url}`))
    document.body.appendChild(script)
  })

const loadLegacyCreatejs = () => {
  const globalAny = globalThis as any
  if (globalAny.createjs?.Ticker?.setFPS) {
    return Promise.resolve()
  }
  if (!createjsPromise) {
    createjsPromise = loadScript('https://static.robotwebtools.org/EaselJS/current/easeljs.js').then(() => {
      if (!globalAny.createjs) {
        throw new Error('Failed to load legacy createjs')
      }
    })
  }
  return createjsPromise
}

export async function loadRos2d() {
  ensureGlobals()
  const globalAny = globalThis as any
  if (globalAny.ROS2D) {
    return globalAny.ROS2D
  }
  if (!ros2dPromise) {
    ros2dPromise = loadLegacyCreatejs()
      .then(() => import('ros2d/build/ros2d.js?url'))
      .then(({ default: url }) => loadScript(url))
      .then(() => globalAny.ROS2D)
  }
  return ros2dPromise
}
