import * as createjs from '@createjs/easeljs'
import * as THREE from 'three'
import EventEmitter2 from 'eventemitter2'
import ROSLIB from 'roslib'

const ensureBaseGlobals = () => {
  if (!(globalThis as any).createjs) {
    ;(globalThis as any).createjs = createjs
  }
  if (!(globalThis as any).THREE) {
    ;(globalThis as any).THREE = THREE
  }
  if (!(globalThis as any).EventEmitter2) {
    ;(globalThis as any).EventEmitter2 = EventEmitter2
  }
  if (!(globalThis as any).ROSLIB) {
    ;(globalThis as any).ROSLIB = ROSLIB
  }
}

const loadScript = (url: string) =>
  new Promise<void>((resolve, reject) => {
    const existing = document.querySelector<HTMLScriptElement>(`script[src="${url}"]`)
    if (existing) {
      if (existing.dataset.loaded === 'true') {
        resolve()
        return
      }
      existing.addEventListener('load', () => resolve(), { once: true })
      existing.addEventListener('error', () => reject(new Error(`Failed to load ${url}`)), { once: true })
      return
    }

    const script = document.createElement('script')
    script.src = url
    script.async = true
    script.dataset.loaded = 'false'
    script.onload = () => {
      script.dataset.loaded = 'true'
      resolve()
    }
    script.onerror = () => reject(new Error(`Failed to load ${url}`))
    document.body.appendChild(script)
  })

let ros2dPromise: Promise<any> | null = null
let ros3dPromise: Promise<any> | null = null

export async function getRos2d() {
  ensureBaseGlobals()
  if ((globalThis as any).ROS2D) {
    return (globalThis as any).ROS2D
  }
  if (!ros2dPromise) {
    ros2dPromise = import('ros2d/build/ros2d.js?url').then(({ default: url }) =>
      loadScript(url).then(() => (globalThis as any).ROS2D)
    )
  }
  return ros2dPromise
}

export async function getRos3d() {
  ensureBaseGlobals()
  if ((globalThis as any).ROS3D) {
    return (globalThis as any).ROS3D
  }
  if (!ros3dPromise) {
    ros3dPromise = import('ros3d/build/ros3d.js?url').then(({ default: url }) =>
      loadScript(url).then(() => (globalThis as any).ROS3D)
    )
  }
  return ros3dPromise
}
