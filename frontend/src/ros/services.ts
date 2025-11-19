import ROSLIB from 'roslib'
import { ros } from './ros'
import { ROS_CONFIG } from './config'

export function callService<T = any>(name: string, serviceType: string, request: any): Promise<T> {
  return new Promise<T>((resolve, reject) => {
    try {
      const svc = new ROSLIB.Service({ ros, name, serviceType })
      const req = new ROSLIB.ServiceRequest(request || {})
      svc.callService(req, (res: any) => resolve(res as T), (err: any) => reject(err))
    } catch (e) {
      reject(e)
    }
  })
}

function parseMaybeJson(anyVal: any): any {
  const s = typeof anyVal === 'string' ? anyVal : (typeof anyVal?.message === 'string' ? anyVal.message : anyVal?.data)
  if (typeof s === 'string') { try { return JSON.parse(s) } catch { return s } }
  return anyVal
}

export async function getMode(): Promise<{ mode?: string; map?: string; msg?: string } | null> {
  const res = await callService<any>(ROS_CONFIG.services.getMode, 'std_srvs/Trigger', {})
  return parseMaybeJson(res) || null
}

export async function setMode(mode: string): Promise<any> {
  // setMode accepts arbitrary mode string for flexibility (e.g. 'slam','localization','idle')
  return callService(ROS_CONFIG.services.setMode, 'interfaces/SetString', { data: mode })
}

export async function stopSlamAndSave(mapName: string): Promise<any> {
  return callService(ROS_CONFIG.services.stopSlamAndSave, 'interfaces/SetString', { data: mapName })
}

export async function loadMap(name: string): Promise<any> {
  return callService(ROS_CONFIG.services.loadMap, 'interfaces/SetString', { data: name })
}

export async function listMaps(): Promise<string[]> {
  const res = await callService<any>(ROS_CONFIG.services.listMaps, 'std_srvs/Trigger', {})
  const parsed = parseMaybeJson(res)
  if (Array.isArray(parsed)) return parsed as string[]
  if (Array.isArray(parsed?.maps)) return parsed.maps as string[]
  if (typeof parsed === 'object' && typeof parsed?.message === 'string') {
    try { const arr = JSON.parse(parsed.message); if (Array.isArray(arr)) return arr } catch {}
  }
  return []
}

export async function cancelNavigation(goalId?: number[] | string): Promise<any> {
  // If a goalId (UUID) is provided, try to send it; otherwise, request cancel all
  let request: any = {}
  if (Array.isArray(goalId)) {
    request = { goal_info: { goal_id: { uuid: goalId } } }
  } else if (typeof goalId === 'string') {
    // Convert hex string to byte array if possible (strip dashes)
    const hex = goalId.replace(/-/g, '')
    const bytes: number[] = []
    for (let i = 0; i < hex.length; i += 2) {
      bytes.push(parseInt(hex.slice(i, i + 2), 16))
    }
    request = { goal_info: { goal_id: { uuid: bytes } } }
  }
  try {
    return await callService(ROS_CONFIG.services.navCancel, 'action_msgs/CancelGoal', request)
  } catch (e) {
    // Some stacks use alternate name; best-effort fallback
    return await callService('/navigate_to_pose/_action/cancel_goal', 'action_msgs/CancelGoal', request)
  }
}

// Teleop emergency stop services
export async function teleopEStop(): Promise<any> {
  return callService(ROS_CONFIG.services.teleopEStop, 'std_srvs/Trigger', {})
}

export async function teleopClearEStop(): Promise<any> {
  return callService(ROS_CONFIG.services.teleopClearEStop, 'std_srvs/Trigger', {})
}
