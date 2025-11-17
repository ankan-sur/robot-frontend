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

export async function setMode(mode: 'slam' | 'localization' | 'idle'): Promise<any> {
  return callService(ROS_CONFIG.services.setMode, 'interfaces/SetString', { data: mode })
}

export async function startSlam(): Promise<any> {
  return callService(ROS_CONFIG.services.startSlam, 'std_srvs/Trigger', {})
}

export async function stopSlamAndSave(name?: string): Promise<any> {
  return callService(ROS_CONFIG.services.stopSlamAndSave, 'interfaces/SetString', { data: name || '' })
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

