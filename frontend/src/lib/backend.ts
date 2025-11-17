export function backendBase(): string {
  const host = typeof window !== 'undefined' ? window.location.hostname : 'localhost'
  return (import.meta as any).env?.VITE_BACKEND_BASE || `http://${host}:8000`
}

async function api<T>(path: string, opts: RequestInit = {}): Promise<T> {
  const res = await fetch(`${backendBase()}${path}`, {
    method: opts.method || 'GET',
    headers: { 'Content-Type': 'application/json', ...(opts.headers || {}) },
    body: opts.body,
  })
  if (!res.ok) throw new Error(`${res.status} ${res.statusText}`)
  return res.json() as Promise<T>
}

export type ModeState = 'IDLE' | 'SLAM' | 'LOCALIZATION'
export type ModeStatus = { state: ModeState; slam_running: boolean; localization_running: boolean }

export const getModeStatus = () => api<ModeStatus>('/mode/status')
export const startSlam = () => api<ModeStatus>('/mode/slam/start', { method: 'POST' })
export const stopSlam = () => api<ModeStatus>('/mode/slam/stop', { method: 'POST' })
export const saveMapAndExitSlam = (name?: string) => api<any>('/mode/slam/save_exit', { method: 'POST', body: JSON.stringify({ name }) })
export const startLocalization = () => api<ModeStatus>('/mode/localization/start', { method: 'POST' })
export const stopLocalization = () => api<ModeStatus>('/mode/localization/stop', { method: 'POST' })
export const reloadMaps = () => api<any>('/maps/reload', { method: 'POST' })

