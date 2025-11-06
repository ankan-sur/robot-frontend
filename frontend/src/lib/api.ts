export async function apiMove(base: string, direction: string, speed?: number, token?: string) {
  const payload: any = { direction }
  if (speed != null) payload.speed = speed
  const res = await fetch(`${base}/move`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
    },
    body: JSON.stringify(payload),
  })
  if (!res.ok) throw new Error('Move failed')
  return res.json()
}

export async function apiStop(base: string, token?: string) {
  const res = await fetch(`${base}/stop`, {
    method: 'POST',
    headers: {
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
    },
  })
  if (!res.ok) throw new Error('Stop failed')
  return res.json()
}

export async function apiStatus(base: string) {
  const res = await fetch(`${base}/status`)
  if (!res.ok) throw new Error('Status failed')
  return res.json()
}

