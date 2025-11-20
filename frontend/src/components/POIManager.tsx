import { useState, useEffect } from 'react'
import { usePointsOfInterest, useRobotPose } from '../ros/hooks'
import { markPOI, deletePOI, renamePOI } from '../ros/services'
import type { POI } from '../ros/services'

export function POIManager() {
  const pois = usePointsOfInterest()
  const robotPose = useRobotPose()
  const [statusMsg, setStatusMsg] = useState<string | null>(null)
  const [operating, setOperating] = useState(false)
  const [editingPoi, setEditingPoi] = useState<string | null>(null)
  const [newName, setNewName] = useState('')

  const clearStatus = () => {
    setTimeout(() => setStatusMsg(null), 3000)
  }

  const handleAddPOIAtRobot = async () => {
    const name = prompt('Enter POI name:')
    if (!name || !name.trim()) {
      setStatusMsg('✗ POI name required')
      clearStatus()
      return
    }

    // Extract pose from robotPose (handles both PoseStamped and PoseWithCovarianceStamped)
    const pose = robotPose?.pose?.pose || robotPose?.pose
    if (!pose?.position) {
      setStatusMsg('✗ Robot pose unavailable')
      clearStatus()
      return
    }

    // Calculate yaw from quaternion
    const q = pose.orientation
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    setOperating(true)
    setStatusMsg(null)
    try {
      await markPOI({
        name: name.trim(),
        pose: {
          x: pose.position.x,
          y: pose.position.y,
          yaw
        }
      })
      setStatusMsg(`✓ Added POI "${name}"`)
      clearStatus()
    } catch (e: any) {
      setStatusMsg(`✗ Failed to add POI: ${e?.message || 'Unknown error'}`)
      clearStatus()
    } finally {
      setOperating(false)
    }
  }

  const handleDelete = async (poiName: string) => {
    if (!confirm(`Delete POI "${poiName}"?`)) return

    setOperating(true)
    setStatusMsg(null)
    try {
      await deletePOI(poiName)
      setStatusMsg(`✓ Deleted POI "${poiName}"`)
      clearStatus()
    } catch (e: any) {
      setStatusMsg(`✗ Failed to delete: ${e?.message || 'Unknown error'}`)
      clearStatus()
    } finally {
      setOperating(false)
    }
  }

  const handleRename = async (oldName: string) => {
    if (!newName.trim()) {
      setStatusMsg('✗ New name required')
      clearStatus()
      setEditingPoi(null)
      return
    }

    setOperating(true)
    setStatusMsg(null)
    try {
      await renamePOI(oldName, newName.trim())
      setStatusMsg(`✓ Renamed "${oldName}" to "${newName}"`)
      clearStatus()
      setEditingPoi(null)
      setNewName('')
    } catch (e: any) {
      setStatusMsg(`✗ Failed to rename: ${e?.message || 'Unknown error'}`)
      clearStatus()
    } finally {
      setOperating(false)
    }
  }

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200 p-4">
      {/* Header */}
      <div className="flex items-center justify-between mb-3 pb-3 border-b border-slate-200">
        <h2 className="text-lg font-semibold text-slate-800">📍 Points of Interest</h2>
        <button
          onClick={handleAddPOIAtRobot}
          disabled={operating || !robotPose}
          className="px-3 py-1 bg-blue-600 text-white text-sm rounded hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
        >
          + Add at Robot
        </button>
      </div>

      {/* Status Message */}
      {statusMsg && (
        <div className={`mb-3 p-2 rounded text-sm ${
          statusMsg.startsWith('✓') ? 'bg-green-50 text-green-800' : 'bg-red-50 text-red-800'
        }`}>
          {statusMsg}
        </div>
      )}

      {/* POI List */}
      {pois.length === 0 ? (
        <div className="text-center py-6 text-slate-500 text-sm">
          No POIs defined for this map
        </div>
      ) : (
        <div className="space-y-2">
          {pois.map((poi, idx) => {
            const poiName = poi.name || `POI ${idx + 1}`
            const x = poi.pose?.x ?? poi.x ?? 0
            const y = poi.pose?.y ?? poi.y ?? 0
            const yaw = poi.pose?.yaw ?? poi.yaw ?? 0
            const isEditing = editingPoi === poiName

            return (
              <div key={idx} className="flex items-center justify-between p-3 bg-slate-50 rounded border border-slate-200">
                <div className="flex-1">
                  {isEditing ? (
                    <div className="flex items-center gap-2">
                      <input
                        type="text"
                        value={newName}
                        onChange={(e) => setNewName(e.target.value)}
                        placeholder={poiName}
                        className="flex-1 px-2 py-1 border border-slate-300 rounded text-sm"
                        autoFocus
                      />
                      <button
                        onClick={() => handleRename(poiName)}
                        disabled={operating}
                        className="px-2 py-1 bg-green-600 text-white text-xs rounded hover:bg-green-700"
                      >
                        Save
                      </button>
                      <button
                        onClick={() => {
                          setEditingPoi(null)
                          setNewName('')
                        }}
                        className="px-2 py-1 bg-slate-400 text-white text-xs rounded hover:bg-slate-500"
                      >
                        Cancel
                      </button>
                    </div>
                  ) : (
                    <>
                      <div className="font-semibold text-slate-800">{poiName}</div>
                      <div className="text-xs text-slate-600 mt-1">
                        x: {x.toFixed(2)}m, y: {y.toFixed(2)}m, θ: {(yaw * 180 / Math.PI).toFixed(1)}°
                      </div>
                    </>
                  )}
                </div>
                {!isEditing && (
                  <div className="flex gap-2 ml-3">
                    <button
                      onClick={() => {
                        setEditingPoi(poiName)
                        setNewName(poiName)
                      }}
                      disabled={operating}
                      className="px-2 py-1 bg-blue-100 text-blue-700 text-xs rounded hover:bg-blue-200 disabled:opacity-50"
                    >
                      Rename
                    </button>
                    <button
                      onClick={() => handleDelete(poiName)}
                      disabled={operating}
                      className="px-2 py-1 bg-red-100 text-red-700 text-xs rounded hover:bg-red-200 disabled:opacity-50"
                    >
                      Delete
                    </button>
                  </div>
                )}
              </div>
            )
          })}
        </div>
      )}

      {/* Instructions */}
      <div className="mt-4 pt-4 border-t border-slate-200">
        <div className="text-xs text-slate-600 space-y-1">
          <p>• Click <strong>"+ Add at Robot"</strong> to save current robot position as POI</p>
          <p>• POIs are saved per map in <code>{'{map_name}'}.pois.json</code></p>
          <p>• Use POIs for navigation waypoints or landmarks</p>
        </div>
      </div>
    </section>
  )
}
