import { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  // Use encodeURI, not encodeURIComponent, so slashes in topic aren't escaped
  const safeTopic = encodeURI(ROS_CONFIG.topics.camera)
  const mjpegUrl = `${ROS_CONFIG.videoBase}/stream?topic=${safeTopic}&type=mjpeg`
  const viewerUrl = `${ROS_CONFIG.videoBase}/stream_viewer?topic=${safeTopic}`
  const [useViewer, setUseViewer] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [dims, setDims] = useState<{ w: number; h: number } | null>(null)
  const [expanded, setExpanded] = useState(false)

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-xl font-semibold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Camera Feed</h2>
        <button
          onClick={() => setExpanded(true)}
          className="px-2 py-1 text-xs rounded bg-blue-100 hover:bg-blue-200 text-blue-700 border border-blue-300"
          title="Maximize"
        >Maximize</button>
      </div>
      {error && (
        <div className="text-sm text-blue-800 bg-blue-100 border border-blue-200 rounded p-3 mb-3">
          {error}. Try the <a className="underline" href={viewerUrl} target="_blank" rel="noreferrer">stream viewer</a>.
        </div>
      )}
      {/* Compact view */}
      <div className="flex items-center justify-center bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 relative shadow-inner p-2">
        {!useViewer && (
          <img
            src={mjpegUrl}
            alt="HFH Robot Camera Feed"
            className="block rounded max-h-40 w-full object-contain"
            onError={() => {
              setUseViewer(true)
              setError('MJPEG stream unavailable, switching to HTML viewer…')
            }}
            onLoad={(e) => {
              const img = e.currentTarget
              setDims({ w: img.naturalWidth, h: img.naturalHeight })
              setError(null)
            }}
          />
        )}
        {useViewer && (
          <iframe
            title="HFH Robot Camera Viewer"
            src={viewerUrl}
            className="rounded border-0 w-full"
            style={{ height: '10rem' }}
          />
        )}
      </div>
      <div className="mt-2 text-xs text-blue-700">
        If the image is blank, try opening the stream directly: {' '}
        <a className="underline" href={mjpegUrl} target="_blank" rel="noreferrer">MJPEG</a>
        {' '}or{' '}
        <a className="underline" href={viewerUrl} target="_blank" rel="noreferrer">Viewer</a>.
      </div>

      {/* Expanded modal */}
      {expanded && (
        <div className="fixed inset-0 z-50 bg-black/70 flex items-center justify-center p-4">
          <div className="bg-white rounded-lg shadow-2xl border-2 border-blue-500 max-w-[95vw] w-full p-3">
            <div className="flex items-center justify-between mb-2">
              <div className="text-base font-semibold text-blue-800">Camera</div>
              <button
                onClick={() => setExpanded(false)}
                className="px-2 py-1 text-xs rounded bg-blue-100 hover:bg-blue-200 text-blue-700 border border-blue-300"
              >Close</button>
            </div>
            <div className="flex items-center justify-center bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 p-2">
              {!useViewer && (
                <img
                  src={mjpegUrl}
                  alt="HFH Robot Camera Feed"
                  className="block rounded max-w-full max-h-[80vh] object-contain"
                  onError={() => {
                    setUseViewer(true)
                    setError('MJPEG stream unavailable, switching to HTML viewer…')
                  }}
                />
              )}
              {useViewer && (
                <iframe
                  title="HFH Robot Camera Viewer"
                  src={viewerUrl}
                  className="rounded border-0 w-full"
                  style={{ height: '80vh' }}
                />
              )}
            </div>
          </div>
        </div>
      )}
    </section>
  )
}
