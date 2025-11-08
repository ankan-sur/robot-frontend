import React, { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  // Try MJPEG first (works well in <img>), fall back to the HTML stream_viewer in an iframe
  const mjpegUrl = `${ROS_CONFIG.videoBase}/stream?topic=${encodeURIComponent(ROS_CONFIG.topics.camera)}&type=mjpeg`;
  const viewerUrl = `${ROS_CONFIG.videoBase}/stream_viewer?topic=${encodeURIComponent(ROS_CONFIG.topics.camera)}`;
  const [useViewer, setUseViewer] = useState(false)
  const [error, setError] = useState<string | null>(null)

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">Live Camera Feed</h2>
      <div className="flex items-center justify-center min-h-64 bg-gradient-to-br from-slate-900 to-blue-900 rounded-lg border-2 border-blue-500 relative shadow-inner w-full">
        {error && (
          <div className="text-blue-200 text-center p-4">
            <div className="text-base mb-2 font-medium">Camera feed unavailable</div>
            <div className="text-sm text-blue-300">{error}</div>
            <div className="text-sm text-blue-300 mt-2">Open the direct viewer: <a className="underline" href={viewerUrl} target="_blank" rel="noreferrer">Open stream_viewer</a></div>
          </div>
        )}

        {!error && !useViewer && (
          <img
            src={mjpegUrl}
            alt="HFH Robot Camera Feed"
            className="max-h-64 w-auto rounded object-contain"
            onError={(e) => {
              console.warn('MJPEG load failed, falling back to viewer', e);
              // fall back to iframe viewer which returns an HTML wrapper
              setUseViewer(true)
            }}
            onLoad={() => setError(null)}
          />
        )}

        {!error && useViewer && (
          <iframe
            title="HFH Robot Camera Viewer"
            src={viewerUrl}
            className="w-full h-64 rounded border-0"
          />
        )}
      </div>
    </section>
  )
}
