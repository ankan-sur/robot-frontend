import React, { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  const [error, setError] = useState(false)
  const streamUrl = `${ROS_CONFIG.videoBase}/stream?topic=${encodeURIComponent(ROS_CONFIG.topics.camera)}&type=webp`;
  
  return (
    <section className="rounded-lg border-2 border-blue-300 bg-white p-4 shadow-md">
      <h2 className="text-lg font-semibold mb-3 text-blue-900">Live Camera Feed</h2>
      <div className="flex items-center justify-center min-h-64 bg-blue-900 rounded border-2 border-blue-400 relative">
        {error ? (
          <div className="text-blue-200 text-center p-4">
            <div className="text-sm mb-2">Camera feed unavailable</div>
            <div className="text-xs text-blue-300">Check web_video_server and camera topic</div>
          </div>
        ) : (
          <img
            src={streamUrl}
            alt="HFH Robot Camera Feed"
            className="max-h-64 w-auto rounded object-contain"
            onError={() => {
              console.error('Video stream error');
              setError(true);
            }}
            onLoad={() => setError(false)}
          />
        )}
      </div>
    </section>
  )
}
