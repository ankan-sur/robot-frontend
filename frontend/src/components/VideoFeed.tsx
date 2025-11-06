import React, { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  const [error, setError] = useState(false)
  const streamUrl = `${ROS_CONFIG.videoBase}/stream?topic=${encodeURIComponent(ROS_CONFIG.topics.camera)}&type=webp`;
  
  return (
    <section className="rounded-lg border-2 border-violet-400 bg-gradient-to-br from-white to-violet-50 p-4 shadow-lg">
      <h2 className="text-lg font-semibold mb-3 bg-gradient-to-r from-violet-600 to-purple-600 bg-clip-text text-transparent">Live Camera Feed</h2>
      <div className="flex items-center justify-center min-h-64 bg-gradient-to-br from-violet-900 to-purple-900 rounded-lg border-2 border-violet-500 relative shadow-inner">
        {error ? (
          <div className="text-violet-200 text-center p-4">
            <div className="text-sm mb-2 font-medium">Camera feed unavailable</div>
            <div className="text-xs text-violet-300">Check web_video_server and camera topic</div>
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
