import React, { useState } from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  const [error, setError] = useState(false)
  // Use MJPEG for broad compatibility with web_video_server
  const streamUrl = `${ROS_CONFIG.videoBase}/stream?topic=${encodeURIComponent(ROS_CONFIG.topics.camera)}&type=mjpeg`;
  
  return (
    <section className="rounded-lg border-2 border-purple-400 bg-gradient-to-br from-white to-purple-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-purple-600 to-pink-600 bg-clip-text text-transparent">Live Camera Feed</h2>
      <div className="flex items-center justify-center min-h-64 bg-gradient-to-br from-purple-900 to-pink-900 rounded-lg border-2 border-purple-500 relative shadow-inner">
        {error ? (
          <div className="text-purple-200 text-center p-4">
            <div className="text-base mb-2 font-medium">Camera feed unavailable</div>
            <div className="text-sm text-purple-300">Check web_video_server and camera topic</div>
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
