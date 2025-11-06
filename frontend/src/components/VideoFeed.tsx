import React from 'react'
import { ROS_CONFIG } from '../ros/config'

export default function VideoFeed() {
  const streamUrl = `${ROS_CONFIG.videoBase}/stream?topic=${encodeURIComponent(ROS_CONFIG.topics.camera)}&type=webp`;
  
  return (
    <section className="rounded-lg border border-slate-200 bg-white p-4">
      <h2 className="text-lg font-medium mb-3 text-slate-800">Live Camera Feed</h2>
      <div className="flex items-center justify-center min-h-64 bg-slate-900 rounded">
        <img
          src={streamUrl}
          alt="HFH Robot Camera Feed"
          className="max-h-64 w-auto rounded object-contain bg-slate-900"
          onError={(e) => {
            console.error('Video stream error');
            (e.currentTarget as HTMLImageElement).src = '';
          }}
        />
      </div>
    </section>
  )
}
