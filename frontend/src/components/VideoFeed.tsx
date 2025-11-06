import React from 'react'

type Props = {
  streamUrl: string
}

export default function VideoFeed({ streamUrl }: Props) {
  return (
    <section className="rounded-lg border border-slate-200 bg-white p-4">
      <h2 className="text-lg font-medium mb-3 text-slate-800">Live Camera Feed</h2>
      <div className="flex items-center justify-center min-h-64 bg-slate-900 rounded">
        {/* Replace <img> with <video> if using HLS/MP4 */}
        <img
          src={streamUrl}
          alt="HFH Robot Camera Feed"
          className="max-h-64 w-auto rounded object-contain bg-slate-900"
          onError={e => (e.currentTarget.src = '')}
        />
      </div>
    </section>
  )
}
