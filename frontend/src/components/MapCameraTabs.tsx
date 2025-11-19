import { useState } from 'react'
import { MapView } from './MapView'
import VideoFeed from './VideoFeed'

export default function MapCameraTabs() {
  const [tab, setTab] = useState<'map' | 'camera'>('map')

  return (
    <section className="bg-white rounded-lg shadow border border-slate-200">
      <div className="flex items-center justify-between px-4 pt-3">
        <h2 className="text-lg font-semibold text-slate-800">View</h2>
        <div className="inline-flex rounded-md border border-slate-300 overflow-hidden">
          <button
            onClick={() => setTab('map')}
            className={`px-3 py-1.5 text-sm font-medium ${tab==='map' ? 'bg-slate-900 text-white' : 'bg-white text-slate-700 hover:bg-slate-50'}`}
            aria-pressed={tab==='map'}
          >
            Map
          </button>
          <button
            onClick={() => setTab('camera')}
            className={`px-3 py-1.5 text-sm font-medium border-l border-slate-300 ${tab==='camera' ? 'bg-slate-900 text-white' : 'bg-white text-slate-700 hover:bg-slate-50'}`}
            aria-pressed={tab==='camera'}
          >
            Camera
          </button>
        </div>
      </div>
      <div className="p-4">
        {tab === 'map' ? (
          <MapView embedded />
        ) : (
          <VideoFeed embedded />
        )}
      </div>
    </section>
  )
}
