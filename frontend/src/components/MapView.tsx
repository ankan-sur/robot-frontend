type Props = {
  position?: { x: number; y: number; z: number }
}

export function MapView({ position }: Props) {
  const x = position?.x ?? 0;
  const y = position?.y ?? 0;

  return (
    <section className="rounded-lg border border-slate-200 bg-white p-4">
      <h2 className="text-lg font-medium mb-3 text-slate-800">Map</h2>
      <div className="h-64 bg-slate-50 rounded flex items-center justify-center text-sm text-slate-500 relative">
        <div className="absolute inset-0 flex items-center justify-center">
          <div className="text-center">
            <div>Map placeholder</div>
            <div className="mt-2 text-xs">Position: ({x.toFixed(2)}, {y.toFixed(2)})</div>
          </div>
        </div>
        {/* Robot position indicator */}
        {position && (
          <div 
            className="absolute w-3 h-3 bg-sky-600 rounded-full border-2 border-white"
            style={{
              left: `${50 + (x * 10)}%`,
              top: `${50 - (y * 10)}%`,
              transform: 'translate(-50%, -50%)'
            }}
            title={`Robot: (${x.toFixed(2)}, ${y.toFixed(2)})`}
          />
        )}
      </div>
    </section>
  )
}



