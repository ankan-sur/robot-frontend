import { useMemo } from 'react'

interface Props {
  title: string
  src?: string
  description: string
  heightClass?: string
}

export function RwtPanel({ title, src, description, heightClass = 'h-96' }: Props) {
  const sanitizedSrc = useMemo(() => src?.trim() || '', [src])

  return (
    <section className="rounded-lg border-2 border-blue-400 bg-gradient-to-br from-white to-blue-50 p-4 shadow-lg">
      <h2 className="text-xl font-semibold mb-4 bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">{title}</h2>
      {sanitizedSrc ? (
        <div className={`${heightClass} bg-slate-900/60 rounded-lg border-2 border-blue-300 overflow-hidden`}>
          <iframe
            title={title}
            src={sanitizedSrc}
            className="w-full h-full border-0"
            sandbox="allow-scripts allow-same-origin allow-forms"
          />
        </div>
      ) : (
        <div className={`${heightClass} bg-slate-100 rounded-lg border-2 border-dashed border-blue-300 flex items-center justify-center text-center text-blue-700 px-4`}>
          <div>
            <p className="font-semibold mb-2">No endpoint configured</p>
            <p className="text-sm">{description}</p>
          </div>
        </div>
      )}
    </section>
  )
}
