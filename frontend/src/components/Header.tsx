type Props = { title: string }

export function Header({ title }: Props) {
  return (
    <header className="border-b border-slate-200 bg-white/80 backdrop-blur sticky top-0 z-10">
      <div className="max-w-7xl mx-auto p-4 flex items-center justify-between">
        <h1 className="text-xl font-semibold text-slate-900">{title}</h1>
      </div>
    </header>
  )
}


