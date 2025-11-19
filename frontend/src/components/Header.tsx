type Props = { title: string }

export function Header({ title }: Props) {
  return (
    <header className="bg-slate-800 text-white shadow-lg sticky top-0 z-10 border-b-2 border-slate-700">
      <div className="max-w-7xl mx-auto p-3 flex items-center justify-between">
        <h1 className="text-xl font-bold">{title}</h1>
      </div>
    </header>
  )
}

