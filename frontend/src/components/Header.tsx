type Props = { title: string }

export function Header({ title }: Props) {
  return (
    <header className="border-b-4 border-blue-500 bg-gradient-to-r from-blue-700 via-indigo-700 to-blue-800 text-white shadow-xl sticky top-0 z-10">
      <div className="max-w-7xl mx-auto p-4 flex items-center justify-between">
        <h1 className="text-2xl font-bold drop-shadow-lg">{title}</h1>
      </div>
    </header>
  )
}

