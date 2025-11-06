type Props = { title: string }

export function Header({ title }: Props) {
  return (
    <header className="border-b-2 border-blue-300 bg-gradient-to-r from-blue-600 to-blue-700 text-white shadow-lg sticky top-0 z-10">
      <div className="max-w-7xl mx-auto p-4 flex items-center justify-between">
        <h1 className="text-2xl font-bold">{title}</h1>
      </div>
    </header>
  )
}


