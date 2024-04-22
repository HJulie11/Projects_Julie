import React from 'react'
import SearchToastie from '@/public/search_icon.png'
import Icon from '@/public/user.png'
import Image from 'next/image'


const Header = () => {
  return (
    <header className="flex items-center justify-between px-6 py-4">
    {/* Logo */}
    <div>
      <h1 className="font-bold text-4xl lg:text-5xl text-red-100">PricePeek</h1>
    </div>

    {/* Search Bar */}
    <div className="flex flex-grow items-center ml-4 px-2">
      <input
        type="text"
        placeholder="Search"
        className="flex-grow px-0 py-2 border-b-2 border-red-100 bg-off_white-100 focus:outline-none font-bold placeholder-cream-100 text-brown-100"
      />
      <Image
          src={SearchToastie}
          alt="Search"
          className="w-12 h-auto cursor-pointer -ml-11"
        />
    </div>

    {/* Profile Icon */}
    <div className="ml-4">
      <Image
        src={Icon}
        alt="Icon"
        className="w-8 h-auto cursor-pointer ml-2 mt-2"
      />
    </div>
  </header>
  )
}

export default Header