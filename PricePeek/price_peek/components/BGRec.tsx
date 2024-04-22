import React from 'react'
import rectangle from '@/public/Rectangle_bg.png'
import Image from 'next/image'

const BGRec = () => {
  return (
    <div className="mt-[-157px] ml-[-204px]">
      <Image src={rectangle} alt='bg-image' />
    </div>
  )
}

export default BGRec