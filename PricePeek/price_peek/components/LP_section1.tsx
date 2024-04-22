'use client'
import Image from 'next/image'
import Button from './Button'
import Toastie from '@/public/Toastie_shadow.png'
import Link from 'next/link'

const LPSection1 = () => {
  return (
    <section className='max-container padding-container flex flex-col gap-20 py-10 pb-32 md:gap-28 lg:py-20 xl:flex-row'>
      <div/>
      {/* LEFT */}

      <div className='relative z-20 flex flex-1 flex-col xl:w-1/2'>

        <div className='my-11 flex flex-wrap gap-5'>
            {Array(1).fill(1).map((_, index) => (
              <div key={index} className='w-full'>
                <h1 className='bold-52 lg:bold-88'>
                  Take a peek, get a price
                </h1>
              </div>
              
            ))}
        </div>

        <div className="flex flex-col w-full gap-3 sm:flex-row">
          <Link href="/login" className='w-full'>
            <Button 
              type="button" 
              title="Log in" 
              variant='bg-red-100 text-yellow-100 w-full h-[75px]'
            />
          </Link>
          <Button 
            type="button" 
            title="Sign up" 
            variant='border-red-100 border-4 text-red-100 w-full h-[75px]'
          />
        </div>
      </div>

      <div className='relative w-full flex flex-1 mt-10 items-center'>
        <Image src={Toastie} alt="toast image" width={900}/>
      </div>

      
    </section>
  )
}

export default LPSection1