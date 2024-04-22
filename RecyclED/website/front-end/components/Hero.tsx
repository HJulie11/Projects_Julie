'use client'
import Image from 'next/image'
import Button from './Button'
import ProductImage from '@/public/product_last draft.png'
import { TypeAnimation } from 'react-type-animation';
import Link from 'next/link';

const Hero = () => {
  return (
    <section className='max-container padding-container flex flex-col gap-20 py-10 pb-32 md:gap-28 lg:py-20 xl:flex-row'>
      <div className='hero-map' />
      {/* LEFT */}

      <div className='relative z-20 flex flex-1 flex-col xl:w-1/2'>

        <div className='my-11 flex flex-wrap gap-5'>
            {Array(1).fill(1).map((_, index) => (
              <div key={index} className='w-full'>
                <h1 className='bold-52 lg:bold-88'>
                  <TypeAnimation sequence={[
                    "The bin that sorts your rubbish for you. "
                  ]}
                  wrapper="span"
                  speed={50}
                  />

                </h1>
                <span>
                  <p className='regular-16 mt-6 text-gray-30 xl:max-w-[520px]'>
                      A project by Lucky 13 🍀
                  </p>
                </span>
              </div>
              
            ))}
        </div>

        <div className="flex flex-col w-full gap-3 sm:flex-row">
          <Link href="/how-it-works">
            <Button 
              type="button" 
              title="See more details" 
              variant='btn_green'
            />
          </Link>
        </div>
      </div>

      <div className='relative w-full flex flex-1 mt-10 items-center'>
            <Image src={ProductImage} alt="product image" width={900}/>
        </div>

      
    </section>
  )
}

export default Hero