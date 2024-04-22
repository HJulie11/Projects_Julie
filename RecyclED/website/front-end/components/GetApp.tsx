import React from 'react'
import Button from './Button'
import Image from 'next/image'
import DLW from '@/public/download_wh.png'
import DLG from '@/public/download_gr.png'
import Phone from '@/public/Phone_eve.png'

const GetApp = () => {
  return (
    <section className="flexCenter w-full flex-col pb-[100px]">
      <div className="get-app rounded-[30px]">
        <div className="z-20 flex w-full flex-1 flex-col items-start justify-center gap-12">
          <h2 className="bold-40 lg:bold-64 xl:max-w-[320px]">Get our app for free now!</h2>
          <p className="regular-16 text-gray-10">Available on iOS and Android</p>
          <div className="flex w-full flex-col gap-3 whitespace-nowrap xl:flex-row">
            <Button 
              type="button"
              title="App Store"
              icon={DLG}
              variant="btn_white"
            />
            <Button 
              type="button"
              title="Play Store"
              icon={DLW}
              variant="btn_green"
            />
          </div>
        </div>

        <div className="flex flex-1 items-center justify-end absolute rotate-6 top-10 right-20">
          <Image src={Phone} alt="phones" width={500}/>
        </div>
      </div>
    </section>
  )
}

export default GetApp