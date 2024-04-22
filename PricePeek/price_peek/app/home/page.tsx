"use client"
import React, { useState } from 'react';
import Header from '@/components/Header'
import Block from '@/components/Block'
import ToggleButton from '@/components/ToggleButton'
import MapView from '@/components/MapView';

const HomePage = () => {
    const [showMap, setShowMap] = useState(false);

    const toggleMap = () => {
      setShowMap(!showMap);
    };
    return (
        <>
          <div className="flex flex-col h-screen bg-off_white-100">
            <Header />
            <ToggleButton onToggle={toggleMap} />
            
            {showMap ? (
                <div className='mt-12 flex items-center mx-8 rounded-custom'>
                    <MapView />
                </div>
            ) : (
              // <div className="flex items-center justify-center ml-0 -mt_5">
              <Block type={'button'} product_name={'Croissant'} brandname={"Sainsbury's"} price={'£3.60'} link_to={'/'} />
            )}
          </div>
        </>
      );
}

export default HomePage