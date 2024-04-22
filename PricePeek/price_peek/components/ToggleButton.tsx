"use client"
import React, { useState } from 'react';

interface ToggleButtonProps {
  onToggle: () => void;
}

const ToggleButton: React.FC<ToggleButtonProps> = ({ onToggle }) => {
  const [isChecked, setIsChecked] = useState(false);
  
  const handleChange = () => {
    setIsChecked(!isChecked);
    onToggle(); // Call the onToggle function when the toggle button is clicked
  };
  
  return (
    <label htmlFor="switcher" className="flex justify-left cursor-pointer mt-8 ml-6">
      <div className="relative flex justify-between w-[150px] sm:w-[150px] md:w-[250px] lg:w-[300px] h-[32px] bg-red-100 rounded-custom">
        <input id="switcher" type="checkbox" className="hidden peer" checked={isChecked} onChange={handleChange} />
        <span className="text-center flex-grow relative z-20 self-center transition text-red-100 peer-checked:text-off_white-100 font-bold">List</span>
        <span className="text-center flex-grow relative z-20 self-center transition text-off_white-100 peer-checked:text-red-100 font-bold">Map</span>
        <span className={`absolute toggle z-10 bg-off_white-100 h-[28px] w-[49.5%] sm:w-[49.5%] md:w-[49.5%] lg:w-[49.5%] rounded-custom transition-all top-0.5 left-0 ${isChecked ? 'peer-checked:left-[calc(100%-50.5%)]' : ''}`}></span>
      </div>
    </label>
  );
}

export default ToggleButton;