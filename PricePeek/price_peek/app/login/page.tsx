// @jsxImportSource next/client
"use client";
import LoginButton from '@/components/LoginButton'
import SignupButton from '@/components/SignupButton'
import LoginForm from '@/components/LoginForm'
import React, { useState } from 'react'
import Toastie2 from '@/public/icon2.png'
import Logo from '@/public/logo_xl.png'
import Image from 'next/image'


const LoginPage = () => {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');

  const handleUsernameChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setUsername(event.target.value);
  };

  const handlePasswordChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setPassword(event.target.value);
  };

  const getNavigationPath = () => {
    if (username.length >= 4 && password.length >= 4) {
      return('/home');
    }
    return ('/login');
  }


  return (
    <div className="flex flex-col items-center justify-center h-screen bg-off_white-100">
        <div className="flex items-center -mb-4 z-0">
            <Image src={Logo} alt="PricePeek Logo" className="w-80 h-auto" />
            <Image src={Toastie2} alt="PricePeek Logo Toast" className="w-24 h-24 mb-auto" />
        </div>
        
        <div className="bg-red-100 rounded-custom p-8 w-full max-w-md">
            <section>
                <h1 className="text-off_white-100 text-center text-5xl font-bold mb-10">Log In</h1>
            </section>
            <LoginForm
              username={username}
              password={password}
              handleUsernameChange={handleUsernameChange}
              handlePasswordChange={handlePasswordChange}
              />
            <div className="flex justify-between mt-10">
                <SignupButton />
                <LoginButton getNavigationPath={getNavigationPath} />
            </div>
                
            <div className="text-center mt-4">
                <button className="text-off_white-100 underline hover:text-grey-100">Forgot Password</button>
            </div>
        </div>
    </div>
  )
}

export default LoginPage