import React from 'react'
import Link from 'next/link';

interface LoginButtonProps {
  getNavigationPath: () => string;
}

const LoginButton: React.FC<LoginButtonProps> = ({ getNavigationPath }) => {
  return (
    <Link href={getNavigationPath()}>
      <button className="bg-yellow-100 text-red-100 border-4 border-yellow-100 px-16 py-3 rounded-custom font-bold hover:bg-red-100 hover:text-yellow-100 transition duration-300">
        Log In
      </button>
    </Link>
  )
}

export default LoginButton