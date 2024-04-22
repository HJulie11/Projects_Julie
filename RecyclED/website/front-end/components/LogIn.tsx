"use client"
import { useState } from 'react';
import Link from 'next/link';
import { useRouter } from 'next/router';
import { BACKEND_SERVER_URL } from '../same';

const Login = () => {
  
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleEmailChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setEmail(e.target.value);
  };

  const handlePasswordChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setPassword(e.target.value);
  };

  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    // Here you handle the login logic by submitting the email and password to your backend
    console.log('Email:', email);
    console.log('Password:', password);

    const payload = {
      email: email,
      password: password,
    };

    fetch(BACKEND_SERVER_URL + '/api/login_user',
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Necessary for cookies if your authentication uses them
      body: JSON.stringify(payload),
    }
    ).then(resp => resp.json())
    .then(data => {
      // Here you can handle the response data
      // document.cookie = "isLoggedIn=true; path=/; max-age=3600"; // Set a cookie to indicate the user is logged in
      window.location.href = '/home';
      // console.log(data);
    }).catch(e => {
      alert('An error occurred during login.');
      console.log(e);
    })
  };

  return (
    <div className="flex items-center justify-center">
      <div className="bg-white p-[70px] lg:px-[300px] h-full w-full">
        <h2 className="text-2xl font-bold mb-4">Login</h2>
        <form onSubmit={handleSubmit}>
          <div className="mb-4">
            <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="email">
              Email Address
            </label>
            <input
              className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
              id="email"
              type="email"
              placeholder="Enter your email"
              value={email}
              onChange={handleEmailChange}
              required
            />
          </div>
          <div className="mb-6">
            <label className="block text-gray-700 text-sm font-bold mb-2" htmlFor="password">
              Password
            </label>
            <input
              className="shadow appearance-none border rounded w-full py-2 px-3 text-gray-700 leading-tight focus:outline-none focus:shadow-outline"
              id="password"
              type="password"
              placeholder="Enter your password"
              value={password}
              onChange={handlePasswordChange}
              required
            />
          </div>
          <div className="flex items-center justify-between">
              <button
              className="bg-green-500 hover:bg-green-600 text-white font-bold py-2 px-4 rounded focus:outline-none focus:shadow-outline"
              type="submit">
                Log In
              </button>
            
          </div>
        </form>
      </div>
    </div>
  );
};

export default Login;