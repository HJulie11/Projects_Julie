"use client"
import '../app/login/login.css'; 
import React, { useState } from 'react';

interface LoginFormProps {
  username: string;
  password: string;
  handleUsernameChange: (event: React.ChangeEvent<HTMLInputElement>) => void;
  handlePasswordChange: (event: React.ChangeEvent<HTMLInputElement>) => void;

}

const LoginForm: React.FC<LoginFormProps> = ({ username, password, handleUsernameChange, handlePasswordChange}) => {

    return (
      <div className="flex flex-col gap-10 w-400">
        <input
          type="text"
          placeholder="Username"
          value={username}
          onChange={handleUsernameChange}
          className="input-field"
        />
        <input
          type="password"
          placeholder="Password"
          value={password}
          onChange={handlePasswordChange}
          className="input-field"
        />
      </div>
    );
}

export default LoginForm