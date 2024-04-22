import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { AuthContextType } from '../types/AuthContextType';

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider = ({ children }: AuthProviderProps) => {
  const [isLoggedIn, setIsLoggedIn] = useState<boolean>(false);

  useEffect(() => {
    const isUserLoggedIn = document.cookie.split('; ').some((item) => item.trim().startsWith('isLoggedIn='));
    setIsLoggedIn(isUserLoggedIn);
  }, []);

  return (
    <AuthContext.Provider value={{ isLoggedIn, setIsLoggedIn }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};