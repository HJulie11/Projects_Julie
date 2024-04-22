// types/AuthContextType.ts
export interface AuthContextType {
    isLoggedIn: boolean;
    setIsLoggedIn: (isLoggedIn: boolean) => void;
  }