import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "PricePeek",
  description: "Price comparison app",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="bg-off_white-100">
      <body>
        {children}
      </body>
    </html>
  );
}
