import Image from "next/image";
import React from "react"
import Logo from "@/components/Logo"
import BGRec from "@/public/Rectangle_bg.png";
import LPSection1 from "@/components/LP_section1";
import GetApp from "@/components/GetApp"

export default function Home() {
  return (

    <>
      <Image src={BGRec} alt="Bg-rec" className="absolute inset-0 w-full h-auto mr-3 -mt-10 object-cover z-0" style={{ zIndex: '-1' }} />
      <div className="absolute inset-0">
        <Logo />
        <LPSection1/>
        <GetApp />
      </div>
    </>

  );
}
