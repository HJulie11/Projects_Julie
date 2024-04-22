import React, { useEffect, useState } from "react";
import { NAV_LINKS } from "@/constants";
import Image from "next/image";
import Link from "next/link";
import Button from "@/components/Button";
import Logo from "../public/logo2.svg";
import MenuBars from "../public/menu.svg";
import DropdownButton from "./DropdownButton";

const Navbar = ({ isLoggedIn }) => {
  const [isNavVisible, setIsNavVisible] = useState(false);

  const toggleNav = () => {
    setIsNavVisible(!isNavVisible);
  };

  return (
    <>
      {isNavVisible && (
        <div
          className="fixed w-full h-full right-0 bg-black opacity-50 z-30"
          onClick={toggleNav}
        ></div>
      )}
      <nav className="flexBetween max-container padding-container relative z-30">
        <Link href="/" className="mt-3">
          <Image src={Logo} width={200} height={32} alt="logo" />
        </Link>

        <div
          className={`${
            isNavVisible ? "" : "hidden"
          } fixed right-0 top-0 h-screen w-1/3 bg-white rounded shadow-lg z-30`}
        >
          <ul className="h-full gap-4 lg:flex">
            {NAV_LINKS.map((link) => (
              <a
                key={link.label}
                className="regular-16 text-gray-50 mt-28 flexCenter cursor-pointer pb-1.5 transition-all hover:font-bold"
                href={link.href}
              >
                {link.label}
              </a>
            ))}
          </ul>
        </div>

        <div className={`sm:${!isNavVisible ? "" : "hidden"} mt-3`}>
          <ul className="hidden h-full gap-12 lg:flex">
            {NAV_LINKS.map((link) => (
              <a
                key={link.label}
                className="regular-16 text-gray-50 mt-3 flexCenter cursor-pointer pb-1.5 transition-all hover:font-bold"
                href={link.href}
              >
                {link.label}
              </a>
            ))}

            {isLoggedIn ? (
                    <div className="regular-16 mt-3 flexCenter cursor-pointer pb-1.5 transition-all hover:font-bold">
                      <DropdownButton 
                      buttonText="My Account" 
                      items={[
                        { label: 'Profile', href:'/profile' },
                        { label: 'My Bins', href: '/home' },
                        { label: 'Log out', href: '/logout' }
                      ]}
                    />
                      </div>

                    ) : (
                      <div className="lg:flexCenter hidden mt-3">
                        <Link href="/login">
                          <Button type="button" title="Login" variant="btn_dark_green" />
                        </Link>
                      </div>
                    )}
          </ul>
        </div>

        <Image
          src={MenuBars}
          alt="menu"
          width={32}
          height={32}
          className="inline-block cursor-pointer lg:hidden mt-3"
          onClick={toggleNav}
        />
      </nav>
    </>
  );
};

export default Navbar;
