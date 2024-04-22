"use client"
import Image, { StaticImageData } from "next/image";
import Link from "next/link";
import router, { useRouter } from "next/navigation";
import { MouseEventHandler } from "react";

//product -> price -> store
type BlockProps = {
  type: 'button' | 'submit';
  product_name: string;
  image?: StaticImageData;
  brandname: string;
  full?: boolean;
  price: string;
  link_to: string;
}

const Block = ({ type, product_name, image, brandname, full, price, link_to}: BlockProps) => {
  return (
    <>

      <Link href={link_to} className="flexCenter w-full flex-col">
        <div className="justify-center items-center">
          {/* <div className="z-20 flex flex-col items-start justify-center gap-12 w-[300px] sm:w-[300px] md:w-[600px] lg:w-[1221px] h-[264px]"> */}
          
          <div className="get-block mt-5 mx-10 w-[400px] lg:w-[800px] md:w-[600px] sm:w-[400px]">
            <div className="pt-10 pl-10">
              <h2 className="regular-16" style={{color: "#FFF5E6", fontSize: 36}}>{brandname}</h2>
            </div>
            <div className="flex flex-col pl-10">
              <p className="bold-16" style={{color: "#FFF5E6", fontSize: 46}}>{product_name}</p>
              <div className="">
                <p className="bold-16" style={{color: "#FFF5E6", fontSize: 46}}>{price}</p>
              </div>
            </div>
          </div>

          
        </div>
      </Link>

    </>
    
  )
}

export default Block