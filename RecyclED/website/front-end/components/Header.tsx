// import Link from 'next/link'
// import Image from 'next/image'
// import DropdownButton from './DropdownButton'

// export interface MenuItem {
//   title: string;
//   route?: string;
//   children?: MenuItem[];
// }
// const menuItems: MenuItem[] = [
//   {
//     title: "Username",
//     children: [
//       {
//         title: "My Account",
//         route: "/",
//       },
//       {
//         title: "My Bins",
//         route: "/",
//       },
//       {
//         title: "Log Out",
//         route: "/app/page.tsx",
//       },
//     ]
//   }
// ];

// export default function Header() {
//   return (
//     <header className='flex gap-10 items-center bg-zinc-800 py-4 px-2'>
//       <Link href="/" target="_blank">
//         <Image src="/Recycled_logo_BL.svg" width={200} height={20} alt="logo" />
//       </Link>

//       <div className='flex gap-8 items-center text-white'>
//         {menuItems.map((item) => {
//           return item.hasOwnProperty("children") ? (
//             <DropdownButton item={item} />
//           ) : (
//             <Link className='hover:text-blue-500' href={item?.route || ""}>
//               {item.title}
//             </Link>
//           );
//         })}
//       </div>
//     </header>
//   )
// }