import AIicon from '@/public/AI.png'
import GraphIcon from '@/public/Graph.png'
import ManageIcon from '@/public/manage.png'
import LandfillIcon from '@/public/landfill.png'

// NAVIGATION
export const NAV_LINKS = [
  { href: '/', key: 'home', label: 'Home' },
  { href: '/how-it-works', key: 'How RECYCLED works', label: 'How It Works' },
  { href: '/about-us', key: 'About Us', label: 'About Us' },
  { href: '/pricing', key: 'pricing ', label: 'Pricing ' },
  { href: '/contact-us', key: 'contact_us', label: 'Contact Us' },
  
];

// CAMP SECTION
export const PEOPLE_URL = [
  '/person-1.png',
  '/person-2.png',
  '/person-3.png',
  '/person-4.png',
];

// FEATURES SECTION
//AI sorting; statistics; Manage all yours bins in one place; reduce landfill tax
export const FEATURES = [
  {
    title: 'AI Sorting',
    icon: AIicon,
    variant: 'green',
    description:
      "Our AI system automatically waste sorts into the correct recycling categories, reducing contamination and human error.",
  },
  {
    title: 'Statistics',
    icon: GraphIcon,
    variant: 'green',
    description:
    "View statistics on how your company is recycling, and how much waste is being diverted from landfill."
  },
  {
    title: 'Manage all your bins in one place',
    icon: ManageIcon,
    variant: 'green',
    description:
    "Monitor the status of all your bins in one place."
  },
  {
    title: 'Reduce landfill tax',
    icon: LandfillIcon,
    variant: 'green',
    description:
    "By recycling more with RecyclED, you can reduce the amount of waste sent to landfill, and therefore reduce your landfill tax."},
];

// FOOTER SECTION
// FOOTER SECTION
export const FOOTER_LINKS = [
  {
    title: 'Learn More',
    links: [
      'About Us',
      'How It Works',
      'Contact Us'
    ],
    urls: [
      '/about-us',
      '/how-it-works',
      '/contact-us'
    ]
  }
];

export const FOOTER_CONTACT_INFO = {
  title: 'Contact Us',
  links: [
    { label: 'Telephone', value: '07123 456789' },
    { label: 'Email', value: 'recycled.sdp@gmail.com' },
  ],
};

export const SOCIALS = {
  title: 'Social',
  links: [
    '/facebook.svg',
    '/instagram.svg',
    '/twitter.svg',
    '/youtube.svg',
    '/wordpress.svg',
  ],
};