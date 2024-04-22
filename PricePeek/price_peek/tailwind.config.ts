/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './pages/**/*.{js,ts,jsx,tsx,mdx}',
    './components/**/*.{js,ts,jsx,tsx,mdx}',
    './app/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        off_white: {
          100: '#FFF5E6',
        },
        yellow: {
          100: '#FFCE5C',
        },
        red: {
          100: '#BC431E',
        },
        brown: {
          100: '#582000',
        },
        grey: {
          100: '#CDCDCD',
        },
        cream: {
          100: '#D0BEA2',
        },
      },
      screens: {
        xs: '400px',
        '3xl': '1680px',
        '4xl': '2200px',
      },
      maxWidth: {
        '10xl': '1512px',
      },
      borderRadius: {
        '5xl': '40px',
        'custom': '30px',
      },
    },
  },
  plugins: [],
};
