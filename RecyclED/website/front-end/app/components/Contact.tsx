import React from 'react';
import Link from 'next/link';

const Contact = () => {
  return (
    <div className="container mx-auto py-10 px-4 max-w-6xl">
      <div className="mb-12">
        <h1 className="text-3xl leading-none tracking-tight md:text-4xl lg:text-5xl font-bold text-gray-900 mb-8">
          Contact Us
        </h1>
      </div>

      <div className="mb-12">
        <p className="text-gray-700 leading-relaxed mb-5">
            If you have any questions or feedback, please feel free to contact us using
            the email address below. We will get back to you as soon as possible.

        </p>
      <Link href="mailto: recycled.sdp@gmail.com" className="text-blue-500 hover:underline">recycled.sdp@gmail.com</Link>
      </div>
    </div>
  );
};

export default Contact;
