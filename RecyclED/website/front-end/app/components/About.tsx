import React from 'react';
import Product from '@/public/product_last draft.png'
import Logo from '@/public/logozoomed.png'
import Image from 'next/image';

const About = () => {
  return (
    <div className="container mx-auto py-10 px-4 max-w-6xl">
        <div className="mb-12">
            <h1 className="text-3xl leading-none tracking-tight md:text-4xl lg:text-5xl font-bold text-gray-900 mb-8">
            About Us
            </h1>
        </div>
        <div className="mb-12">
            <h2 className="text-xl md:text-2xl lg:text-3xl font-bold leading-none tracking-tight text-transparent bg-clip-text bg-gradient-to-t from-green-1 to-teal-1 mb-4">
            The Team
            </h2>
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Lauren Cosgrove</span>
                    </h3>
                    <p className="text-gray-700">
                    Lauren worked on the software team, developing the front-end of our website with Julie and carrying out usability tests to improve the website's UX. Additionally, she made the slides for our demo and carried out market research to use in the presentation.</p></div>
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Christopher Dalziel</span>
                    </h3>
                    <p className="text-gray-700">         
                    Chris worked on hardware, doing most of the low-level design for the project. He did any tasks which involved requisitioning parts from the technicians or using Fusion360, as well as assisting with tasks which involved physical construction work.</p>
                    </div>
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Julie Hong</span>
                    </h3>
                    <p className="text-gray-700">
                    Julie mainly worked on developing front-end of the website with Lauren using Next.js and TailwindCSS. Tasks that requires designs such as website front-end, poster, and slides were a major focus for her during the project. </p>
                    </div>
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Sara Khatun Guluzade</span>
                    </h3>
                    <p className="text-gray-700">
                    Sara built an object recognition and classification model using TensorFlow and Google Teachable Machine, focusing on image augmentation and model fine-tuning. She implemented key performance metrics to evaluate the model, as well as developing a new evaluation dataset, and improving model performance. </p>
                    </div>
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Nik Peleshatyi</span>
                    </h3>
                    <p className="text-gray-700">
                    Nik worked primarily on software/hardware integration, which includes controlling hardware, and set up the AI model made by Sara as a recognition service. He rotated between all teams to help the progress: he shared his experience to aid front-end, back-end, and Docker infrastructure; developed a Motion Detection algorithm; built a backup LEGO cart that mimics the 3D-printed one and linked Raspberry Pi with Arduino to work together and control the cart. In addition, he built a data collection portal to store images for the model training dataset.</p>
                </div> 
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Siddharth Sakriya</span>
                    </h3>
                    <p className="text-gray-700">
                    Sid built the backend connecting the website, model, and hardware. Helped containerising the application with Docker. Built a CI pipeline (test automation), and performed extensive QA for the backend and hardware. Involved heavily with programming the Pi, Arduino, and EV3, and helped build the backup Lego cart. In addition, he was heavily involved in presenting at all 3 demos. </p>
                    </div>
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Kunyu Wang</span>
                    </h3>
                    <p className="text-gray-700">
                    Kunyu mainly focused on the hardware aspect of the project. He worked with Chris and Yihan to assemble the arm, and later, assembled and improved the cart and related sliding doors. He worked with Yihan to repair and assemble the new cart modeled by Chris using 3D printing technology.</p>
                </div>
                <div>
                    <h3 className="text-lg font-semibold mb-2">
                    <span className="font-bold">Yihan Wang</span>
                    </h3>
                    <p className="text-gray-700">
                    Yihan worked within the hardware team throughout the entire project, concentrating on the design, mechanism, and implementation of hardware components, particularly the cart and the outer frame. He ensured the reliability and robustness of our hardware system. He also made contributions to the databases for object recognition. </p>
                </div>
            </div>
        </div>
        <div className="mb-12">
            <h2 className="text-xl md:text-2xl lg:text-3xl font-bold leading-none tracking-tight text-transparent bg-clip-text bg-gradient-to-t from-green-1 to-teal-1 mb-4">
            The Project
            </h2>
                <div>
                    <p className="text-gray-700">
                    Have you ever been in a position where you don't know what bin to dispose of your waste in? RecyclED has been created to alleviate the ambiguities around waste disposal and to prevent individuals from putting items in the wrong bin. When non-recyclable items end up in recycling bins, the whole bin becomes contaminated - none of the waste can be recycled anymore.
                    </p>
                    <br></br>
                    <p className="text-gray-700">
                    With 84% of UK households unintentionally contaminating their recycling bins, and only 7.5% of all recyclable waste in offices ending up in recycling facilities (75% of all office waste is recyclable), this situation is imperative to control, especially when the practice is crucial to reducing the volume of waste sent to landfills, conserving natural resources and reduces the need to mine or source new raw materials. It is evident that this is a large issue, and if not mitigated swiftly, likely to contribute to climate change.
                    </p>
                    <br></br>
                    <p className="text-gray-700">
                    Our product, RecyclED, is a smart recycling device that classifies items placed into it and automatically places them in the correct bin with the use of pistons on the left and right to move items over the correct hatch to be disposed of. Bin owners are also provided with an application that notifies them when their bins are close to filling up so that the appropriate staff can be sent to empty the bins, thus minimising the likelihood of bins overflowing.
                    </p>
                    <br></br>
                    <p className="text-gray-700">
                    RecyclED is perfect for large corporations seeking to enhance their waste management and simplify the recycling process.
                    </p>
                </div>
                <div className='flex flex-wrap justify-center'>
                    <div>
                        <h3 className="text-lg font-semibold text-green-1 mt-5">
                            <span className="font-bold">Model</span>
                        </h3>
                        <Image src={Product} alt="project1" width={400}/>
                    </div>
                    <div>
                        <h3 className="text-lg font-semibold text-green-1 mt-5">
                            <span className="font-bold">Logo</span>
                        </h3>
                        <Image src={Logo} alt="project1" height={300}/>
                    </div>
                </div>
        </div>
    </div>
  );
};

export default About;
