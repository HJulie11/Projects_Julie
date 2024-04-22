'use client'
import AreaChartPlot from "./AreaChartPlot";
import PieChartPlot from "./PieChartPlot";
import RadarChartPlot from "./RadarChartPlot";

const IndividualCharts = () => {
  return (
    <>
        <section>
                <div className="flex m-4 gap-2">
                <div className="flex-1 px-2 justify-center w-16 bg-white shadow rounded h-300px">
                    <div className="">
                    <p className="text-black font-bold">Total items recycled</p>
                    <p className="py-4 font-bold">100,000 </p>
                    <p className="text-green-300">34.5% of all waste</p>
                    </div>
                </div>
                <div className="flex-1 px-2 justify-center w-16 bg-white shadow rounded max-h-300px">
                    <div className="">
                    <p className="text-black font-bold">Total sales</p>
                    <p className="py-4 font-bold">$30,000 </p>
                    <p className="text-green-300">+34.5%</p>
                    </div>
                </div>
                <div className="flex-1 px-2 justify-center w-16 bg-white shadow rounded max-h-300px">
                    <div className="">
                    <p className="text-black font-bold">Total subscriptions</p>
                    <p className="py-4 font-bold">$30,000 </p>
                    <p className="text-green-300">+34.5%</p>
                    </div>
                </div>
                <div className="flex-1 px-2 justify-center w-16 bg-white shadow rounded h-300px">
                    <div className="">
                    <p className="text-black font-bold">Total returns</p>
                    <p className="py-4 font-bold ">$30,000 </p>
                    <p className="text-green-300">+34.5%</p>
                    </div>
                </div>
                </div>
            </section>


            <div className="h-[300px] content-center bg-white rounded"><AreaChartPlot/></div>

            <section className="flex my-4 px-4 gap-2">
                <div className=" w-1/2 h-[250px] bg-white rounded"><PieChartPlot/></div>
                <div className=" w-1/2 h-[250px]  bg-white rounded"><RadarChartPlot/></div>
        </section>

    </>
  );
};

export default IndividualCharts;