'use client'
import AreaChartPlot from "./AreaChartPlot";
import PieChartPlot from "./PieChartPlot";
import RadarChartPlot from "./RadarChartPlot";
import React, { useEffect, useState } from 'react';
import { BACKEND_SERVER_URL } from '../same';

interface RecycledData {
    proportion: number;
    recycled: number;
    total: number;
  }

  const Charts = () => {
    const [data, setData] = useState<RecycledData[]>([]);

    useEffect(() => {
        const fetchData = async () => {
            try {
                const response = await fetch(BACKEND_SERVER_URL + '/api/recycled_items');
                const json: RecycledData[] = await response.json();
                setData(json);
                console.log("Fetched data:", json);
            } catch (error) {
                console.error("Failed to fetch data:", error);
            }
        };
        fetchData();
    }, []);

    if (data.length === 0) {
        return <p>Loading...</p>;
    }

    const proportionValue = data[0].proportion;
    const recycledValue = data[0].recycled;
    const totalValue = data[0].total;

    return (
        <>
            <section>
                <div className="flex flex-wrap justify-center gap-2 md:gap-0 md:flex-nowrap md:justify-between m-4">
                    <div className="flex-1 w-full md:w-auto px-2 justify-center rounded h-300px">
                        <div className="">
                            <p className="text-transparent bg-clip-text bg-gradient-to-t from-teal-1 to-green-400 py-4 text-3xl font-extrabold leading-none tracking-tight md:text-4xl lg:text-5xl inline-block">{recycledValue}</p>
                            <p className="text-black font-bold">items recycled (all time)</p>
                        </div>
                    </div>
                    <div className="flex-1 w-full md:w-auto px-2 justify-center rounded max-h-300px">
                        <div className="">
                            <p className="text-transparent bg-clip-text bg-gradient-to-t from-teal-1 to-green-400 py-4 text-3xl font-extrabold leading-none tracking-tight md:text-4xl lg:text-5xl inline-block">{Math.round(proportionValue * 100)}%</p>
                            <p className="text-black font-bold">of all waste recycled</p>
                        </div>
                    </div>
                    <div className="flex-1 w-full md:w-auto px-2 justify-centerrounded max-h-300px">
                        <div className="">
                            <p className="text-transparent bg-clip-text bg-gradient-to-t from-teal-1 to-green-400 py-4 text-3xl font-extrabold leading-none tracking-tight md:text-4xl lg:text-5xl inline-block">{totalValue}</p>
                            <p className="text-black font-bold">items recycled this week</p>
                        </div>
                    </div>
                    <div className="flex-1 w-full md:w-auto px-2 justify-center rounded h-300px">
                        <div className="">
                            <p className="text-transparent bg-clip-text bg-gradient-to-t from-teal-1 to-green-400 py-4 text-3xl font-extrabold leading-none tracking-tight md:text-4xl lg:text-5xl inline-block">1pm</p>
                            <p className="text-black font-bold">most active time of day</p>
                        </div>
                    </div>
                </div>
            </section>
            <br></br>

            <section className="flex flex-col md:flex-row my-4 px-4 gap-2">
                <div className="flex md:w-1/3"><PieChartPlot/></div>
                <div className="flex md:w-1/3"><AreaChartPlot/></div>
                <div className="flex md:w-1/3"><RadarChartPlot/></div>
            </section>
        </>
    );
};

export default Charts;