import { AreaChart, Area, XAxis, YAxis, Tooltip, ResponsiveContainer } from 'recharts';
import React, { useEffect, useState } from 'react';
import { BACKEND_SERVER_URL } from '../same';

const AreaChartPlot = () => {

  const [data, setData] = useState([]);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch(BACKEND_SERVER_URL + '/api/area_chart_data'); // Adjust the URL as needed
        const json = await response.json();
        setData(json);
        console.log("Fetched data:", json);
      } catch (error) {
        console.error("Failed to fetch data:", error);
      }
    };

    fetchData();
  }, []);
  
  return (
    <>
        <ResponsiveContainer width="100%" height="100%">
            <AreaChart
            width={730}
            height={250}
            data={data}
            margin={{ top: 10, right: 30, left: 0, bottom: 0 }}
            >
            <defs>
                <linearGradient id="colorUv" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="#8884d8" stopOpacity={1} />
                <stop offset="95%" stopColor="#8884d8" stopOpacity={0.1} />
                </linearGradient>
                <linearGradient id="colorPv" x1="0" y1="0" x2="0" y2="1">
                <stop offset="5%" stopColor="#82ca9d" stopOpacity={1} />
                <stop offset="95%" stopColor="#82ca9d" stopOpacity={0.1} />
                </linearGradient>
            </defs>
            <XAxis dataKey="date" />
            <YAxis />
            <Tooltip />
            <Area
                type="monotone"
                dataKey="general waste"
                stroke="#8884d8"
                fillOpacity={1}
                fill="url(#colorUv)"
            />
            <Area
                type="monotone"
                dataKey="recycled"
                stroke="#82ca9d"
                fillOpacity={1}
                fill="url(#colorPv)"
            />
            </AreaChart>
        </ResponsiveContainer>
    </>
  )
};

export default AreaChartPlot;