import { PieChart, Pie, Cell, Tooltip, ResponsiveContainer } from "recharts";
import React, { useEffect, useState } from 'react';
import { BACKEND_SERVER_URL } from '../same';

const PieChartPlot = () => {
  const [data, setData] = useState([]);
  useEffect(() => {
    const fetchData = async () => {
      try {
        const response = await fetch(BACKEND_SERVER_URL + '/api/pi_chart_data'); 
        const json = await response.json();
        setData(json);
        console.log("Fetched data:", json);
      } catch (error) {
        console.error("Failed to fetch data:", error);
      }
    };
    fetchData();
  }, []);

  const colors = [
    "#82CA9D",
    "#027512",
    "#8884D8",
    "#FF5A5F",
    "#087E8B",


  ];

  return (
    <>
      <ResponsiveContainer width="100%" height="100%">
        <PieChart width={730} height={250}>
          <Pie
            data={data}
            dataKey="value"
            nameKey="name"
            cx="50%"
            cy="50%"
            fill="#8884d8"
            label
          >
            {data.map((entry, index) => (
              <Cell key={`cell-${index}`} fill={colors[index]} />
            ))}
          </Pie>
          <Tooltip />
        </PieChart>
      </ResponsiveContainer>
      <h2 style={{ textAlign: 'center' }}></h2>
    </>
  );
};

export default PieChartPlot;
