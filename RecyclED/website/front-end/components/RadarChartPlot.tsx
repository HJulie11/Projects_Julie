import React from "react";
import {
  RadarChart,
  Radar,
  PolarAngleAxis,
  PolarGrid,
  Legend,
  Tooltip,
  ResponsiveContainer
} from "recharts";

const RadarChartPlot = () => {
    const data = [
      { "hour": "12 AM", "number of users": 0 },
      { "hour": "1 AM", "number of users": 0 },
      { "hour": "2 AM", "number of users": 0 },
      { "hour": "3 AM", "number of users": 0 },
      { "hour": "4 AM", "number of users": 0 },
      { "hour": "5 AM", "number of users": 1 },
      { "hour": "6 AM", "number of users": 4 },
      { "hour": "7 AM", "number of users": 10 },
      { "hour": "8 AM", "number of users": 50 },
      { "hour": "9 AM", "number of users": 54 },
      { "hour": "10 AM", "number of users": 60 },
      { "hour": "11 AM", "number of users": 105 },
      { "hour": "12 PM", "number of users": 100 },
      { "hour": "1 PM", "number of users": 110 },
      { "hour": "2 PM", "number of users": 120 },
      { "hour": "3 PM", "number of users": 110 },
      { "hour": "4 PM", "number of users": 100 },
      { "hour": "5 PM", "number of users": 80 },
      { "hour": "6 PM", "number of users": 30 },
      { "hour": "7 PM", "number of users": 10 },
      { "hour": "8 PM", "number of users": 10 },
      { "hour": "9 PM", "number of users": 2 },
      { "hour": "10 PM", "number of users": 1 },
      { "hour": "11 PM", "number of users": 0 }
  ];
      

  const formatHour = (hour: string) => {
    if (hour === "12 AM" || hour === "6 AM" || hour === "12 PM" || hour === "6 PM") {
      return hour;
    } else {
      return "";
    }
  };

  return (
    <ResponsiveContainer width="100%" height={300}>
      <RadarChart outerRadius={90} width={730} height={250} data={data}>
        <PolarGrid />
        <PolarAngleAxis dataKey="hour" tickFormatter={formatHour} />
        <Radar
          name="Users"
          dataKey="number of users"
          stroke="#087E8B"
          fill="#087E8B"
          fillOpacity={0.6}
        />
        <Legend />
        <Tooltip />
      </RadarChart>
    </ResponsiveContainer>
  );
};

export default RadarChartPlot;
