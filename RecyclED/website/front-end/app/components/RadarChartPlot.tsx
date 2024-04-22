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
        { hour: "12:00 AM", "number of users": 3 },
        { hour: "01:00 AM", "number of users": 0 },
        { hour: "02:00 AM", "number of users": 1 },
        { hour: "03:00 AM", "number of users": 0 },
        { hour: "04:00 AM", "number of users": 5 },
        { hour: "05:00 AM", "number of users": 10 },
        { hour: "06:00 AM", "number of users": 40 },
        { hour: "07:00 AM", "number of users": 100 },
        { hour: "08:00 AM", "number of users": 500 },
        { hour: "09:00 AM", "number of users": 540},
        { hour: "10:00 AM", "number of users": 600 },
        { hour: "11:00 AM", "number of users": 1050 },
        { hour: "12:00 PM", "number of users": 1000 },
        { hour: "01:00 PM", "number of users": 1100 },
        { hour: "02:00 PM", "number of users": 1200 },
        { hour: "03:00 PM", "number of users": 1100 },
        { hour: "04:00 PM", "number of users": 1000 },
        { hour: "05:00 PM", "number of users": 800 },
        { hour: "06:00 PM", "number of users": 300 },
        { hour: "07:00 PM", "number of users": 100 },
        { hour: "08:00 PM", "number of users": 100 },
        { hour: "09:00 PM", "number of users": 20 },
        { hour: "10:00 PM", "number of users": 10 },
        { hour: "11:00 PM", "number of users": 7 }
      ];
      

  const formatHour = (hour: string) => {
    if (hour === "12:00 AM" || hour === "06:00 AM" || hour === "12:00 PM" || hour === "06:00 PM") {
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
