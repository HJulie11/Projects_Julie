import { PieChart, Pie, Cell, Tooltip, ResponsiveContainer } from "recharts";

const PieChartPlot = () => {
  const data = [
    {
      name: "Plastic",
      value: 200400,
    },
    {
      name: "Glass",
      value: 205000,
    },
    {
      name: "Paper",
      value: 23400,
    },
    {
      name: "Metal",
      value: 20000,
    },
    {
      name: "General Waste",
      value: 29078,
    }
  ];

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
