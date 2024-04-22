import { AreaChart, Area, XAxis, YAxis, Tooltip, ResponsiveContainer } from 'recharts';
const AreaChartPlot = () => {
  const data = [
    {
      "date": "15/02",
      "general waste": 4000,
      "recycled": 2400
    },
    {
      "date": "16/02",
      "general waste": 3000,
      "recycled": 1398
    },
    {
      "date": "17/02",
      "general waste": 2000,
      "recycled": 9800
    },
    {
      "date": "18/02",
      "general waste": 2780,
      "recycled": 3908
    },
    {
      "date": "19/02",
      "general waste": 1890,
      "recycled": 4800
    },
    {
      "date": "20/02",
      "general waste": 2390,
      "recycled": 3800
    },
    {
      "date": "21/02",
      "general waste": 3490,
      "recycled": 4300
    }
  ]

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