import React from 'react';
import Link from 'next/link'

interface Bin {
  id: number;
  name: string;
  status: string;
}

interface BinsProps {
  bins: Bin[];
}

const getStatusColor = (status: string): string => {
  switch (status) {
    case 'Available':
      return 'text-green-600';
    case 'Full':
      return 'text-yellow-600';
    case 'Maintenance Needed':
      return 'text-red-600';
    default:
      return 'text-gray-600'; 
  }
};

const BinTable: React.FC<BinsProps> = ({ bins }) => {
  return (
    <div className="flex flex-col">
      <div className="overflow-x-auto sm:-mx-6 lg:-mx-8">
        <div className="inline-block min-w-full py-2 sm:px-6 lg:px-8">
          <div className="overflow-hidden">
            <table className="min-w-full text-left text-sm font-light">
              <thead className="border-b bg-white font-medium">
                <tr>
                  <th scope="col" className="px-6 py-4">Bin Name</th>
                  <th scope="col" className="px-6 py-4">ID</th>
                  <th scope="col" className="px-6 py-4">Status</th>
                </tr>
              </thead>
              <tbody>
                {bins.map((bin, index) => (
                  <tr key={bin.id} className={`${index % 2 === 0 ? 'bg-neutral-100' : 'bg-white'} border-b`}>
                    <td className="whitespace-nowrap px-6 py-4 font-medium">
                      <Link href={`/home/${bin.id}`}>
                        {bin.name}
                      </Link>
                    </td>
                    <td className="whitespace-nowrap px-6 py-4">{bin.id}</td>
                    <td className={`whitespace-nowrap px-6 py-4 ${getStatusColor(bin.status)}`}>{bin.status}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </div>
    </div>
  );
}

export default BinTable;
