// import { BinData } from '@/Data/Bin/BinData';
// import { useState } from 'react';

// async function fetchData(input: RequestInfo, init?: RequestInit) {
//   const response = await fetch(input, init)
//   if(response.ok){
//     return response
//   } else {
//     const errorBody = await response.json();
//     const errorMessage = errorBody.error;
//     throw (errorMessage);
//   }
// }

// export async function fetchBins(): Promise<BinData>{
//   const [binData, setBinData] = useState<any>(null);
//   const [isLoading, setIsLoading] = useState<boolean>(true);
//   const [error, setError] = useState<Error | null>(null);
//   try {
//     const response = await fetchData('http://172.21.4.224:5000/api/read_bin', {method: "GET"}); // Replace with your API endpoint
//     if (!response.ok) {
//       throw new Error('Network response was not ok');
//     }
//     const jsonData = await response.json();
//     setBinData(jsonData);
//   } catch (error) {
//     setError(error);
//   } finally {
//     setIsLoading(false);
//   }
//   return response.json();
// }

// export interface BinInput {
//   id: number;
//   name: string;
//   status: string;
// }

// export async function CreateBin(bin: BinInput): Promise<BinData>{
//   const response = await fetchData("http://172.21.4.224:5000/api/read_bin",
//   {
//     method: "POST",
//     headers: {
//       "Content-Type": "applicaiton/json"
//     },
//     body: JSON.stringify(bin)
//   });
//   return response.json();
// }

// export async function UpdateStatus(): Promise<BinData[]>{
//   const response = await fetchData("http://172.21.4.224:5000/api/update_bin_status", {method: "GET"})
//   return response.json()
// }

// // export interface BinRead {
// //   id: number;

// // }

// // export async function readBin(bin: BinRead):