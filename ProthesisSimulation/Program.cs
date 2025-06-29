using BufferPrint;
using Inventor;
using System.Collections.Generic;

namespace ProthesisSimulation
{
    internal class Program
    {
        //static BufferPrint.InventorObjectData inventorObject;
        static BufferPrint.BufferManager bufferManager = new();
        private static bool isRunning = false;

        static void Main(string[] args)
        {
            Console.WriteLine("---Welcome to shared memory manager---");
            bufferManager.CreateOrOpenSharedMemory();

            bufferManager.SetInventorObjectData(new BufferPrint.InventorObjectData(bufferManager.GetInventorAppInstance(), bufferManager.GetAssemblyDocument()));


            if (!bufferManager.Init())
            {
                Console.WriteLine("Press any key to exit...");
                Console.ReadKey();
            }
            else
            {
                isRunning = true;

                ReadInventorAssemblyData();

                Console.WriteLine("Reading Buffer...");
                ReedBufferLoop();
            }
        }

        private static void ReedBufferLoop()
        {
            int c = 0;
            while (isRunning)
            {
                BufferManagerMessage msg = ReadFromBuffer(c);
                c++;

                msg = RequestInventorCommandData(bufferManager.GetInventorObjectData(), msg);

                Thread.Sleep(700);
            }
        }

        private static BufferManagerMessage ReadFromBuffer(int c)
        {
            BufferManagerMessage dequeueResult = bufferManager.ReadFromBuffer();
            Console.WriteLine($"Nr. {c} buffer read: " + dequeueResult.ToString());
            return dequeueResult;
        }

        private static void ReadInventorAssemblyData()
        {
            //objData.ExtractAssemblyData();
            MassCenterAccumulator myData = bufferManager.GetSubassemblyAccumulator();

            IEnumerable<string> labelsData = myData.Labels;
            foreach (string label in labelsData)
            {
                Console.WriteLine($"Mass center accumulator data - {label}:");
                Console.WriteLine($"Total Mass: {myData.GetTotalMass(label)}");
                Point COM = myData.GetCenterOfMass(label, bufferManager.GetInventorAppInstance());
                Console.WriteLine($"Total Center of Mass: x:{COM.X}, y:{COM.Y}, z:{COM.Z}");
            }
        }

        private static BufferManagerMessage RequestInventorCommandData(InventorObjectData objData, BufferManagerMessage msg)
        {
            return objData.CommandData(msg);
        }
    }
}
