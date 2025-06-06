using Microsoft.SqlServer.Server;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Emit;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace BufferPrint
{
    internal class Program
    {
        static BufferManager bufferManager = new BufferManager();

        private static bool isRunning = false;
        static void Main(string[] args)
        {
            bufferManager.CreateOrOpenSharedMemory();

            if (!bufferManager.Init())
            {
                Console.WriteLine("Press any key to exit...");
                Console.ReadKey();
            }
            else
            {
                isRunning = true;
                Console.WriteLine("Reading Buffer...");
                RunBufferLoop();
            }
        }

        private static void RunBufferLoop()
        {
            InventorObjectData objData = new InventorObjectData(bufferManager.GetInventorAppInstance(), bufferManager.GetAssemblyDocument());

            int c = 0;
            while (isRunning)
            {
                BufferManagerMessage msg = ReadFromBuffer(c);
                c++;

                msg = RequestInventorCommandData(objData, msg);

                ReadInventorAssemblyData(objData);

                Thread.Sleep(1500);


            }
        }

        private static BufferManagerMessage ReadFromBuffer(int c)
        {
            BufferManagerMessage dequeueResult = bufferManager.ReadFromBuffer();
            Console.WriteLine($"Nr. {c} buffer read: " + dequeueResult.ToString());
            return dequeueResult;
        }

        private static void ReadInventorAssemblyData(InventorObjectData objData)
        {
            objData.ExtractAssemblyData();
        }

        private static BufferManagerMessage RequestInventorCommandData(InventorObjectData objData, BufferManagerMessage msg)
        {
            return objData.CommandData(msg);
        }
    }
}
