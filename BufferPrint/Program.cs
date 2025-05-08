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
            int c = 0;

            while (isRunning)
            {
                ReadFromBuffer(c);
                c++;

                ReadInventorObjData();

                WriteIntentorObjData();

                Thread.Sleep(1500);


            }
        }

        private static void ReadFromBuffer(int c)
        {
            BufferManagerMessage dequeueResult = bufferManager.ReadFromBuffer();
            Console.WriteLine($"Count {c} " + dequeueResult.ToString());
        }

        private static void ReadInventorObjData()
        {
            InventorObjectData objData = new InventorObjectData(bufferManager.GetAssemblyDocument());
            objData.PrintToConsoleAssemblyDocumentData();
        }

        private static void WriteIntentorObjData()
        {
            InventorObjectData objData = new InventorObjectData(bufferManager.GetAssemblyDocument());
            objData.PushData();
        }
    }
}
