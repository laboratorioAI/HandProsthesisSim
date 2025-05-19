using BufferPrint;
using Inventor;

namespace ProthesisSimulation
{
    internal class Program
    {
        static BufferPrint.BufferManager bufferManager = new();
        static BufferPrint.InventorObjectData inventorObject;
        private static bool isRunning = false;

        static void Main(string[] args)
        {
            Console.WriteLine("---Welcome to shared memory manager---");
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
                ReedBufferLoop();
            }
        }

        private static void ReedBufferLoop()
        {
            inventorObject = new BufferPrint.InventorObjectData(bufferManager.GetAssemblyDocument());

            int c = 0;
            while (isRunning)
            {
                BufferManagerMessage msg = ReadFromBuffer(c);
                c++;

                msg = RequestInventorCommandData(inventorObject, msg);

                ReadInventorAssemblyData(inventorObject);

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
