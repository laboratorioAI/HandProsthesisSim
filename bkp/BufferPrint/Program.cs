namespace BufferPrint
{
    internal static class Program
    {
        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            // To customize application configuration such as set high DPI settings or default font,
            // see https://aka.ms/applicationconfiguration.
            ApplicationConfiguration.Initialize();
            if (bufferManager.Init())
            {
                Application.Run(new Form1(bufferManager));
            }
        }

        static BufferManager bufferManager = new BufferManager();
    }
}