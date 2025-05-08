using Inventor;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace BufferPrint
{
    public class BufferManager
    {
        private Inventor.Application m_InventorApplication;
        private Inventor.AssemblyDocument m_assemblyDocument;
        private ArrayList m_ComponentOccurrenceArray = new ArrayList();
        private ArrayList m_JointInfoArray = new ArrayList();
        private TreeView m_tree;

        private ArrayList buffer = new ArrayList();

        public void AddDataToBuffer(string data)
        {
            buffer.Add(data);
        }

        // Method to retrieve data from the buffer
        public string PopBuffer()
        {
            if (buffer.Count > 0)
            {
                // Get the first element (FIFO)
                object firstElement = buffer[0];

                // Remove the first element from the buffer
                buffer.RemoveAt(0);

                // Return the first element as a string (you may need to adjust this depending on the data type)
                return firstElement.ToString();
            }
            else
            {
                // Return a message if the buffer is empty
                return "Buffer is empty";
            }
        }


        public BufferManager()
        {
        }

        private bool ConnectInventor()
        {
            try
            {
                try
                {
                    // Get active inventor object
                    m_InventorApplication = MarshalCore.GetActiveObject("Inventor.Application") as Inventor.Application;
                }
                catch (COMException)
                {
                    MessageBox.Show("BufferPrint: Inventor must be running.");
                    return false;
                }

                // Make sure that at least one document is opened
                m_assemblyDocument = m_InventorApplication.ActiveDocument as AssemblyDocument;
                if (m_assemblyDocument == null)
                {
                    MessageBox.Show("BufferPrint: An assembly document must be active.");
                    return false;
                }
            }
            catch (Exception e)
            {
                MessageBox.Show(e.Message);
                return false;
            }
            return true;
        }

        public bool Init()
        {
            if (!ConnectInventor())
            {
                return false;
            }

            return true;
        }


    }
}
