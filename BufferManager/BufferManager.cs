using Inventor;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO.MemoryMappedFiles;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace BufferPrint
{
    public class BufferManager
    {
        private Inventor.Application m_InventorApplication;
        private Inventor.AssemblyDocument m_assemblyDocument;
        private ArrayList m_ComponentOccurrenceArray = new ArrayList();
        private ArrayList m_JointInfoArray = new ArrayList();
        private Tree<string> m_tree;

        private MemoryMappedFile _mmf;
        private string _sharedMemoryName = "InventorValuesInput";
        private int _size = 4096;
        private int _maxFloats = 100;
        private Mutex _mutex;
        private int timeout = 750;

        private const int HeadOffset = 0;
        private const int TailOffset = 4;
        private const int BufferStartOffset = 8;

        public AssemblyDocument GetAssemblyDocument()
        {
            return m_assemblyDocument;
        }

        public void SetAssemblyDocument(AssemblyDocument assemblyDocument)
        {
            m_assemblyDocument = assemblyDocument;
        }

        public BufferManager()
        {
            _mutex = new Mutex(false, $"Global\\{_sharedMemoryName}_Mutex");
        }

        public void CreateOrOpenSharedMemory()
        {
            _mmf = MemoryMappedFile.CreateOrOpen(_sharedMemoryName, _size);
        }

        // Enqueue a float (write to the buffer)
        public BufferManagerMessage WriteToBuffer(float data)
        {
            BufferManagerMessage myMsg = new BufferManagerMessage();

            if (_mmf == null)
            {
                myMsg.Status = false;
                myMsg.Message = "Shared memory not initialized.";
                return myMsg;
            }

            if (!_mutex.WaitOne(TimeSpan.FromMilliseconds(timeout)))
            {
                myMsg.Status = false;
                myMsg.Message = $"Failed to acquire mutex, timeout {timeout}.";
                return myMsg;
            }

            try
            {
                using (MemoryMappedViewAccessor accessor = _mmf.CreateViewAccessor())
                {
                    // Read head and tail
                    int head = accessor.ReadInt32(HeadOffset);
                    int tail = accessor.ReadInt32(TailOffset);

                    // Check if buffer is full (tail + 1 == head, modulo buffer size)
                    if((tail + 1) % _maxFloats == head)
                    {
                        myMsg.Status = false;
                        myMsg.Message = "Buffer is full.";
                        return myMsg;
                    }

                    // Write float data to tail position
                    accessor.Write(BufferStartOffset + (tail * sizeof(float)), data);

                    // Update tail (wrap around using modulo)
                    tail = (tail + 1) % _maxFloats;
                    accessor.Write(TailOffset, tail); // Save updated tail pointer

                    myMsg.Status = true;
                    myMsg.Message = "Float enqueued successfully.";
                    return myMsg;
                }
            }
            catch (Exception ex)
            {
                myMsg.Status = false;
                myMsg.Message = $"Failed to write to shared memory: {ex.Message}";
                return myMsg;
            }
            finally
            {
                _mutex.ReleaseMutex();
            }
        }

        // Dequeue a float (read from the buffer)
        public BufferManagerMessage ReadFromBuffer()
        {
            BufferManagerMessage myMsg = new BufferManagerMessage();

            if (_mmf == null)
            {
                myMsg.Status = false;
                myMsg.Message = "Shared memory not initialized.";
                return myMsg;
            }

            if (!_mutex.WaitOne(TimeSpan.FromMilliseconds(timeout)))
            {
                myMsg.Status = false;
                myMsg.Message= $"Failed to acquire mutex, timeout {timeout}.";
                return myMsg;
            }

            try
            {
                using(MemoryMappedViewAccessor accessor = _mmf.CreateViewAccessor())
                {
                    // Read head and tail
                    int head = accessor.ReadInt32(HeadOffset);
                    int tail = accessor.ReadInt32(TailOffset);

                    // Check if buffer is empty (head == tail)
                    if(head == tail)
                    {
                        myMsg.Status = false;
                        myMsg.Message = "Buffer is empty.";
                        return myMsg;
                    }

                    // Read float data at head position
                    float data = accessor.ReadSingle(BufferStartOffset + (head * sizeof(float)));

                    // Update head (wrap around using modulo)
                    head = (head + 1) % _maxFloats;
                    accessor.Write(HeadOffset, head); // Save updated head pointer

                    myMsg.Status = true;
                    myMsg.Data = data;
                    myMsg.Message = "Float dequeued successfully.";
                    return myMsg;
                }
            }
            catch (Exception ex)
            {
                myMsg.Status = false;
                myMsg.Message = $"Failed to read from shared memory: {ex.Message}";
                return myMsg;
            }
            finally
            {
                _mutex.ReleaseMutex();
            }
        }

        public void CloseSharedMemory()
        {
            if (_mmf != null)
            {
                _mmf.Dispose();
                _mmf = null;
            }

            if (_mutex != null)
            {
                _mutex.Dispose();
                _mutex = null;
            }
        }

        private bool ConnectInventor()
        {
            Console.WriteLine("Checking Inventor connection...");
            try
            {
                try
                {
                    // Get active inventor object
                    m_InventorApplication = MarshalCore.GetActiveObject("Inventor.Application") as Inventor.Application;
                }
                catch (COMException)
                {
                    Console.WriteLine("BufferPrint: Inventor must be running.");
                    return false;
                }

                // Make sure that at least one document is opened
                m_assemblyDocument = m_InventorApplication.ActiveDocument as AssemblyDocument;
                if (m_assemblyDocument == null)
                {
                    Console.WriteLine("BufferPrint: An assembly document must be active.");
                    return false;
                }

                string fullPath = m_assemblyDocument.FullDocumentName;
                string fileName = System.IO.Path.GetFileName(fullPath);
                string projectName = System.IO.Path.GetFileNameWithoutExtension(fullPath);

                Console.WriteLine($"BufferPrint: Active document: {fileName}");
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
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

    public class InventorObjectData
    {
        public string AssemblyName { get; private set; }
        public int ComponentCount { get; private set; }
        public List<string> ComponentNames { get; private set; }
        public List<string> JointNames { get; private set; }

        private AssemblyDocument _assemblyDoc;

        public InventorObjectData(AssemblyDocument assemblyDoc)
        {
            _assemblyDoc = assemblyDoc;
            ComponentNames = new List<string>();
            JointNames = new List<string>();

            ExtractData();
        }

        private void ExtractData()
        {
            AssemblyName = System.IO.Path.GetFileNameWithoutExtension(_assemblyDoc.FullFileName);

            // Components
            ComponentOccurrences occurrences = _assemblyDoc.ComponentDefinition.Occurrences;
            ComponentCount = occurrences.Count;

            foreach (ComponentOccurrence occ in occurrences)
            {
                ComponentNames.Add(occ.Name);
            }

            // Joints
            //Joints joints = _assemblyDoc.ComponentDefinition.Joints;
            AssemblyJoints joints = _assemblyDoc.ComponentDefinition.Joints;
            Console.WriteLine($"joints count: {joints.Count}");
            foreach (AssemblyJoint joint in joints)
            {
                Console.WriteLine($"Joint name: {joint.Name}");
                double angleRad = joint.Definition.AngularPosition.Value;
                double andleDeg = angleRad * (180.0 / Math.PI);
                Console.WriteLine($"Joint angle (deg): {andleDeg}");
                JointNames.Add(joint.Name);
            }
        }

        public void PrintToConsoleAssemblyDocumentData()
        {
            Console.WriteLine($"Assembly: {AssemblyName}");
            Console.WriteLine($"Components ({ComponentCount}):");
            foreach (var name in ComponentNames)
            {
                Console.WriteLine($"  - {name}");
            }

            Console.WriteLine("Joints:");
            foreach (var joint in JointNames)
            {
                Console.WriteLine($"  - {joint}");
            }
        }

        public void PushData()
        {
            int jointNumber = 0;
            float angleDeg = 0;

            while (true)
            {
                Console.Write("Enter joint number (1 to 3): ");
                string jointInput = Console.ReadLine();

                if (int.TryParse(jointInput, out jointNumber) && jointNumber >= 1 && jointNumber <= 3)
                {
                    break;
                }
                else
                {
                    Console.WriteLine("Invalid input. Please enter a number between 1 and 3.");
                }
            }

            // Get angle (0 to 90 degrees)
            while (true)
            {
                Console.Write("Enter angle in degrees (0 to 90): ");
                string angleInput = Console.ReadLine();

                if (float.TryParse(angleInput, out angleDeg) && angleDeg >= 0 && angleDeg <= 90)
                {
                    break;
                }
                else
                {
                    Console.WriteLine("Invalid input. Please enter a number between 0 and 90.");
                }
            }

            Console.WriteLine($"You selected: Joint {jointNumber} with angle {angleDeg}°");

            AssemblyJoints joints = _assemblyDoc.ComponentDefinition.Joints;
            foreach (AssemblyJoint joint in joints)
            {
                if(joint.Name == $"Rotational:{jointNumber}")
                {
                    Console.WriteLine($"Joint name: {joint.Name}");
                    Console.WriteLine($"Joint angle (deg): {angleDeg}");
                    double angleRad = angleDeg * (Math.PI/ 180.0) * -1;
                    joint.Definition.AngularPosition.Value = angleRad;
                    _assemblyDoc.Update();
                }
            }

        }
    }


    public class BufferManagerMessage
    {
        public bool Status { get; set; }
        public float Data { get; set; }
        public string Message { get; set; }
        public override string ToString()
        {
            return $"Message: Status: {Status}, Data: {Data}, Message: {Message}";
        }
    }

    public class TreeNode<T>
    {
        public T Data { get; set; }
        public List<TreeNode<T>> Children { get; set; }

        public TreeNode(T data)
        {
            Data = data;
            Children = new List<TreeNode<T>>();
        }

        public void AddChild(TreeNode<T> child)
        {
            Children.Add(child);
        }
    }

    public class Tree<T>
    {
        public TreeNode<T> Root { get; set; }

        public Tree(T rootData)
        {
            Root = new TreeNode<T>(rootData);
        }

        public void Traverse(TreeNode<T> node, int level = 0)
        {
            if (node != null)
            {
                Console.WriteLine(new string('-', level) + node.Data);
                foreach (var child in node.Children)
                {
                    Traverse(child, level + 1);
                }
            }
        }
    }

}
