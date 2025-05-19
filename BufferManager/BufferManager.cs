using BufferManager;
using Inventor;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO.MemoryMappedFiles;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using static BufferManager.ManagementConstants;

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
        //private int _size = 4096;
        //private int _maxFloats = 100;
        private Mutex _mutex;
        private readonly int timeout = 750;

        private const int HeadOffset = 0;
        private const int TailOffset = 4;

        private const int EntryLength = 19;
        private const int MaxEntries = 100;
        private const int BufferStartOffset = 8;
        private const int _size = BufferStartOffset + EntryLength * MaxEntries;


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
        public BufferManagerMessage WriteToBuffer(string action, string jointId, float angle)
        {
            var msg = new BufferManagerMessage();

            //if (angle < 0 || angle > 180)
            //{
            //    msg.Status = false;
            //    msg.Message = $"Angle {angle} is out of valid range.";
            //    return msg;
            //}

            if (angle < -360 || angle > 360)
            {
                msg.Status = false;
                msg.Message = $"Angle {angle} is out of valid range.";
                return msg;
            }

            if (_mmf == null)
            {
                msg.Status = false;
                msg.Message = "Shared memory not initialized.";
                return msg;
            }

            if (!_mutex.WaitOne(TimeSpan.FromMilliseconds(timeout)))
            {
                msg.Status = false;
                msg.Message = "Failed to acquire mutex.";
                return msg;
            }

            try
            {
                action = action.Trim();
                jointId = jointId.Trim();

                using (var accessor = _mmf.CreateViewAccessor())
                {
                    int head = accessor.ReadInt32(HeadOffset);
                    int tail = accessor.ReadInt32(TailOffset);

                    if ((tail + 1) % MaxEntries == head)
                    {
                        msg.Status = false;
                        msg.Message = "Buffer is full.";
                        return msg;
                    }

                    // Compose entry
                    //string angleStr = angle.ToString("000.00;-000.00", System.Globalization.CultureInfo.InvariantCulture);
                    string angleStr = angle.ToString("000.00;-000.00", CultureInfo.InvariantCulture).PadLeft(7);
                    //string fullEntry = $"{action.PadRight(6).Substring(0, 6)}{jointId.PadRight(6).Substring(0, 6)}{angleStr.PadLeft(6)}";
                    string fullEntry = $"{action.PadRight(6).Substring(0, 6)}{jointId.PadRight(6).Substring(0, 6)}{angleStr}";


                    // Write entry as bytes
                    long pos = BufferStartOffset + (tail * EntryLength);
                    byte[] entryBytes = Encoding.ASCII.GetBytes(fullEntry);

                    if (entryBytes.Length != EntryLength)
                    {
                        msg.Status = false;
                        msg.Message = $"Encoded entry has invalid length: {entryBytes.Length}, expected {EntryLength}.";
                        return msg;
                    }

                    accessor.WriteArray(pos, entryBytes, 0, EntryLength);

                    // Update tail
                    tail = (tail + 1) % MaxEntries;
                    accessor.Write(TailOffset, tail);

                    msg.Status = true;
                    msg.Message = "Entry written.";
                    Console.WriteLine($"[WRITE] action={action}, jointId={jointId}, angle={angle}, pos={pos}");
                }
            }
            catch (Exception ex)
            {
                msg.Status = false;
                msg.Message = $"Write failed: {ex.Message}";
            }
            finally
            {
                _mutex.ReleaseMutex();
            }

            return msg;
        }

        // Dequeue a float (read from the buffer)
        public BufferManagerMessage ReadFromBuffer()
        {
            var msg = new BufferManagerMessage();

            if (_mmf == null)
            {
                msg.Status = false;
                msg.Message = "Shared memory not initialized.";
                return msg;
            }

            if (!_mutex.WaitOne(TimeSpan.FromMilliseconds(timeout)))
            {
                msg.Status = false;
                msg.Message = "Failed to acquire mutex.";
                return msg;
            }

            try
            {
                using (var accessor = _mmf.CreateViewAccessor())
                {
                    int head = accessor.ReadInt32(HeadOffset);
                    int tail = accessor.ReadInt32(TailOffset);

                    if (head == tail)
                    {
                        msg.Status = false;
                        msg.Message = "Buffer is empty.";
                        return msg;
                    }

                    long pos = BufferStartOffset + (head * EntryLength);
                    byte[] entryBytes = new byte[EntryLength];
                    accessor.ReadArray(pos, entryBytes, 0, EntryLength);
                    string entry = Encoding.ASCII.GetString(entryBytes);

                    if (entry.Length < EntryLength)
                    {
                        msg.Status = false;
                        msg.Message = $"Corrupted entry: expected {EntryLength} chars, got {entry.Length}.";
                        return msg;
                    }


                    // Parse fields
                    string action = entry.Substring(0, 6).Trim();
                    string jointId = entry.Substring(6, 6).Trim();
                    //string angleStr = entry.Substring(12).Trim();
                    string angleStr = entry.Substring(12, 7).Trim();

                    if (!float.TryParse(angleStr, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float angle))
                    {
                        msg.Status = false;
                        msg.Message = $"Failed to parse float from '{angleStr}' in entry '{entry}'";
                        return msg;
                    }

                    // Update head
                    head = (head + 1) % MaxEntries;
                    accessor.Write(HeadOffset, head);

                    msg.Status = true;
                    msg.Action = action;
                    msg.JointId = jointId;
                    msg.Data = angle;
                    msg.Message = "Entry read successfully.";
                }
            }
            catch (Exception ex)
            {
                msg.Status = false;
                msg.Message = $"Read failed: {ex.Message}";
            }
            finally
            {
                _mutex.ReleaseMutex();
            }

            return msg;
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
        }

        public void ExtractAssemblyData()
        {
            AssemblyName = System.IO.Path.GetFileNameWithoutExtension(_assemblyDoc.FullFileName);

            // Components
            ComponentOccurrences occurrences = _assemblyDoc.ComponentDefinition.Occurrences;
            ComponentCount = occurrences.Count;

            foreach (ComponentOccurrence occ in occurrences)
            {
                ComponentNames.Add(occ.Name);
                Console.WriteLine($"Component name: {occ.Name}");
            }

            // Joints
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

        public BufferManagerMessage CommandData(BufferManagerMessage msg)
        {
            if (!msg.Status)
            {
                Console.WriteLine("Error reading buffer: " + msg.Message);
                msg.Status = false;
                msg.Message = $"Error reading buffer: {msg.Message}";
                return msg;
            }

            //if (msg.Data < 0 || msg.Data > 90)
            //{
            //    Console.WriteLine($"Invalid angle value: {msg.Data}");
            //    msg.Status = false;
            //    msg.Message = $"Invalid angle value: {msg.Data}";
            //    return msg;
            //}

            if (!Enum.TryParse<BufferManagerAction>(msg.Action, ignoreCase: false, out var actionEnum))
            {
                Console.WriteLine($"Invalid action: '{msg.Action}'");
                msg.Status = false;
                msg.Message = $"Invalid action: '{msg.Action}'";
                return msg;
            }

            // 2. Validate Joint ID Enum
            if (Enum.TryParse<BufferManagerElement>(msg.JointId, out var jointEnum) &&
            JointNameMapper.Map.TryGetValue(jointEnum, out string inventorJointName))
            {
                // Use inventorJointName to find and update the joint in the Inventor assembly
                Console.WriteLine($"[Buffer Message] Action: {msg.Action}, JointId: {msg.JointId}, Angle: {msg.Data}");
                Console.WriteLine($"Inventor joint name: {inventorJointName}");

                int actionCtn = (int)Enum.Parse(typeof(BufferManagerAction), msg.Action);

                AssemblyJoints joints = _assemblyDoc.ComponentDefinition.Joints;

                foreach (AssemblyJoint joint in joints)
                {
                    if (actionCtn == (int)BufferManagerAction.writte)
                    {
                        if (joint.Name == inventorJointName)
                        {
                            Console.WriteLine($"Applying {msg.Data}° to jointId: {joint.Name} and jointName: {inventorJointName}");
                            //double angleRad = angleDeg * (Math.PI / 180.0) * -1;

                            try
                            {
                                joint.Definition.AngularPosition.Value = (double)msg.Data * (Math.PI / 180.0);
                                _assemblyDoc.Update();
                                Console.WriteLine("Joint updated.");

                                msg.Status = true;
                                msg.Message = $"Joint {joint.Name}: {inventorJointName} updated to {msg.Data}°";
                            }
                            catch (Exception ex)
                            {
                                Console.WriteLine($"Failed to set joint angle: Inventor API update Error: {ex.Message}");
                                msg.Status = false;
                                msg.Message = $"Failed to set joint angle: {ex.Message}";
                            }
                            return msg;
                        }
                    }

                    if (actionCtn == (int)BufferManagerAction.read__)
                    {
                        if (joint.Name == inventorJointName)
                        {
                            Console.WriteLine($"Applying {msg.Data}° to jointId: {joint.Name} and jointName: {inventorJointName}");

                            try
                            {
                                float angleDeg = (float)joint.Definition.AngularPosition.Value * (180.0f / (float)Math.PI);
                                Console.WriteLine($"Joint angle (deg): {angleDeg}");
                                msg.Status = true;
                                msg.Data = angleDeg;
                                msg.Message = $"Joint {joint.Name}: {inventorJointName} angle is {angleDeg}°";
                            }
                            catch (Exception ex)
                            {
                                Console.WriteLine($"Failed to read joint angle: Inventor API read Error: {ex.Message}");
                                msg.Status = false;
                                msg.Message = $"Failed to set joint angle: {ex.Message}";
                            }
                            return msg;
                        }
                    }
                }

                Console.WriteLine($"Joint not found: {inventorJointName}");
                msg.Status = false;
                msg.Message = $"Joint not found: {inventorJointName}";
                return msg;
            }
            else
            {
                Console.WriteLine($"Invalid or unmapped jointId: {msg.JointId}");

                msg.Status = false;
                msg.Message = $"Invalid or unmapped jointId: {msg.JointId}";
                return msg;
            }
        }
    }

    public class ProthesisData
    {
        public SimpleFinger IndexFinger { get; set; }
        public SimpleFinger MiddleFinger { get; set; }
        public SimpleFinger RingFinger { get; set; }
        public SimpleFinger PinkyFinger { get; set; }
        public ComplexFinger Thumb { get; set; }

        public ProthesisData()
        {
            IndexFinger = new SimpleFinger();
            MiddleFinger = new SimpleFinger();
            RingFinger = new SimpleFinger();
            PinkyFinger = new SimpleFinger();
            Thumb = new ComplexFinger();
        }

        public void PrintData()
        {
            Console.WriteLine("Index: " + IndexFinger);
            Console.WriteLine("Middle: " + MiddleFinger);
            Console.WriteLine("Ring: " + RingFinger);
            Console.WriteLine("Pinky: " + PinkyFinger);
            Console.WriteLine("Thumb: " + Thumb);
        }
    }

    public class SimpleFinger
    {
        public double Joint1 { get; set; }
        public double Joint2 { get; set; }
        public double Joint3 { get; set; }

        public SimpleFinger(double j1 = 0, double j2 = 0, double j3 = 0)
        {
            Joint1 = j1;
            Joint2 = j2;
            Joint3 = j3;
        }

        public override string ToString()
        {
            return $"Joints: [1: {Joint1}, 2: {Joint2}, 3: {Joint3}]";
        }
    }

    public class ComplexFinger
    {
        public double Joint1 { get; set; }
        public double Joint2 { get; set; }
        public double Joint3 { get; set; }

        public ComplexFinger(double j1 = 0, double j2 = 0, double j3 = 0)
        {
            Joint1 = j1;
            Joint2 = j2;
            Joint3 = j3;
        }

        public override string ToString()
        {
            return $"Joints: [1: {Joint1}, 2: {Joint2}, 3: {Joint3}]";
        }
    }

    public class BufferManagerMessage
    {
        public bool Status { get; set; }
        public string Action { get; set; }
        public string JointId { get; set; }
        public float Data { get; set; }
        public string Message { get; set; }

        public override string ToString()
        {
            return $"Action: {Action}, Joint: {JointId}, Angle: {Data}, Status: {Status}, Message: {Message}";
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
