using BufferManager;
using Inventor;
using Newtonsoft.Json;
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
        private BufferPrint.InventorObjectData _inventorObject;
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

        private bool inventorDataExtracted = false;
        private bool isConnectedToInventor = false;

        public InventorObjectData GetInventorObjectData()
        {
            return _inventorObject;
        }

        public void SetInventorObjectData(InventorObjectData inventorObject)
        {
            _inventorObject = inventorObject;
        }

        public void SetInventorAppInstance(Application inventorApp)
        {
            m_InventorApplication = inventorApp;
        }

        public Application GetInventorAppInstance()
        {
            if (Init())
            {
                return m_InventorApplication;
            }
            else
            {
                return null;
            }
        }

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
            if (!isConnectedToInventor)
            {
                if (ConnectInventor())
                {
                    isConnectedToInventor = true;
                    return true;
                }
                return false;
            }
            else
            {
                return true;
            }
        }

        public MassCenterAccumulator GetSubassemblyAccumulator()
        {
            if (!Init())
            {
                Console.WriteLine("Press any key to exit...");
                Console.ReadKey();
                MassCenterAccumulator tmp = new MassCenterAccumulator();
                return tmp;
            }
            else
            {
                if (!inventorDataExtracted)
                {
                    _inventorObject.ExtractAssemblyData();
                    inventorDataExtracted = true;
                }
                return _inventorObject.accumulator;
            }
        }

        //public string GetsubassemblyAccumulatorJson()
        //{
        //    var dtos = _inventorObject.accumulator.ToDTOs();
        //    return JsonConvert.SerializeObject(dtos);
        //}

    }

    public class InventorObjectData
    {
        public string AssemblyName { get; private set; }
        public int ComponentCount { get; private set; }
        public List<string> ComponentNames { get; private set; }
        public List<string> JointNames { get; private set; }
        public MassCenterAccumulator accumulator { get; private set; } = new MassCenterAccumulator();

        private AssemblyDocument _assemblyDoc;
        private Application _inventorApp;

        public InventorObjectData(Application inventorApp, AssemblyDocument assemblyDoc)
        {
            _assemblyDoc = assemblyDoc;
            _inventorApp = inventorApp;
            ComponentNames = new List<string>();
            JointNames = new List<string>();
        }

        void ProcessOccurrence(ComponentOccurrence occurrence, MassCenterAccumulator accumulator)
        {
            Document doc = occurrence.Definition.Document;

            if (doc.DocumentType == DocumentTypeEnum.kPartDocumentObject)
            {
                // Leaf part — ignore, we only process at phalanx assembly level
                return;
            }

            if (doc.DocumentType == DocumentTypeEnum.kAssemblyDocumentObject)
            {
                string fingerName = WhichPhalanxSubassembly(occurrence);

                if(fingerName != "")
                {
                    ProcessPhalanx(occurrence, fingerName, accumulator);
                }
                else
                {
                    Console.WriteLine("ProcessOccurrence method: this ocurrence is not part of a finger set");
                }
            }
        }

        private void ProcessPhalanx(ComponentOccurrence phalanxOccurrence, string phalanxName, MassCenterAccumulator accumulator)
        {
            // Inverse transform to bring global COMs into this phalanx's frame
            Matrix inverseTransform = phalanxOccurrence.Transformation.Copy();
            inverseTransform.Invert();

            // Get sub-occurrences (parts only)
            AssemblyComponentDefinition phalanxDef = ((AssemblyDocument)phalanxOccurrence.Definition.Document).ComponentDefinition;

            foreach (ComponentOccurrence partOcc in phalanxDef.Occurrences)
            {
                if (partOcc.DefinitionDocumentType == DocumentTypeEnum.kPartDocumentObject)
                {
                    double mass = partOcc.MassProperties.Mass;

                    Point partOccurenceCOM = partOcc.MassProperties.CenterOfMass;

                    // NOTA: finalemente globalCOM es la variable que guarda el valor de COM que corresponde a la phalanx en coordenadas locales de phalanx
                    //Point localCOM = TransformPoint(globalCOM, inverseTransform, _inventorApp);

                    accumulator.Add(phalanxName, mass, partOccurenceCOM);
                }
            }

            foreach (ComponentOccurrence partOcc in phalanxDef.Occurrences)
            {
                if (partOcc.DefinitionDocumentType == DocumentTypeEnum.kPartDocumentObject)
                {
                    MassProperties massProps = partOcc.MassProperties;
                    double Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Iyz = 0, Ixz = 0;

                    // get full local inertia tensor (about COM, in part frame)
                    massProps.XYZMomentsOfInertia(out Ixx, out Iyy, out Izz, out Ixy, out Iyz, out Ixz);

                    //float factorConversion = 292.6397f;     // lb_in2 to kg mm2
                    //float factorConversion = 1.0f;     //
                    float factorConversion = 100.0f;     // kg cm2 to kg mm2

                    double[,] I_local = new double[3, 3]
                    {
                        { Ixx * factorConversion, -Ixy * factorConversion, -Ixz * factorConversion },
                        { -Ixy * factorConversion, Iyy * factorConversion, -Iyz * factorConversion },
                        { -Ixz * factorConversion, -Iyz * factorConversion, Izz *factorConversion }
                    };

                    // extract rotatiaon from part frame to phalanx frame
                    Matrix T = partOcc.Transformation;
                    double[,] R = new double[3, 3]
                    {
                        { T.Cell[1,1], T.Cell[1,2], T.Cell[1,3] },
                        { T.Cell[2,1], T.Cell[2,2], T.Cell[2,3] },
                        { T.Cell[3,1], T.Cell[3,2], T.Cell[3,3] }
                    };

                    // rotate inertia tensor to phalanx frame
                    double[,] R_T = Transpose(R);
                    double[,] I_rotated = Multiply(Multiply(R, I_local), R_T);      // matrix de inercia en frame de phalanx assembly,
                                                                                    // expressed in the phalanx assembly's local coordinate frame
                                                                                    // and measured about the phalanx's center of mass

                    // paralle axis theorem
                    Point partOccurenceCOM = partOcc.MassProperties.CenterOfMass;
                    Point assemblyCOM = accumulator.GetCenterOfMass(phalanxName, _inventorApp);

                    double dx = partOccurenceCOM.X - assemblyCOM.X;
                    double dy = partOccurenceCOM.Y - assemblyCOM.Y;
                    double dz = partOccurenceCOM.Z - assemblyCOM.Z;

                    double[,] shift = new double[3, 3]
                    {
                        { dy*dy + dz*dz, -dx*dy,       -dx*dz },
                        { -dy*dx,        dx*dx + dz*dz, -dy*dz },
                        { -dz*dx,        -dz*dy,       dx*dx + dy*dy }
                    };

                    double mass = partOcc.MassProperties.Mass;

                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++)
                            I_rotated[i, j] += mass * shift[i, j];

                    accumulator.SetInertiaMatrixCOM(phalanxName, I_rotated);

                    // translate inertia matrix from the assembly COM to the phalanx origin
                    // Apply second shift: from COM to joint (origin of phalanx frame)
                    double dx_joint = assemblyCOM.X; // assuming joint is at origin (0,0,0)
                    double dy_joint = assemblyCOM.Y;
                    double dz_joint = assemblyCOM.Z;

                    double[,] shiftToJoint = new double[3, 3]
                    {
                        { dy_joint*dy_joint + dz_joint*dz_joint, -dx_joint*dy_joint, -dx_joint*dz_joint },
                        { -dy_joint*dx_joint, dx_joint*dx_joint + dz_joint*dz_joint, -dy_joint*dz_joint },
                        { -dz_joint*dx_joint, -dz_joint*dy_joint, dx_joint*dx_joint + dy_joint*dy_joint }
                    };

                    // Now shift entire part contribution to joint origin
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++)
                            I_rotated[i, j] += mass * shiftToJoint[i, j];


                    //accumulator.SetInertiaMatrix(I_rotated);
                    accumulator.SetInertiaMatrix(phalanxName, I_rotated);
                }
            }
        }

        double[,] Multiply(double[,] A, double[,] B)
        {
            double[,] result = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                        result[i, j] += A[i, k] * B[k, j];
            return result;
        }

        double[,] Transpose(double[,] A)
        {
            double[,] result = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    result[i, j] = A[j, i];
            return result;
        }

        private string WhichPhalanxSubassembly(ComponentOccurrence occ)
        {
            string name = occ.Name.ToLower();

            if(!name.Contains("proximal") && !name.Contains("middle") && !name.Contains("distal"))
            {
                return ""; // Not a phalanx subassembly
            }

            if (name.Contains("proximal") || name.Contains("middle") || name.Contains("distal"))
            {
                string[] phalanxTypes = new[] { "proximal", "middle", "distal" };
                string matchedType = phalanxTypes.FirstOrDefault(type => name.Contains(type));

                if (name.Contains("1")) return $"{matchedType}Ring"; // Ring finger
                if (name.Contains("2")) return $"{matchedType}Middle"; // Middle finger
                if (name.Contains("3")) return $"{matchedType}Picky"; // Picky finger
                if (name.Contains("4")) return $"{matchedType}Index"; // Index finger
                if (name.Contains("5")) return $"{matchedType}Thumb"; // Thumb finger
            }

            return "";
        }

        public void ExtractAssemblyData()
        {
            AssemblyName = System.IO.Path.GetFileNameWithoutExtension(_assemblyDoc.FullFileName);

            // Components
            ComponentOccurrences occurrences = _assemblyDoc.ComponentDefinition.Occurrences;
            ComponentCount = occurrences.Count;

            foreach (ComponentOccurrence occ in occurrences)
            {
                //Console.WriteLine($"Component name: {occ.Name}");

                // Entry point from your top-level assembly document
                ProcessOccurrence(occ, accumulator);
            }

            // printing subassemblies properties
            foreach (string label in accumulator.Labels)
            {
                double totalMass = accumulator.GetTotalMass(label);
                Inventor.Point com = accumulator.GetCenterOfMass(label, _inventorApp);

                double[,] inertialMat = accumulator.GetInertiaMatrix(label);

                //Console.WriteLine($"[Phalanx: {label}] Total Mass: {totalMass} kg, Center of Mass (local frame): X={com.X} cm, Y={com.Y} cm, Z={com.Z} cm, I_COMx={inertialMat[0, 0]} kg·mm², I_COMy={inertialMat[1, 1]} kg·mm², I_COMz={inertialMat[2, 2]} kg·mm²");
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

    // Struct to accumulate total mass and weighted COM sum
    public class MassCenterData
    {
        public double TotalMass { get; set; }
        public double WeightedX { get; set; }
        public double WeightedY { get; set; }
        public double WeightedZ { get; set; }

        private double[,] _InertiaMatrix;
        private double[,] _InertiaMatrixCOM;

        public void Add(double mass, Inventor.Point com, bool convertToMm2 = true)
        {
            TotalMass += mass;
            WeightedX += mass * com.X;
            WeightedY += mass * com.Y;
            WeightedZ += mass * com.Z;
        }

        public Inventor.Point GetCenterOfMass(Inventor.Application app)
        {
            if (TotalMass == 0)
                return app.TransientGeometry.CreatePoint(0, 0, 0);

            return app.TransientGeometry.CreatePoint(
                WeightedX / TotalMass,
                WeightedY / TotalMass,
                WeightedZ / TotalMass
            );
        }

        public PointDTO GetCenterOfMassDTO()
        {
            if (TotalMass == 0)
                return new PointDTO(0, 0, 0);

            return new PointDTO(
                WeightedX / TotalMass,
                WeightedY / TotalMass,
                WeightedZ / TotalMass
            );
        }

        public class PointDTO
        {
            public double X { get; set; }
            public double Y { get; set; }
            public double Z { get; set; }

            public PointDTO(double x, double y, double z)
            {
                X = x;
                Y = y;
                Z = z;
            }

            public PointDTO()
            {
                X = double.NaN;
                Y = double.NaN;
                Z = double.NaN;
            }
        }

        public double[,] GetInertia() => _InertiaMatrix;

        public void SetInertia(double[,] inertia) => _InertiaMatrix = inertia;

        public double[,] GetInertiaCOM() => _InertiaMatrixCOM;

        public void SetInertiaCOM(double[,] inertiaCOM) => _InertiaMatrixCOM = inertiaCOM;
    }

    public class MassCenterDTO
    {
        public string Label { get; set; }
        public double TotalMass { get; set; }
        public double[] CenterOfMass { get; set; }  // [x, y, z]
        public double[] InertiaMatrixFlat { get; set; } // 3x3 matrix as flat array (9 elements)
    }


    public class MassCenterAccumulator
    {
        private Dictionary<string, MassCenterData> _dataMap;

        public MassCenterAccumulator()
        {
            _dataMap = new Dictionary<string, MassCenterData>();
        }

        public void Add(string label, double mass, Inventor.Point com)
        //public void Add(string label, double mass, Inventor.Point com, double ixx, double iyy, double izz)
        {
            if (!_dataMap.ContainsKey(label))
                _dataMap[label] = new MassCenterData();

            MassCenterData current = _dataMap[label];
            //current.Add(mass, com, ixx, iyy, izz);
            current.Add(mass, com);
            _dataMap[label] = current;
        }

        public Point GetCenterOfMass(string label, Inventor.Application app)
        {
            if (_dataMap.ContainsKey(label))
            {
                return _dataMap[label].GetCenterOfMass(app);
            }

            return null;
        }

        public MassCenterData.PointDTO GetCenterOfMassDTO(string label)
        {
            if (_dataMap.ContainsKey(label))
            {
                return _dataMap[label].GetCenterOfMassDTO();
            }

            return new MassCenterData.PointDTO();
        }

        public double GetTotalMass(string label)
        {
            return _dataMap.ContainsKey(label) ? _dataMap[label].TotalMass : 0.0;
        }

        public IEnumerable<string> Labels => _dataMap.Keys;

        public void SetInertiaMatrix(string label, double[,] InertiaMatrix)
        {
            if (!_dataMap.ContainsKey(label))
                _dataMap[label] = new MassCenterData();

            MassCenterData current = _dataMap[label];
            current.SetInertia(InertiaMatrix);
            _dataMap[label] = current;
        }

        public void SetInertiaMatrixCOM(string label, double[,] InertiaMatrix)
        {
            if (!_dataMap.ContainsKey(label))
                _dataMap[label] = new MassCenterData();

            MassCenterData current = _dataMap[label];
            current.SetInertia(InertiaMatrix);
            _dataMap[label] = current;
        }

        public double[,] GetInertiaMatrix(string label)
        {
            MassCenterData current = _dataMap[label];
            return current.GetInertia();
        }

        //public List<MassCenterDTO> ToDTOs()
        //{
        //    var result = new List<MassCenterDTO>();

        //    foreach (var kvp in _dataMap)
        //    {
        //        var label = kvp.Key;
        //        var data = kvp.Value;

        //        var com = data.GetCenterOfMass(null);  // Replace with appropriate app or adapt data to store XYZ directly

        //        double[] comArray = new double[] { com.X, com.Y, com.Z };

        //        double[,] matrix = data.GetInertia();
        //        double[] inertiaFlat = matrix != null ?
        //            new double[] {
        //        matrix[0,0], matrix[0,1], matrix[0,2],
        //        matrix[1,0], matrix[1,1], matrix[1,2],
        //        matrix[2,0], matrix[2,1], matrix[2,2]
        //            } : new double[9];

        //        result.Add(new MassCenterDTO
        //        {
        //            Label = label,
        //            TotalMass = data.TotalMass,
        //            CenterOfMass = comArray,
        //            InertiaMatrixFlat = inertiaFlat
        //        });
        //    }

        //    return result;
        //}

    }


}
