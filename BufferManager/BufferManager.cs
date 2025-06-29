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
using System.Reflection.Emit;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Linq;
using static BufferManager.ManagementConstants;
using static BufferPrint.MassCenterData;

namespace BufferPrint
{
    public class BufferManager
    {
        private Inventor.Application m_InventorApplication;
        private Inventor.AssemblyDocument m_assemblyDocument;
        private BufferPrint.InventorObjectData _inventorObject;
        private ArrayList m_ComponentOccurrenceArray = new ArrayList();
        private ArrayList m_JointInfoArray = new ArrayList();

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
                Console.WriteLine("Buffer Manager Not initialized");
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

        public SpecialTransformation GetOriginTransformation()
        {
            if (!Init())
            {
                Console.WriteLine("Buffer Manager Not initialized");
                Console.ReadKey();
                return new SpecialTransformation();
            }
            else
            {
                if (!inventorDataExtracted)
                {
                    _inventorObject.ExtractAssemblyData();
                    inventorDataExtracted = true;
                }

                UCSData tmpUCS = _inventorObject._ModelOrigin;

                return new SpecialTransformation(
                    tmpUCS.UCSMat.Cell[1, 1] / 100.0,
                    tmpUCS.UCSMat.Cell[2, 2] / 100.0,
                    tmpUCS.UCSMat.Cell[3, 3] / 100.0,
                    tmpUCS.UCSMat.Cell[1, 2] / 100.0,
                    tmpUCS.UCSMat.Cell[1, 3] / 100.0,
                    tmpUCS.UCSMat.Cell[2, 1] / 100.0,
                    tmpUCS.UCSMat.Cell[2, 3] / 100.0,
                    tmpUCS.UCSMat.Cell[3, 1] / 100.0,
                    tmpUCS.UCSMat.Cell[3, 2] / 100.0,
                    tmpUCS.UCSMat.Cell[1, 4] / 100.0,
                    tmpUCS.UCSMat.Cell[2, 4] / 100.0,
                    tmpUCS.UCSMat.Cell[3, 4] / 100.0
                );
            }
        }
    
        public SpecialTransformation ComputeRotationWithTheta(double theta1)
        {
            SpecialTransformation originTranformation = GetOriginTransformation();

            List<UCSData> proximalPhalanxUCSs = GetSubassemblyAccumulator().GetUCSs("proximalThumb");

            Matrix UCS2Data = null;

            foreach(UCSData tmpUCS in proximalPhalanxUCSs)
            {
                Console.WriteLine(String.Format("UCS Name: %s", tmpUCS.Name));

                if("UCS2" == tmpUCS.Name)
                {
                    UCS2Data = tmpUCS.UCSMat;
                    break;
                }
            }

            if(UCS2Data == null)
            {
                Console.WriteLine("UCS2 not found in the UCS list");
                throw new Exception("UCS2 not found in the UCS list");
            }

            // Assuming 'invApp' is your Inventor.Application object
            Inventor.TransientGeometry tg = m_InventorApplication.TransientGeometry;

            // Create a new Matrix (4x4 identity matrix)
            Inventor.Matrix T0 = tg.CreateMatrix();
            T0.Cell[0, 0] = originTranformation.XX;
            T0.Cell[0, 1] = originTranformation.XY;
            T0.Cell[0, 2] = originTranformation.XZ;
            T0.Cell[0, 3] = 0.0;
            T0.Cell[1, 0] = originTranformation.YX;
            T0.Cell[1, 1] = originTranformation.YY;
            T0.Cell[1, 2] = originTranformation.YZ;
            T0.Cell[1, 3] = 0.0;
            T0.Cell[2, 0] = originTranformation.ZX;
            T0.Cell[2, 1] = originTranformation.ZY;
            T0.Cell[2, 2] = originTranformation.ZZ;
            T0.Cell[2, 3] = 0.0;
            T0.Cell[3, 0] = originTranformation.Trans_x;
            T0.Cell[3, 1] = originTranformation.Trans_y;
            T0.Cell[3, 2] = originTranformation.Trans_z;
            T0.Cell[3, 3] = 1.0;

            T0.Invert();

            // Step 4: Convert UCS2Data to Inventor.Matrix (T1)
            Inventor.Matrix T1 = tg.CreateMatrix();
            for (int r = 1; r <= 4; r++)
            {
                for (int c = 1; c <= 4; c++)
                {
                    T1.set_Cell(r, c, UCS2Data.Cell[r - 1, c - 1]);
                }
            }

            // Step 5: Compute T_base = T0⁻¹ * T1
            Inventor.Matrix T_base = T0.Copy();
            T_base.PreMultiplyBy(T1);  // T_base = T0⁻¹ * T1

            // Step 6: Get rotation axis (x-axis of UCS1)
            double[] x_axis = new double[]
            {
                originTranformation.XX,
                originTranformation.YX,
                originTranformation.ZX
            };

            // Step 7: Get origin translation
            Inventor.Vector originVector = tg.CreateVector(
                originTranformation.Trans_x,
                originTranformation.Trans_y,
                originTranformation.Trans_z
            );

            // Step 8: Build axis-angle rotation matrix (R_rot)
            Inventor.Matrix R_rot = CreateAxisAngleRotationMatrix(theta1, x_axis);

            // Step 9: Create translation matrices
            Inventor.Matrix T_translate = tg.CreateMatrix();
            T_translate.set_Cell(1, 4, originVector.X);
            T_translate.set_Cell(2, 4, originVector.Y);
            T_translate.set_Cell(3, 4, originVector.Z);

            Inventor.Matrix T_translate_inv = tg.CreateMatrix();
            T_translate_inv.set_Cell(1, 4, -originVector.X);
            T_translate_inv.set_Cell(2, 4, -originVector.Y);
            T_translate_inv.set_Cell(3, 4, -originVector.Z);

            // Step 10: Apply rotation about the point (origin)
            Inventor.Matrix R_about_point = T_translate.Copy();
            R_about_point.PreMultiplyBy(R_rot);
            R_about_point.PreMultiplyBy(T_translate_inv);

            // Step 11: Final combined transformation
            Inventor.Matrix T_combined = R_about_point.Copy();
            T_combined.PreMultiplyBy(T_base);

            // Step 12: Extract the 3x3 rotation and return it wrapped
            SpecialTransformation result = new SpecialTransformation();

            result.XX = T_combined.get_Cell(1, 1);
            result.XY = T_combined.get_Cell(1, 2);
            result.XZ = T_combined.get_Cell(1, 3);

            result.YX = T_combined.get_Cell(2, 1);
            result.YY = T_combined.get_Cell(2, 2);
            result.YZ = T_combined.get_Cell(2, 3);

            result.ZX = T_combined.get_Cell(3, 1);
            result.ZY = T_combined.get_Cell(3, 2);
            result.ZZ = T_combined.get_Cell(3, 3);

            result.Trans_x = 0.0;
            result.Trans_y = 0.0;
            result.Trans_z = 0.0;

            Console.WriteLine("Computed computeRotationWithTheta once");
            return result;

        }
        private Inventor.Matrix CreateAxisAngleRotationMatrix(double angleRad, double[] axis)
        {
            // Normalize axis
            double x = axis[0], y = axis[1], z = axis[2];
            double norm = Math.Sqrt(x * x + y * y + z * z);
            x /= norm;
            y /= norm;
            z /= norm;

            double c = Math.Cos(angleRad);
            double s = Math.Sin(angleRad);
            double t = 1 - c;

            Inventor.TransientGeometry tg = m_InventorApplication.TransientGeometry;
            Inventor.Matrix R = tg.CreateMatrix();

            R.set_Cell(1, 1, t * x * x + c);
            R.set_Cell(1, 2, t * x * y - s * z);
            R.set_Cell(1, 3, t * x * z + s * y);

            R.set_Cell(2, 1, t * x * y + s * z);
            R.set_Cell(2, 2, t * y * y + c);
            R.set_Cell(2, 3, t * y * z - s * x);

            R.set_Cell(3, 1, t * x * z - s * y);
            R.set_Cell(3, 2, t * y * z + s * x);
            R.set_Cell(3, 3, t * z * z + c);

            return R;
        }


    }

    public class InventorObjectData
    {
        public string AssemblyName { get; private set; }
        public int ComponentCount { get; private set; }
        public List<string> ComponentNames { get; private set; }
        public List<string> JointNames { get; private set; }
        public MassCenterAccumulator accumulator { get; private set; } = new MassCenterAccumulator();
        public UCSData _ModelOrigin { get; private set; }

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

                // check for part ucs
                CheckPartUCS(occurrence);
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

        private void CheckPartUCS(ComponentOccurrence partOccurrence)
        {
            if (partOccurrence.DefinitionDocumentType == DocumentTypeEnum.kPartDocumentObject)
            {
                PartComponentDefinition asmDef = (PartComponentDefinition)partOccurrence.Definition;
                UserCoordinateSystems ucsList = asmDef.UserCoordinateSystems;

                Console.WriteLine(string.Format("part - ucsList length: {0}", ucsList.Count));

                foreach (UserCoordinateSystem ucs in ucsList)
                {
                    Console.WriteLine("assembly - UCS Name: " + ucs.Name);
                    Matrix UCS_Mat = ucs.Transformation; // UCS → Global
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

            if (phalanxOccurrence.DefinitionDocumentType == DocumentTypeEnum.kAssemblyDocumentObject)
            {
                AssemblyComponentDefinition asmDef = (AssemblyComponentDefinition)phalanxOccurrence.Definition;
                UserCoordinateSystems ucsList = asmDef.UserCoordinateSystems;

                Console.WriteLine(string.Format("assembly - ucsList length: {0}", ucsList.Count));

                foreach (UserCoordinateSystem ucs in ucsList)
                {
                    Console.WriteLine("assembly - UCS Name: " + ucs.Name);
                    Matrix UCS_Mat = ucs.Transformation; // UCS → Global
                    accumulator.AddUCS(phalanxName, ucs.Name, UCS_Mat);
                }

                Matrix ucs1_Mat = null;
                Matrix ucs2_Mat = null;

                List<UCSData> UCSs = accumulator.GetUCSs(phalanxName);
                foreach (UCSData ucsData in UCSs)
                {

                    Console.WriteLine($"UCS Name: {ucsData.Name}");
                    //Console.WriteLine(MatrixToString(ucsData.UCSMat));

                    if (ucsData.Name == "UCS1")
                    {
                        ucs1_Mat = ucsData.UCSMat;
                    }
                    if (ucsData.Name == "UCS2")
                    {
                        ucs2_Mat = ucsData.UCSMat;
                    }
                }

                if(ucs1_Mat != null && ucs2_Mat != null)
                {
                    Console.WriteLine("Processing transform...");
                    Matrix m1To2 = ucs2_Mat.Copy();   // M2⁻¹ * M1
                    m1To2.Invert();
                    m1To2.TransformBy(ucs1_Mat);
                    Matrix m2To1 = m1To2.Copy();
                    m2To1.Invert();          // now transforms UCS-2 → UCS-1

                    accumulator.SetSpecialTransformation(phalanxName, m2To1);

                    Console.WriteLine(MatrixToString(m2To1));
                }
            }
        }

        private string MatrixToString(Matrix mat)
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            for (int r = 1; r <= 4; r++)
            {
                for (int c = 1; c <= 4; c++)
                {
                    sb.AppendFormat("{0,10:F4}", mat.get_Cell(r, c));
                }
                sb.AppendLine();
            }
            return sb.ToString();
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

                if (name.Contains("1")) return $"{matchedType}Middle"; // Ring finger
                if (name.Contains("2")) return $"{matchedType}Ring"; // Middle finger
                if (name.Contains("3")) return $"{matchedType}Index"; // Picky finger
                if (name.Contains("4")) return $"{matchedType}Picky"; // Index finger
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

            // getting root frame
            AssemblyComponentDefinition asmDef = _assemblyDoc.ComponentDefinition;
            UserCoordinateSystems ucsList = asmDef.UserCoordinateSystems;

            Console.WriteLine(string.Format("root - ucsList length: {0}", ucsList.Count));

            foreach (UserCoordinateSystem ucs in ucsList)
            {
                Console.WriteLine("root - UCS Name: " + ucs.Name);
                Matrix UCS_Mat = ucs.Transformation; // UCS → Global
                _ModelOrigin = new UCSData(ucs.Name, UCS_Mat);
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

    public class UCSData
    {
        public string Name { get; set; }
        public Matrix UCSMat { get; set; }

        public UCSData(string name, Matrix ucsMatrix)
        {
            Name = name;
            UCSMat = ucsMatrix;
        }
        public UCSData(UCSData myUCSData)
        {
            Name = myUCSData.Name;
            UCSMat = myUCSData.UCSMat.Copy(); // Create a copy of the matrix to avoid reference issues
        }
        public SpecialTransformation GetMatDto()
        {
            Matrix UCSMat = this.UCSMat;

            return new SpecialTransformation(
                UCSMat.Cell[1, 1],
                UCSMat.Cell[2, 2],
                UCSMat.Cell[3, 3],
                UCSMat.Cell[1, 2],
                UCSMat.Cell[1, 3],
                UCSMat.Cell[2, 1],
                UCSMat.Cell[2, 3],
                UCSMat.Cell[3, 1],
                UCSMat.Cell[3, 2],
                UCSMat.Cell[1, 4],
                UCSMat.Cell[2, 4],
                UCSMat.Cell[3, 4]
                );
        }
    }

    // Object to accumulate total mass and weighted COM sum
    public class MassCenterData
    {
        public double TotalMass { get; set; }
        public double WeightedX { get; set; }
        public double WeightedY { get; set; }
        public double WeightedZ { get; set; }

        private double[,] _InertiaMatrix;
        private double[,] _InertiaMatrixCOM;
        private List<UCSData> _UCSPositions = new List<UCSData>();
        private Matrix _SpecialTransformation;

        public void Add(double mass, Inventor.Point com, bool convertToMm2 = true)
        {
            TotalMass += mass;
            WeightedX += mass * com.X / 100.0;
            WeightedY += mass * com.Y / 100.0;
            WeightedZ += mass * com.Z / 100.0;
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

        public class SpecialTransformation
        {
            public double XX { get; set; }
            public double YY { get; set; }
            public double ZZ { get; set; }
            public double XY { get; set; }
            public double XZ { get; set; }
            public double YX { get; set; }
            public double YZ { get; set; }
            public double ZX { get; set; }
            public double ZY { get; set; }
            public double Trans_x { get; set; }
            public double Trans_y { get; set; }
            public double Trans_z { get; set; }

            public SpecialTransformation(double xx, double yy, double zz, double xy, double xz, double yx, double yz, double zx, double zy, double trans_x, double trans_y, double trans_z)
            {
                this.XX = xx;
                this.YY = yy;
                this.ZZ = zz;
                this.XY = xy;
                this.XZ = xz;
                this.YX = yx;
                this.YZ = yz;
                this.ZX = zx;
                this.ZY = zy;
                this.Trans_x = trans_x;
                this.Trans_y = trans_y;
                this.Trans_z = trans_z;
            }

            public SpecialTransformation()
            {
                this.XX = double.NaN;
                this.YY = double.NaN;
                this.ZZ = double.NaN;
                this.XY = double.NaN;
                this.XZ = double.NaN;
                this.YX = double.NaN;
                this.YZ = double.NaN;
                this.ZX = double.NaN;
                this.ZY = double.NaN;
                this.Trans_x = double.NaN;
                this.Trans_y = double.NaN;
                this.Trans_z = double.NaN;
            }
        }

        public double[,] GetInertia() => _InertiaMatrix;

        public void SetInertia(double[,] inertia) => _InertiaMatrix = inertia;

        public double[,] GetInertiaCOM() => _InertiaMatrixCOM;

        public void SetInertiaCOM(double[,] inertiaCOM) => _InertiaMatrixCOM = inertiaCOM;

        public void AddUCS(string UCSName, Matrix UCSMatrix)
        {
            UCSData ucs = new UCSData(UCSName, UCSMatrix);
            _UCSPositions.Add(ucs);
        }

        public List<UCSData> GetUCSs()
        {
            return _UCSPositions;
        }
    
        public void SetSpecialTransformation(Matrix transformation)
        {
            _SpecialTransformation = transformation;
        }

        public Matrix GetSpecialTransformation()
        {
            return _SpecialTransformation;
        }

        public SpecialTransformation GetSpecialTransformationDTO()
        {
            return new SpecialTransformation(
                _SpecialTransformation.Cell[1, 1],
                _SpecialTransformation.Cell[2, 2],
                _SpecialTransformation.Cell[3, 3],
                _SpecialTransformation.Cell[1, 2],
                _SpecialTransformation.Cell[1, 3],
                _SpecialTransformation.Cell[2, 1],
                _SpecialTransformation.Cell[2, 3],
                _SpecialTransformation.Cell[3, 1],
                _SpecialTransformation.Cell[3, 2],
                _SpecialTransformation.Cell[1, 4] / 100.0,
                _SpecialTransformation.Cell[2, 4] / 100.0,
                _SpecialTransformation.Cell[3, 4] / 100.0
                );
        }
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
        private UCSData _OriginUCS;
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

        public void AddUCS(string label, string UCSName, Matrix UCS_Matrix)
        {
            if (!_dataMap.ContainsKey(label))
                _dataMap[label] = new MassCenterData();

            MassCenterData current = _dataMap[label];
            current.AddUCS(UCSName, UCS_Matrix);
            _dataMap[label] = current;
        }

        public List<UCSData> GetUCSs(string label)
        {
            MassCenterData current = _dataMap[label];
            return current.GetUCSs();
        }

        public void SetSpecialTransformation(string label, Matrix transformation)
        {
            if (!_dataMap.ContainsKey(label))
                _dataMap[label] = new MassCenterData();

            MassCenterData current = _dataMap[label];
            current.SetSpecialTransformation(transformation);
            _dataMap[label] = current;
        }

        public Matrix GetSpecialTransformation(string label)
        {
            MassCenterData current = _dataMap[label];
            return current.GetSpecialTransformation();
        }
        public MassCenterData.SpecialTransformation GetSpecialTransformationDTO(string label)
        {
            MassCenterData current = _dataMap[label];
            return current.GetSpecialTransformationDTO();
        }
    
        public UCSData GetOriginUCS()
        {
            return _OriginUCS;
        }

        private void SetOriginUCS(UCSData OriginUCS)
        {
            _OriginUCS = OriginUCS;
        }



    }
}
