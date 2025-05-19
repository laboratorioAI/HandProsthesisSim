using BufferPrint;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BufferManager
{
    public class ManagementConstants
    {
        public enum BufferManagerElement
        {
            // left thumb
            LThB__ = 0,
            LThPxJ = 1,
            LThMdJ = 2,
            LThDsJ = 3,
            // left index
            LInPxJ = 4,
            LInMdJ = 5,
            LInDsJ = 6,
            // left middle
            LMdPxJ = 7,
            LMdMdJ = 8,
            LMdDsJ = 9, 
            // left ring
            LRnPxJ = 10,
            LRnMdJ = 11,
            LRnDsJ = 12,
            // left pinky
            LPkPxJ = 13,
            LPkMdJ = 14,
            LPkDsJ = 15,

            // right thumb
            RThB__ = 16,
            RThPxJ = 17,
            RThMdJ = 18,
            RThDsJ = 19,
            // Rigth index
            RInPxJ = 20,
            RInMdJ = 21,
            RInDsJ = 22,
            // Rigth middRe
            RMdPxJ = 23,
            RMdMdJ = 24,
            RMdDsJ = 25,
            // Rigth ring
            RRnPxJ = 26,
            RRnMdJ = 27,
            RRnDsJ = 28,
            // Rigth pinky
            RPkPxJ = 29,
            RPkMdJ = 30,
            RPkDsJ = 31,
        }

        public enum BufferManagerAction
        {
            read__ = 0,
            writte = 1
        }

        public static class JointNameMapper
        {
            public static readonly Dictionary<BufferManagerElement, string> Map = new Dictionary<BufferManagerElement, string>
            {
                // LEFT THUMB
                { BufferManagerElement.LThB__, "LeftThumbBase" },
                { BufferManagerElement.LThPxJ, "LeftThumbProximal" },
                { BufferManagerElement.LThMdJ, "LeftThumbMiddle" },
                { BufferManagerElement.LThDsJ, "LeftThumbDistal" },

                // LEFT INDEX
                { BufferManagerElement.LInPxJ, "LeftIndexProximal" },
                { BufferManagerElement.LInMdJ, "LeftIndexMiddle" },
                { BufferManagerElement.LInDsJ, "LeftIndexDistal" },

                // LEFT MIDDLE
                { BufferManagerElement.LMdPxJ, "LeftMiddleProximal" },
                { BufferManagerElement.LMdMdJ, "LeftMiddleMiddle" },
                { BufferManagerElement.LMdDsJ, "LeftMiddleDistal" },

                // LEFT RING
                { BufferManagerElement.LRnPxJ, "LeftRingProximal" },
                { BufferManagerElement.LRnMdJ, "LeftRingMiddle" },
                { BufferManagerElement.LRnDsJ, "LeftRingDistal" },

                // LEFT PINKY
                { BufferManagerElement.LPkPxJ, "LeftPinkyProximal" },
                { BufferManagerElement.LPkMdJ, "LeftPinkyMiddle" },
                { BufferManagerElement.LPkDsJ, "LeftPinkyDistal" },

                // RIGHT THUMB
                { BufferManagerElement.RThB__, "RightThumbBase" },
                { BufferManagerElement.RThPxJ, "RightThumbProximal" },
                { BufferManagerElement.RThMdJ, "RightThumbMiddle" },
                { BufferManagerElement.RThDsJ, "RightThumbDistal" },

                // RIGHT INDEX
                { BufferManagerElement.RInPxJ, "Rotational:9" },
                { BufferManagerElement.RInMdJ, "Rotational:7" },
                { BufferManagerElement.RInDsJ, "Rotational:8" },

                // RIGHT MIDDLE
                { BufferManagerElement.RMdPxJ, "Rotational:1" },
                { BufferManagerElement.RMdMdJ, "Rotational:2" },
                { BufferManagerElement.RMdDsJ, "Rotational:3" },

                // RIGHT RING
                { BufferManagerElement.RRnPxJ, "Rotational:6" },
                { BufferManagerElement.RRnMdJ, "Rotational:4" },
                { BufferManagerElement.RRnDsJ, "Rotational:5" },

                // RIGHT PINKY
                { BufferManagerElement.RPkPxJ, "Rotational:12" },
                { BufferManagerElement.RPkMdJ, "Rotational:10" },
                { BufferManagerElement.RPkDsJ, "Rotational:11" },
            };
        }

    }
}
