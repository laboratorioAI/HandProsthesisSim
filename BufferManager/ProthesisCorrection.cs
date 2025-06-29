using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BufferManager
{
    public class ProthesisCorrection
    {
        public static float Correct_RInPxJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 180.0f - value; // Example correction
        }

        public static float Correct_RInMdJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 50.0f - value; // Example correction
        }

        public static float Correct_RInDsJ(float value)
        {
            double delta = value * 70.0 / 90.0;
            // Correct the value for the right index proximal joint
            return (float)(200.0f + delta); // Example correction
        }

        public static float Correct_RMdPxJ(float value)
        {
            // Correct the value for the right index proximal joint
            return -90.0f - value; // Example correction
        }

        public static float Correct_RMdMdJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 50.0f - value; // Example correction
        }

        public static float Correct_RMdDsJ(float value)
        {
            // Correct the value for the right index proximal joint
            return value + 200.0f; // Example correction
        }

        public static float Correct_RRnPxJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 180.0f - value; // Example correction
        }

        public static float Correct_RRnMdJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 45.0f - value; // Example correction
        }

        public static float Correct_RRnDsJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 200.0f + value; // Example correction
        }

        public static float Correct_RPkPxJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 0.0f - value; // Example correction
        }

        public static float Correct_RPkMdJ(float value)
        {
            // Correct the value for the right index proximal joint
            return 50.0f - value; // Example correction
        }

        public static float Correct_RPkDsJ(float value)
        {
            // Correct the value for the right index proximal joint
            return value - 160.0f; // Example correction
        }

        public static float Correct_RThPxJ(float value)
        {
            // Correct the value for the right index proximal joint

            return 0.0f + value; // Example correction
        }

        public static float Correct_RThMdJ(float value)
        {
            double delta = value * 75.0 / 90.0;
            // Correct the value for the right index proximal joint
            return (float)(120.0f + delta); // Example correction
        }

        public static float Correct_RThDsJ(float value)
        {
            double delta = value * 70.0 / 90.0;
            // Correct the value for the right index proximal joint
            return (float)(120.0f + delta); // Example correction
        }

    }
}
