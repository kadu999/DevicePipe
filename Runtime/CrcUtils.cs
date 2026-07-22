using System;

namespace DevicePipe
{
    internal static class CrcUtils
    {
        /// <summary>16-bit sum (accumulates in Int16, matching reference impl).</summary>
        public static bool ValidateSum16(byte[] data, int start, int len)
        {
            short temp = 0;
            int end = start + len - 2;
            for (int i = start; i < end; i++)
                temp += (short)data[i];

            byte lo = (byte)(temp & 0xFF);
            byte hi = (byte)(temp >> 8 & 0xFF);
            return data[end] == lo && data[end + 1] == hi;
        }

        /// <summary>CRC-16 Modbus check.</summary>
        public static bool ValidateCrc16Modbus(byte[] data, int start, int len)
        {
            ushort crc = 0xFFFF;
            int end = start + len - 2; // last 2 bytes are CRC

            for (int i = start; i < end; i++)
            {
                crc ^= data[i];
                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 1) != 0)
                        crc = (ushort)((crc >> 1) ^ 0xA001);
                    else
                        crc >>= 1;
                }
            }

            byte lo = (byte)(crc & 0xFF);
            byte hi = (byte)(crc >> 8);
            return data[end] == lo && data[end + 1] == hi;
        }

        /// <summary>CRC-32 MPEG-2 check.</summary>
        public static bool ValidateCrc32Mpeg2(byte[] data, int start, int len)
        {
            uint crc = 0xFFFFFFFF;
            int end = start + len - 4; // last 4 bytes are CRC

            for (int i = start; i < end; i++)
            {
                crc ^= (uint)data[i] << 24;
                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 0x80000000) != 0)
                        crc = (crc << 1) ^ 0x04C11DB7;
                    else
                        crc <<= 1;
                }
            }

            byte b0 = (byte)(crc >> 24);
            byte b1 = (byte)((crc >> 16) & 0xFF);
            byte b2 = (byte)((crc >> 8) & 0xFF);
            byte b3 = (byte)(crc & 0xFF);
            return data[end] == b0 && data[end + 1] == b1
                && data[end + 2] == b2 && data[end + 3] == b3;
        }
    }
}
