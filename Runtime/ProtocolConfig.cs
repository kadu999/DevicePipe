namespace DevicePipe
{
    public enum ChecksumType
    {
        Sum16,
        CRC16_Modbus,
        CRC32_Mpeg2
    }

    /// <summary>Protocol frame configuration.</summary>
    public class ProtocolConfig
    {
        /// <summary>Frame header as hex, e.g. "A55A01"</summary>
        public string HeaderHex = "A55A01";

        /// <summary>Bits per data sample: 8 or 16</summary>
        public int BitsPerSample = 8;

        /// <summary>Header byte length (usually 3 for A55A01)</summary>
        public int HeaderByteLength = 3;

        /// <summary>Overhead bytes included in the frame length field value. Default 6 = header(3) + length(2) + 1.</summary>
        public int HeadLen = 6;

        /// <summary>Set true to skip checksum validation (for devices that don't implement it).</summary>
        public bool SkipChecksum = false;

        /// <summary>Checksum method</summary>
        public ChecksumType Checksum = ChecksumType.Sum16;

        /// <summary>Data grid row count</summary>
        public int RowCount = 1;

        /// <summary>Data grid column count</summary>
        public int ColCount = 1;

        public int DataByteCount => RowCount * ColCount * (BitsPerSample / 8);
        public byte[] HeaderBytes => HexToBytes(HeaderHex);

        static byte[] HexToBytes(string hex)
        {
            hex = hex.Replace(" ", "");
            if (hex.Length % 2 != 0) hex = "0" + hex;
            var bytes = new byte[hex.Length / 2];
            for (int i = 0; i < bytes.Length; i++)
                bytes[i] = System.Convert.ToByte(hex.Substring(i * 2, 2), 16);
            return bytes;
        }
    }
}
