// Minimal DeviceLink surface used by DevicePipe, for compile-check only.
namespace DeviceLink
{
    public class SerialBridge
    {
        public event System.Action<string> OnError;
        public event System.Action<byte[], int, int> OnDataReceived;
        public bool IsOpen => false;
        public static string[] GetPortNames() => new string[0];
        public void Open(string portName, int baudRate) { }
        public void Close() { }
    }
}
