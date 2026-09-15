// Minimal UnityEngine surface for compile-checking DevicePipe + DeviceViz outside Unity.
namespace UnityEngine
{
    public class Object
    {
        public static void DontDestroyOnLoad(Object obj) { }
        public static void Destroy(Object obj) { }
        public static implicit operator bool(Object o) => !ReferenceEquals(o, null);
        public static bool operator ==(Object a, Object b) => ReferenceEquals(a, b);
        public static bool operator !=(Object a, Object b) => !ReferenceEquals(a, b);
        public override bool Equals(object obj) => ReferenceEquals(this, obj);
        public override int GetHashCode() => base.GetHashCode();
    }

    public static class Mathf
    {
        public const float PI = (float)System.Math.PI;
        public const float Deg2Rad = PI / 180f;
        public const float Rad2Deg = 180f / PI;

        public static float Sqrt(float f) => (float)System.Math.Sqrt(f);
        public static float Abs(float f) => System.Math.Abs(f);
        public static float Exp(float f) => (float)System.Math.Exp(f);
        public static float Cos(float f) => (float)System.Math.Cos(f);
        public static float Sin(float f) => (float)System.Math.Sin(f);
        public static float Atan2(float y, float x) => (float)System.Math.Atan2(y, x);
        public static float Max(float a, float b) => System.Math.Max(a, b);
        public static float Min(float a, float b) => System.Math.Min(a, b);
        public static int Max(int a, int b) => System.Math.Max(a, b);
        public static int Min(int a, int b) => System.Math.Min(a, b);
        public static float Clamp(float value, float min, float max) => value < min ? min : value > max ? max : value;
        public static int CeilToInt(float f) => (int)System.Math.Ceiling(f);
        public static int RoundToInt(float f) => (int)System.Math.Round(f);
    }

    public static class Debug
    {
        public static void Log(object message) => System.Console.WriteLine(message);
        public static void LogWarning(object message) => System.Console.WriteLine("WARN: " + message);
        public static void LogError(object message) => System.Console.WriteLine("ERROR: " + message);
    }

    public static class Application
    {
        public static bool isPlaying => true;
    }
}
