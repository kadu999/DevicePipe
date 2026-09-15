// Minimal UnityEngine/UI surface for compile-checking DeviceViz outside Unity.
using System;

namespace UnityEngine
{
    public struct Vector2
    {
        public float x, y;
        public Vector2(float x, float y) { this.x = x; this.y = y; }
        public static Vector2 zero => new Vector2(0f, 0f);
        public static Vector2 one => new Vector2(1f, 1f);
        public static Vector2 operator +(Vector2 a, Vector2 b) => new Vector2(a.x + b.x, a.y + b.y);
        public static Vector2 operator -(Vector2 a, Vector2 b) => new Vector2(a.x - b.x, a.y - b.y);
        public static Vector2 operator *(Vector2 a, float d) => new Vector2(a.x * d, a.y * d);
        public static Vector2 operator *(float d, Vector2 a) => a * d;
    }

    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static implicit operator Vector3(Vector2 v) => new Vector3(v.x, v.y, 0f);
        public static explicit operator Vector2(Vector3 v) => new Vector2(v.x, v.y);
    }

    public struct Vector4
    {
        public float x, y, z, w;
        public Vector4(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        public static implicit operator Vector4(Vector2 v) => new Vector4(v.x, v.y, 0f, 0f);
        public static implicit operator Vector4(Vector3 v) => new Vector4(v.x, v.y, v.z, 0f);
        public static implicit operator Vector4(Color c) => new Vector4(c.r, c.g, c.b, c.a);
    }

    public struct Color
    {
        public float r, g, b, a;
        public Color(float r, float g, float b) { this.r = r; this.g = g; this.b = b; a = 1f; }
        public Color(float r, float g, float b, float a) { this.r = r; this.g = g; this.b = b; this.a = a; }
        public static Color white => new Color(1f, 1f, 1f, 1f);
        public static Color clear => new Color(0f, 0f, 0f, 0f);
        public static Color black => new Color(0f, 0f, 0f, 1f);
        public static Color HSVToRGB(float h, float s, float v) => new Color(v, s, h);
    }

    public struct Quaternion
    {
        public static Quaternion Euler(float x, float y, float z) => new Quaternion();
        public static Quaternion identity => new Quaternion();
    }

    public struct Rect
    {
        public float x, y, width, height;
        public Rect(float x, float y, float width, float height)
        { this.x = x; this.y = y; this.width = width; this.height = height; }
        public Vector2 center => new Vector2(x + width * 0.5f, y + height * 0.5f);
        public Vector2 size => new Vector2(width, height);
    }

    public class Camera : Object { }

    public class Component : Object
    {
        public GameObject gameObject => new GameObject();
        public Transform transform => new Transform();
        public T GetComponent<T>() => default;
    }

    public class Behaviour : Component
    {
        public bool enabled { get; set; }
    }

    public class Transform : Component
    {
        public void SetParent(Transform parent, bool worldPositionStays) { }
        public void SetAsLastSibling() { }
    }

    public class RectTransform : Transform
    {
        public Rect rect => new Rect(0f, 0f, 100f, 100f);
        public Vector2 anchoredPosition { get; set; }
        public Vector2 sizeDelta { get; set; }
        public Quaternion localRotation { get; set; }
        public Vector2 anchorMin { get; set; }
        public Vector2 anchorMax { get; set; }
        public Vector2 pivot { get; set; }
        public Vector2 offsetMin { get; set; }
        public Vector2 offsetMax { get; set; }
    }

    public static class RectTransformUtility
    {
        public static bool ScreenPointToLocalPointInRectangle(
            RectTransform rect, Vector2 screenPoint, Camera cam, out Vector2 localPoint)
        { localPoint = Vector2.zero; return true; }
    }

    public class GameObject : Object
    {
        public GameObject() { }
        public GameObject(string name, params Type[] components) { }
        public Transform transform => new Transform();
        public bool activeInHierarchy => true;
        public void SetActive(bool value) { }
        public T GetComponent<T>() => default;
        public T AddComponent<T>() where T : Component, new() => new T();
    }

    public static class ObjectOps { } // placeholder to keep namespace tidy

    public class Texture : Object
    {
        public FilterMode filterMode { get; set; }
        public TextureWrapMode wrapMode { get; set; }
    }

    public class Texture2D : Texture
    {
        public Texture2D(int width, int height, TextureFormat format, bool mipChain) { }
        public void SetPixels(Color[] colors) { }
        public void Apply() { }
    }

    public class RenderTexture : Texture
    {
        public RenderTexture(int width, int height, int depth, RenderTextureFormat format) { }
        public bool enableRandomWrite { get; set; }
        public static RenderTexture active { get; set; }
        public void Create() { }
        public void Release() { }
    }

    public class Sprite : Object
    {
        public static Sprite Create(Texture2D texture, Rect rect, Vector2 pivot) => new Sprite();
    }

    public class Font : Object { }

    public class Shader : Object
    {
        public static Shader Find(string name) => new Shader();
    }

    public class Material : Object
    {
        public Material(Shader shader) { }
        public void SetColor(string name, Color value) { }
        public void SetFloat(string name, float value) { }
        public void SetVector(string name, Vector4 value) { }
        public void SetBuffer(string name, ComputeBuffer buffer) { }
        public bool SetPass(int pass) => true;
    }

    public class ComputeBuffer : Object
    {
        public ComputeBuffer(int count, int stride) { }
        public void SetData<T>(T[] data) { }
        public void SetData<T>(T[] data, int managedBufferStartIndex, int computeBufferStartIndex, int count) { }
        public void Release() { }
    }

    public class ComputeShader : Object
    {
        public int FindKernel(string name) => 0;
        public void SetBuffer(int kernel, string name, ComputeBuffer buffer) { }
        public void SetTexture(int kernel, string name, Texture texture) { }
        public void SetInt(string name, int value) { }
        public void SetFloat(string name, float value) { }
        public void SetVector(string name, Vector4 value) { }
        public void Dispatch(int kernel, int x, int y, int z) { }
    }

    public static class Resources
    {
        public static T Load<T>(string path) where T : Object => null;
        public static T GetBuiltinResource<T>(string path) where T : Object => null;
    }

    public static class GL
    {
        public static void Clear(bool clearDepth, bool clearColor, Color backgroundColor) { }
    }

    public static class Graphics
    {
        public static void DrawProceduralNow(MeshTopology topology, int vertexCount) { }
    }

    public static class Time
    {
        public static float deltaTime => 0f;
    }

    public class MonoBehaviour : Behaviour
    {
        protected virtual void Update() { }
    }

    public enum TextureFormat { RGBA32 }
    public enum TextureWrapMode { Clamp, Repeat }
    public enum FilterMode { Point, Bilinear }
    public enum RenderTextureFormat { ARGB32 }
    public enum MeshTopology { Triangles, Quads }
    public enum TextAnchor { UpperLeft, UpperCenter, UpperRight, MiddleLeft, MiddleCenter, MiddleRight, LowerLeft, LowerCenter, LowerRight }

    public class PropertyAttribute : Attribute { }
    public class HeaderAttribute : PropertyAttribute
    {
        public HeaderAttribute(string header) { }
    }
    public class RangeAttribute : PropertyAttribute
    {
        public RangeAttribute(float min, float max) { }
    }
    public class SerializeField : Attribute { }

    public class RectOffset
    {
        public RectOffset(int left, int right, int top, int bottom) { }
    }
}

namespace UnityEngine.UI
{
    using UnityEngine;

    public class UIBehaviour : MonoBehaviour { }

    public class Graphic : UIBehaviour
    {
        public Color color { get; set; }
        public bool raycastTarget { get; set; }
        public RectTransform rectTransform => new RectTransform();
        public void SetVerticesDirty() { }
        protected virtual void OnPopulateMesh(VertexHelper vh) { }
    }

    public class MaskableGraphic : Graphic { }

    public class Image : MaskableGraphic
    {
        public Sprite sprite { get; set; }
    }

    public class RawImage : Graphic
    {
        public Texture texture { get; set; }
    }

    public class Text : MaskableGraphic
    {
        public string text { get; set; }
        public Font font { get; set; }
        public int fontSize { get; set; }
        public TextAnchor alignment { get; set; }
    }

    public class Selectable : UIBehaviour { }

    public class Toggle : Selectable
    {
        public bool isOn { get; set; }
        public UnityEngine.Events.UnityEvent<bool> onValueChanged { get; } =
            new UnityEngine.Events.UnityEvent<bool>();
    }

    public class LayoutGroup : UIBehaviour
    {
        public RectOffset padding { get; set; }
        public float spacing { get; set; }
        public TextAnchor childAlignment { get; set; }
        public bool childControlWidth { get; set; }
        public bool childControlHeight { get; set; }
    }

    public class VerticalLayoutGroup : LayoutGroup { }

    public struct UIVertex
    {
        public Vector3 position;
        public Color color;
        public Vector2 uv0;
    }

    public class VertexHelper
    {
        public int currentVertCount => 0;
        public void Clear() { }
        public void AddVert(UIVertex vertex) { }
        public void AddTriangle(int idx0, int idx1, int idx2) { }
    }

    public class CanvasRenderer : Component { }
}

namespace UnityEngine.Events
{
    public delegate void UnityAction<T0>(T0 arg0);

    public class UnityEvent<T0>
    {
        public void AddListener(UnityAction<T0> call) { }
        public void RemoveListener(UnityAction<T0> call) { }
    }
}

namespace UnityEngine.EventSystems
{
    using UnityEngine;
    using UnityEngine.Events;

    public class BaseEventData
    {
        public Vector2 position { get; set; }
    }

    public class PointerEventData : BaseEventData
    {
        public GameObject pointerDrag { get; set; }
    }

    public enum EventTriggerType { PointerClick, Drag, BeginDrag }

    public class EventTrigger : MonoBehaviour
    {
        public class TriggerEvent : UnityEvent<BaseEventData> { }

        public class Entry
        {
            public EventTriggerType eventID;
            public TriggerEvent callback { get; } = new TriggerEvent();
        }

        public System.Collections.Generic.List<Entry> triggers { get; } =
            new System.Collections.Generic.List<Entry>();
    }
}

namespace UnityEngine.InputSystem
{
    using UnityEngine;

    public class Vector2Control
    {
        public Vector2 ReadValue() => Vector2.zero;
    }

    public class Mouse
    {
        public static Mouse current { get; } = new Mouse();
        public Vector2Control position { get; } = new Vector2Control();
        public Vector2Control scroll { get; } = new Vector2Control();
    }
}
