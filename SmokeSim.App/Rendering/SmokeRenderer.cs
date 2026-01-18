using System;
using Avalonia;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using SmokeSim.Core;

namespace SmokeSim.App.Rendering;

public sealed class SmokeRenderer
{
    private readonly int _width;
    private readonly int _height;
    private readonly uint[] _pixels;
    private readonly uint[] _densityLut;
    private readonly float _densityToLut;
    private readonly int _lutMaxIndex;
    private readonly uint _solidColor;
    private readonly WriteableBitmap _bitmap;

    public SmokeRenderer(int width, int height)
    {
        _width = width;
        _height = height;
        _pixels = new uint[width * height];
        _densityLut = BuildDensityLut(1024, 220f);
        _densityToLut = (_densityLut.Length - 1) / 220f;
        _lutMaxIndex = _densityLut.Length - 1;
        _solidColor = Pack(14, 16, 18, 255);
        _bitmap = new WriteableBitmap(
            new PixelSize(width, height),
            new Vector(96, 96),
            PixelFormat.Bgra8888,
            AlphaFormat.Premul);
    }

    public WriteableBitmap Bitmap => _bitmap;

    public void Update(FluidSolver2D solver)
    {
        var density = solver.Fields.Density;
        var solid = solver.Fields.Obstacles.Data;

        int stride = solver.Stride;
        int pixelIndex = 0;

        for (int y = 1; y <= _height; y++)
        {
            int fieldRow = y * stride;
            for (int x = 1; x <= _width; x++)
            {
                int fieldIndex = fieldRow + x;

                if (solid[fieldIndex])
                {
                    _pixels[pixelIndex++] = _solidColor;
                    continue;
                }

                float raw = density[fieldIndex];
                if (!float.IsFinite(raw) || raw <= 0f)
                {
                    _pixels[pixelIndex++] = 0u;
                    continue;
                }

                int lutIndex = (int)(raw * _densityToLut);
                if (lutIndex > _lutMaxIndex)
                {
                    lutIndex = _lutMaxIndex;
                }

                _pixels[pixelIndex++] = _densityLut[lutIndex];
            }
        }

        unsafe
        {
            using var frameBuffer = _bitmap.Lock();
            int rowBytes = frameBuffer.RowBytes;
            int srcStride = _width * 4;
            int byteCount = _pixels.Length * sizeof(uint);

            fixed (uint* srcPtr = _pixels)
            {
                byte* destBase = (byte*)frameBuffer.Address;
                byte* srcBytes = (byte*)srcPtr;
                if (rowBytes == srcStride)
                {
                    Buffer.MemoryCopy(srcBytes, destBase, byteCount, byteCount);
                }
                else
                {
                    for (int y = 0; y < _height; y++)
                    {
                        byte* destRow = destBase + y * rowBytes;
                        byte* srcRow = srcBytes + y * srcStride;
                        Buffer.MemoryCopy(srcRow, destRow, rowBytes, srcStride);
                    }
                }
            }
        }
    }

    private static uint[] BuildDensityLut(int size, float maxDensity)
    {
        var lut = new uint[size];
        float inv = 1f / (size - 1);

        const float lowR = 205f;
        const float lowG = 150f;
        const float lowB = 120f;
        const float highR = 220f;
        const float highG = 60f;
        const float highB = 40f;

        for (int i = 0; i < size; i++)
        {
            float raw = maxDensity * (i * inv);
            float t = 1f - MathF.Exp(-raw * 0.0045f);
            if (t > 1f)
            {
                t = 1f;
            }

            float colorT = MathF.Pow(t, 0.85f);
            float alphaT = MathF.Sqrt(t);

            float rF = lowR + (highR - lowR) * colorT;
            float gF = lowG + (highG - lowG) * colorT;
            float bF = lowB + (highB - lowB) * colorT;

            float alphaF = alphaT * 200f;
            float premul = alphaF / 255f;

            byte r = (byte)Math.Clamp(rF * premul, 0f, 255f);
            byte g = (byte)Math.Clamp(gF * premul, 0f, 255f);
            byte b = (byte)Math.Clamp(bF * premul, 0f, 255f);
            byte alpha = (byte)Math.Clamp(alphaF, 0f, 255f);

            lut[i] = Pack(r, g, b, alpha);
        }

        return lut;
    }

    private static uint Pack(byte r, byte g, byte b, byte a)
        => (uint)b | ((uint)g << 8) | ((uint)r << 16) | ((uint)a << 24);
}
