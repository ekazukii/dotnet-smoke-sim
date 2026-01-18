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
    private readonly byte[] _pixels;
    private readonly WriteableBitmap _bitmap;

    public SmokeRenderer(int width, int height)
    {
        _width = width;
        _height = height;
        _pixels = new byte[width * height * 4];
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
                    _pixels[pixelIndex++] = 18;
                    _pixels[pixelIndex++] = 16;
                    _pixels[pixelIndex++] = 14;
                    _pixels[pixelIndex++] = 255;
                    continue;
                }

                float raw = density[fieldIndex];
                if (!float.IsFinite(raw))
                {
                    raw = 0f;
                }

                float d = raw * 0.05f;
                if (d < 0f)
                {
                    d = 0f;
                }
                else if (d > 1f)
                {
                    d = 1f;
                }

                d = MathF.Sqrt(d);
                byte r = (byte)(60 + 175 * d);
                byte g = (byte)(70 + 175 * d);
                byte b = (byte)(85 + 175 * d);

                _pixels[pixelIndex++] = b;
                _pixels[pixelIndex++] = g;
                _pixels[pixelIndex++] = r;
                _pixels[pixelIndex++] = 255;
            }
        }

        unsafe
        {
            using var frameBuffer = _bitmap.Lock();
            int rowBytes = frameBuffer.RowBytes;
            int srcStride = _width * 4;

            fixed (byte* srcPtr = _pixels)
            {
                byte* destBase = (byte*)frameBuffer.Address;
                if (rowBytes == srcStride)
                {
                    Buffer.MemoryCopy(srcPtr, destBase, _pixels.Length, _pixels.Length);
                }
                else
                {
                    for (int y = 0; y < _height; y++)
                    {
                        byte* destRow = destBase + y * rowBytes;
                        byte* srcRow = srcPtr + y * srcStride;
                        Buffer.MemoryCopy(srcRow, destRow, rowBytes, srcStride);
                    }
                }
            }
        }

    }
}
