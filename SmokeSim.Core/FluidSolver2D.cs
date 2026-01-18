using System;

namespace SmokeSim.Core;

public sealed class FluidSolver2D
{
    private FluidFields _fields;
    private readonly SimulationParameters _parameters;

    public FluidSolver2D(int width, int height, SimulationParameters? parameters = null)
    {
        _fields = new FluidFields(width, height);
        _parameters = parameters ?? new SimulationParameters();
        ApplyBoundaryMode(_parameters.BoundaryMode);
    }

    public FluidFields Fields => _fields;
    public SimulationParameters Parameters => _parameters;
    public int Width => _fields.Width;
    public int Height => _fields.Height;
    public int Stride => _fields.Stride;

    public void Clear()
    {
        Array.Clear(_fields.Density, 0, _fields.Density.Length);
        Array.Clear(_fields.DensityPrev, 0, _fields.DensityPrev.Length);
        Array.Clear(_fields.VelocityU, 0, _fields.VelocityU.Length);
        Array.Clear(_fields.VelocityV, 0, _fields.VelocityV.Length);
        Array.Clear(_fields.VelocityUPrev, 0, _fields.VelocityUPrev.Length);
        Array.Clear(_fields.VelocityVPrev, 0, _fields.VelocityVPrev.Length);
        Array.Clear(_fields.Pressure, 0, _fields.Pressure.Length);
        Array.Clear(_fields.Divergence, 0, _fields.Divergence.Length);
        Array.Clear(_fields.Curl, 0, _fields.Curl.Length);
        Array.Clear(_fields.Scratch, 0, _fields.Scratch.Length);
    }

    public void ResetObstacles()
    {
        _fields.Obstacles.ClearInterior();
    }

    public void ApplyBoundaryMode(BoundaryMode mode)
    {
        _parameters.BoundaryMode = mode;
        bool borderSolid = mode == BoundaryMode.Bounce;
        _fields.Obstacles.SetBorderSolid(borderSolid);
    }

    public void Step(float dt)
    {
        if (dt <= 0f)
        {
            return;
        }

        AddSources(_fields.VelocityU, _fields.VelocityUPrev, dt);
        AddSources(_fields.VelocityV, _fields.VelocityVPrev, dt);

        ApplyBuoyancy(_fields.VelocityV, _fields.Density, dt);

        if (_parameters.Viscosity > 0f)
        {
            Swap(ref _fields.VelocityUPrev, ref _fields.VelocityU);
            Diffuse(1, _fields.VelocityU, _fields.VelocityUPrev, _parameters.Viscosity, dt);

            Swap(ref _fields.VelocityVPrev, ref _fields.VelocityV);
            Diffuse(2, _fields.VelocityV, _fields.VelocityVPrev, _parameters.Viscosity, dt);
        }

        Project(_fields.VelocityU, _fields.VelocityV, _fields.Pressure, _fields.Divergence);

        Swap(ref _fields.VelocityUPrev, ref _fields.VelocityU);
        Swap(ref _fields.VelocityVPrev, ref _fields.VelocityV);
        Advect(1, _fields.VelocityU, _fields.VelocityUPrev, _fields.VelocityUPrev, _fields.VelocityVPrev, dt);
        Advect(2, _fields.VelocityV, _fields.VelocityVPrev, _fields.VelocityUPrev, _fields.VelocityVPrev, dt);

        Project(_fields.VelocityU, _fields.VelocityV, _fields.Pressure, _fields.Divergence);

        if (_parameters.Vorticity > 0f)
        {
            ApplyVorticityConfinement(_fields.VelocityU, _fields.VelocityV, dt, _parameters.Vorticity);
        }

        if (_parameters.Diffusion > 0f)
        {
            Swap(ref _fields.DensityPrev, ref _fields.Density);
            Diffuse(0, _fields.Density, _fields.DensityPrev, _parameters.Diffusion, dt);
        }

        Swap(ref _fields.DensityPrev, ref _fields.Density);
        Advect(0, _fields.Density, _fields.DensityPrev, _fields.VelocityU, _fields.VelocityV, dt);

        if (_parameters.BoundaryMode == BoundaryMode.Open)
        {
            ClearInteriorEdges(_fields.Density);
        }

        ApplyDissipation(_fields.Density, dt);
    }

    public void AddDensityCircle(int centerX, int centerY, float amount, int radius)
    {
        if (amount <= 0f)
        {
            return;
        }

        ApplyCircle(centerX, centerY, radius, (x, y, weight) =>
        {
            int idx = Idx(x, y);
            if (_fields.Obstacles.Data[idx])
            {
                return;
            }

            _fields.Density[idx] += amount * weight;
        });
    }

    public void AddVelocityCircle(int centerX, int centerY, float forceX, float forceY, int radius)
    {
        if (forceX == 0f && forceY == 0f)
        {
            return;
        }

        ApplyCircle(centerX, centerY, radius, (x, y, weight) =>
        {
            int idx = Idx(x, y);
            if (_fields.Obstacles.Data[idx])
            {
                return;
            }

            _fields.VelocityU[idx] += forceX * weight;
            _fields.VelocityV[idx] += forceY * weight;
        });
    }

    public void SetObstacleCircle(int centerX, int centerY, int radius, bool solid)
    {
        ApplyCircle(centerX, centerY, radius, (x, y, _) =>
        {
            _fields.Obstacles.SetSolid(x, y, solid);
            int idx = Idx(x, y);
            _fields.VelocityU[idx] = 0f;
            _fields.VelocityV[idx] = 0f;
            _fields.Density[idx] = 0f;
        });
    }

    public int Idx(int x, int y) => x + Stride * y;

    private void ApplyCircle(int centerX, int centerY, int radius, Action<int, int, float> apply)
    {
        if (radius < 1)
        {
            radius = 1;
        }

        int r2 = radius * radius;
        int startX = Math.Max(1, centerX - radius);
        int endX = Math.Min(Width, centerX + radius);
        int startY = Math.Max(1, centerY - radius);
        int endY = Math.Min(Height, centerY + radius);

        for (int y = startY; y <= endY; y++)
        {
            int dy = y - centerY;
            for (int x = startX; x <= endX; x++)
            {
                int dx = x - centerX;
                int dist2 = dx * dx + dy * dy;
                if (dist2 > r2)
                {
                    continue;
                }

                float weight = 1f - MathF.Sqrt(dist2) / radius;
                apply(x, y, weight);
            }
        }
    }

    private void AddSources(float[] x, float[] s, float dt)
    {
        for (int i = 0; i < x.Length; i++)
        {
            if (s[i] != 0f)
            {
                x[i] += s[i] * dt;
                s[i] = 0f;
            }
        }
    }

    private void ApplyBuoyancy(float[] v, float[] density, float dt)
    {
        if (_parameters.Buoyancy <= 0f)
        {
            return;
        }

        float strength = _parameters.Buoyancy * dt;
        var solid = _fields.Obstacles.Data;

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                if (solid[idx])
                {
                    continue;
                }

                v[idx] -= strength * density[idx];
            }
        }
    }

    private void ApplyDissipation(float[] density, float dt)
    {
        if (_parameters.Dissipation <= 0f)
        {
            return;
        }

        float decay = MathF.Max(0f, 1f - _parameters.Dissipation * dt);
        var solid = _fields.Obstacles.Data;

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                if (solid[idx])
                {
                    density[idx] = 0f;
                }
                else
                {
                    density[idx] *= decay;
                }
            }
        }
    }

    private void Diffuse(int b, float[] x, float[] x0, float diff, float dt)
    {
        float a = dt * diff * Width * Height;
        int iterations = _parameters.SolverIterations;
        var solid = _fields.Obstacles.Data;

        for (int k = 0; k < iterations; k++)
        {
            for (int y = 1; y <= Height; y++)
            {
                int row = y * Stride;
                for (int xPos = 1; xPos <= Width; xPos++)
                {
                    int idx = row + xPos;
                    if (solid[idx])
                    {
                        x[idx] = 0f;
                        continue;
                    }

                    float sum = 0f;
                    int fluidCount = 0;

                    int idxL = idx - 1;
                    if (!solid[idxL])
                    {
                        sum += x[idxL];
                        fluidCount++;
                    }

                    int idxR = idx + 1;
                    if (!solid[idxR])
                    {
                        sum += x[idxR];
                        fluidCount++;
                    }

                    int idxD = idx - Stride;
                    if (!solid[idxD])
                    {
                        sum += x[idxD];
                        fluidCount++;
                    }

                    int idxU = idx + Stride;
                    if (!solid[idxU])
                    {
                        sum += x[idxU];
                        fluidCount++;
                    }

                    float denom = 1f + a * fluidCount;
                    x[idx] = (x0[idx] + a * sum) / denom;
                }
            }

            SetBounds(b, x);
        }
    }

    private void Advect(int b, float[] d, float[] d0, float[] u, float[] v, float dt)
    {
        float dt0x = dt * Width;
        float dt0y = dt * Height;
        var solid = _fields.Obstacles.Data;
        bool bounce = _parameters.BoundaryMode == BoundaryMode.Bounce;
        bool wrap = _parameters.BoundaryMode == BoundaryMode.Wrap;

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int xPos = 1; xPos <= Width; xPos++)
            {
                int idx = row + xPos;
                if (solid[idx])
                {
                    d[idx] = 0f;
                    continue;
                }

                float x = xPos - dt0x * u[idx];
                float yPos = y - dt0y * v[idx];

                if (!bounce && !wrap)
                {
                    if (x < 0.5f || x > Width + 0.5f || yPos < 0.5f || yPos > Height + 0.5f)
                    {
                        d[idx] = 0f;
                        continue;
                    }
                }

                if (wrap)
                {
                    x = WrapCoord(x, Width);
                    yPos = WrapCoord(yPos, Height);
                }
                else
                {
                    x = Math.Clamp(x, 0.5f, Width + 0.5f);
                    yPos = Math.Clamp(yPos, 0.5f, Height + 0.5f);
                }

                int i0 = (int)MathF.Floor(x);
                int i1 = i0 + 1;
                int j0 = (int)MathF.Floor(yPos);
                int j1 = j0 + 1;

                float s1 = x - i0;
                float s0 = 1f - s1;
                float t1 = yPos - j0;
                float t0 = 1f - t1;

                int idx00 = Idx(i0, j0);
                int idx10 = Idx(i1, j0);
                int idx01 = Idx(i0, j1);
                int idx11 = Idx(i1, j1);

                float d00 = solid[idx00] ? 0f : d0[idx00];
                float d10 = solid[idx10] ? 0f : d0[idx10];
                float d01 = solid[idx01] ? 0f : d0[idx01];
                float d11 = solid[idx11] ? 0f : d0[idx11];

                d[idx] = s0 * (t0 * d00 + t1 * d01) + s1 * (t0 * d10 + t1 * d11);
            }
        }

        SetBounds(b, d);
    }

    private void Project(float[] u, float[] v, float[] p, float[] div)
    {
        float invW = 1f / Width;
        float invH = 1f / Height;
        var solid = _fields.Obstacles.Data;

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                if (solid[idx])
                {
                    div[idx] = 0f;
                    p[idx] = 0f;
                    continue;
                }

                float uR = solid[idx + 1] ? 0f : u[idx + 1];
                float uL = solid[idx - 1] ? 0f : u[idx - 1];
                float vU = solid[idx + Stride] ? 0f : v[idx + Stride];
                float vD = solid[idx - Stride] ? 0f : v[idx - Stride];

                div[idx] = -0.5f * ((uR - uL) * invW + (vU - vD) * invH);
                p[idx] = 0f;
            }
        }

        SetBounds(0, div);
        SetBounds(0, p);

        int iterations = _parameters.SolverIterations;
        for (int k = 0; k < iterations; k++)
        {
            for (int y = 1; y <= Height; y++)
            {
                int row = y * Stride;
                for (int x = 1; x <= Width; x++)
                {
                    int idx = row + x;
                    if (solid[idx])
                    {
                        p[idx] = 0f;
                        continue;
                    }

                    float pL = solid[idx - 1] ? p[idx] : p[idx - 1];
                    float pR = solid[idx + 1] ? p[idx] : p[idx + 1];
                    float pD = solid[idx - Stride] ? p[idx] : p[idx - Stride];
                    float pU = solid[idx + Stride] ? p[idx] : p[idx + Stride];

                    int fluidCount = 0;
                    fluidCount += solid[idx - 1] ? 0 : 1;
                    fluidCount += solid[idx + 1] ? 0 : 1;
                    fluidCount += solid[idx - Stride] ? 0 : 1;
                    fluidCount += solid[idx + Stride] ? 0 : 1;
                    if (fluidCount == 0)
                    {
                        p[idx] = 0f;
                    }
                    else
                    {
                        p[idx] = (div[idx] + pL + pR + pD + pU) / fluidCount;
                    }
                }
            }

            SetBounds(0, p);
        }

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                if (solid[idx])
                {
                    u[idx] = 0f;
                    v[idx] = 0f;
                    continue;
                }

                float pL = solid[idx - 1] ? p[idx] : p[idx - 1];
                float pR = solid[idx + 1] ? p[idx] : p[idx + 1];
                float pD = solid[idx - Stride] ? p[idx] : p[idx - Stride];
                float pU = solid[idx + Stride] ? p[idx] : p[idx + Stride];

                u[idx] -= 0.5f * (pR - pL) * Width;
                v[idx] -= 0.5f * (pU - pD) * Height;
            }
        }

        SetBounds(1, u);
        SetBounds(2, v);
    }

    private void ApplyVorticityConfinement(float[] u, float[] v, float dt, float strength)
    {
        var solid = _fields.Obstacles.Data;
        float invW = 1f / Width;
        float invH = 1f / Height;

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                if (solid[idx])
                {
                    _fields.Curl[idx] = 0f;
                    continue;
                }

                float uU = solid[idx + Stride] ? 0f : u[idx + Stride];
                float uD = solid[idx - Stride] ? 0f : u[idx - Stride];
                float vR = solid[idx + 1] ? 0f : v[idx + 1];
                float vL = solid[idx - 1] ? 0f : v[idx - 1];

                float duDy = (uU - uD) * 0.5f * invH;
                float dvDx = (vR - vL) * 0.5f * invW;
                _fields.Curl[idx] = dvDx - duDy;
            }
        }

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                if (solid[idx])
                {
                    continue;
                }

                float curlL = MathF.Abs(_fields.Curl[idx - 1]);
                float curlR = MathF.Abs(_fields.Curl[idx + 1]);
                float curlD = MathF.Abs(_fields.Curl[idx - Stride]);
                float curlU = MathF.Abs(_fields.Curl[idx + Stride]);

                float dx = (curlR - curlL) * 0.5f * invW;
                float dy = (curlU - curlD) * 0.5f * invH;

                float len = MathF.Sqrt(dx * dx + dy * dy) + 1e-5f;
                dx /= len;
                dy /= len;

                float force = strength * _fields.Curl[idx];
                u[idx] += dt * dy * force;
                v[idx] -= dt * dx * force;
            }
        }

        SetBounds(1, u);
        SetBounds(2, v);
    }

    private void SetBounds(int b, float[] x)
    {
        if (_parameters.BoundaryMode == BoundaryMode.Open)
        {
            for (int i = 1; i <= Width; i++)
            {
                x[Idx(i, 0)] = 0f;
                x[Idx(i, Height + 1)] = 0f;
            }

            for (int j = 1; j <= Height; j++)
            {
                x[Idx(0, j)] = 0f;
                x[Idx(Width + 1, j)] = 0f;
            }

            x[Idx(0, 0)] = 0f;
            x[Idx(0, Height + 1)] = 0f;
            x[Idx(Width + 1, 0)] = 0f;
            x[Idx(Width + 1, Height + 1)] = 0f;

            var solidOpen = _fields.Obstacles.Data;
            for (int y = 1; y <= Height; y++)
            {
                int row = y * Stride;
                for (int xPos = 1; xPos <= Width; xPos++)
                {
                    int idx = row + xPos;
                    if (solidOpen[idx])
                    {
                        x[idx] = 0f;
                    }
                }
            }

            return;
        }

        if (_parameters.BoundaryMode == BoundaryMode.Wrap)
        {
            for (int i = 1; i <= Width; i++)
            {
                x[Idx(i, 0)] = x[Idx(i, Height)];
                x[Idx(i, Height + 1)] = x[Idx(i, 1)];
            }

            for (int j = 1; j <= Height; j++)
            {
                x[Idx(0, j)] = x[Idx(Width, j)];
                x[Idx(Width + 1, j)] = x[Idx(1, j)];
            }

            x[Idx(0, 0)] = x[Idx(Width, Height)];
            x[Idx(0, Height + 1)] = x[Idx(Width, 1)];
            x[Idx(Width + 1, 0)] = x[Idx(1, Height)];
            x[Idx(Width + 1, Height + 1)] = x[Idx(1, 1)];

            var solidWrap = _fields.Obstacles.Data;
            for (int y = 1; y <= Height; y++)
            {
                int row = y * Stride;
                for (int xPos = 1; xPos <= Width; xPos++)
                {
                    int idx = row + xPos;
                    if (solidWrap[idx])
                    {
                        x[idx] = 0f;
                    }
                }
            }

            return;
        }

        for (int i = 1; i <= Width; i++)
        {
            x[Idx(i, 0)] = b == 2 ? -x[Idx(i, 1)] : x[Idx(i, 1)];
            x[Idx(i, Height + 1)] = b == 2 ? -x[Idx(i, Height)] : x[Idx(i, Height)];
        }

        for (int j = 1; j <= Height; j++)
        {
            x[Idx(0, j)] = b == 1 ? -x[Idx(1, j)] : x[Idx(1, j)];
            x[Idx(Width + 1, j)] = b == 1 ? -x[Idx(Width, j)] : x[Idx(Width, j)];
        }

        x[Idx(0, 0)] = 0.5f * (x[Idx(1, 0)] + x[Idx(0, 1)]);
        x[Idx(0, Height + 1)] = 0.5f * (x[Idx(1, Height + 1)] + x[Idx(0, Height)]);
        x[Idx(Width + 1, 0)] = 0.5f * (x[Idx(Width, 0)] + x[Idx(Width + 1, 1)]);
        x[Idx(Width + 1, Height + 1)] = 0.5f * (x[Idx(Width, Height + 1)] + x[Idx(Width + 1, Height)]);

        var solid = _fields.Obstacles.Data;
        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int xPos = 1; xPos <= Width; xPos++)
            {
                int idx = row + xPos;
                if (solid[idx])
                {
                    x[idx] = 0f;
                }
            }
        }
    }

    private static float WrapCoord(float value, int size)
    {
        float min = 0.5f;
        float max = size + 0.5f;
        float span = max - min;
        float wrapped = value - min;
        wrapped = wrapped - MathF.Floor(wrapped / span) * span;
        return wrapped + min;
    }

    private void ClearInteriorEdges(float[] field)
    {
        for (int x = 1; x <= Width; x++)
        {
            field[Idx(x, 1)] = 0f;
            field[Idx(x, Height)] = 0f;
        }

        for (int y = 1; y <= Height; y++)
        {
            field[Idx(1, y)] = 0f;
            field[Idx(Width, y)] = 0f;
        }
    }

    private static void Swap(ref float[] a, ref float[] b)
    {
        (a, b) = (b, a);
    }
}
