using System;

namespace SmokeSim.Core;

public sealed class FluidSolver2D
{
    private const float MaxVelocity = 10f;
    private const int MaxCflSubsteps = 4;
    private const float MaxFrameDt = 1f / 30f;
    private FluidFields _fields;
    private readonly SimulationParameters _parameters;

    public FluidSolver2D(int width, int height, SimulationParameters? parameters = null)
    {
        _fields = new FluidFields(width, height);
        _parameters = parameters ?? new SimulationParameters();
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

    public void Step(float dt)
    {
        if (dt <= 0f)
        {
            return;
        }

        if (dt > MaxFrameDt)
        {
            dt = MaxFrameDt;
        }

        float maxSpeed = ComputeMaxSpeed();
        float cfl = maxSpeed * dt * Math.Max(Width, Height);
        int steps = cfl <= 0f ? 1 : (int)MathF.Ceiling(cfl / 0.9f);
        steps = Math.Clamp(steps, 1, MaxCflSubsteps);

        float subDt = dt / steps;
        for (int i = 0; i < steps; i++)
        {
            StepInternal(subDt);
        }
    }

    public SolverDiagnostics ComputeDiagnostics(float dt)
    {
        var u = _fields.VelocityU;
        var v = _fields.VelocityV;
        var density = _fields.Density;
        var solid = _fields.Obstacles.Data;

        float maxU = 0f;
        float maxV = 0f;
        float minD = float.PositiveInfinity;
        float maxD = 0f;
        float maxDiv = 0f;
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
                    continue;
                }

                float uAbs = MathF.Abs(u[idx]);
                float vAbs = MathF.Abs(v[idx]);
                if (uAbs > maxU)
                {
                    maxU = uAbs;
                }
                if (vAbs > maxV)
                {
                    maxV = vAbs;
                }

                float d = density[idx];
                if (float.IsFinite(d))
                {
                    if (d < minD)
                    {
                        minD = d;
                    }
                    if (d > maxD)
                    {
                        maxD = d;
                    }
                }

                float uR = solid[idx + 1] ? 0f : u[idx + 1];
                float uL = solid[idx - 1] ? 0f : u[idx - 1];
                float vU = solid[idx + Stride] ? 0f : v[idx + Stride];
                float vD = solid[idx - Stride] ? 0f : v[idx - Stride];
                float div = -0.5f * ((uR - uL) * invW + (vU - vD) * invH);
                float absDiv = MathF.Abs(div);
                if (absDiv > maxDiv)
                {
                    maxDiv = absDiv;
                }
            }
        }

        if (!float.IsFinite(minD))
        {
            minD = 0f;
        }

        float maxVel = MathF.Max(maxU, maxV);
        float cfl = maxVel * dt * Math.Max(Width, Height);

        return new SolverDiagnostics(maxU, maxV, maxVel, cfl, minD, maxD, maxDiv);
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

    private void StepInternal(float dt)
    {
        AddSources(_fields.VelocityU, _fields.VelocityUPrev, dt);
        AddSources(_fields.VelocityV, _fields.VelocityVPrev, dt);

        ApplyBuoyancy(_fields.VelocityV, _fields.Density, dt);
        ClampVelocity(_fields.VelocityU, _fields.VelocityV);

        if (_parameters.Viscosity > 0f)
        {
            Swap(ref _fields.VelocityUPrev, ref _fields.VelocityU);
            Diffuse(1, _fields.VelocityU, _fields.VelocityUPrev, _parameters.Viscosity, dt);

            Swap(ref _fields.VelocityVPrev, ref _fields.VelocityV);
            Diffuse(2, _fields.VelocityV, _fields.VelocityVPrev, _parameters.Viscosity, dt);
        }

        Project(_fields.VelocityU, _fields.VelocityV, _fields.Pressure, _fields.Divergence);
        ClampVelocity(_fields.VelocityU, _fields.VelocityV);

        Swap(ref _fields.VelocityUPrev, ref _fields.VelocityU);
        Swap(ref _fields.VelocityVPrev, ref _fields.VelocityV);
        Advect(1, _fields.VelocityU, _fields.VelocityUPrev, _fields.VelocityUPrev, _fields.VelocityVPrev, dt);
        Advect(2, _fields.VelocityV, _fields.VelocityVPrev, _fields.VelocityUPrev, _fields.VelocityVPrev, dt);

        Project(_fields.VelocityU, _fields.VelocityV, _fields.Pressure, _fields.Divergence);

        if (_parameters.Vorticity > 0f)
        {
            ApplyVorticityConfinement(_fields.VelocityU, _fields.VelocityV, dt, _parameters.Vorticity);
        }
        ClampVelocity(_fields.VelocityU, _fields.VelocityV);

        if (_parameters.Diffusion > 0f)
        {
            Swap(ref _fields.DensityPrev, ref _fields.Density);
            Diffuse(0, _fields.Density, _fields.DensityPrev, _parameters.Diffusion, dt);
        }

        Swap(ref _fields.DensityPrev, ref _fields.Density);
        Advect(0, _fields.Density, _fields.DensityPrev, _fields.VelocityU, _fields.VelocityV, dt);

        ClearInteriorEdges(_fields.Density);
        ApplyDissipation(_fields.Density, dt);
    }

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

    private float ComputeMaxSpeed()
    {
        var u = _fields.VelocityU;
        var v = _fields.VelocityV;
        var solid = _fields.Obstacles.Data;
        float max = 0f;

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

                float uAbs = MathF.Abs(u[idx]);
                float vAbs = MathF.Abs(v[idx]);
                if (uAbs > max)
                {
                    max = uAbs;
                }
                if (vAbs > max)
                {
                    max = vAbs;
                }
            }
        }

        return max;
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

        bool isVelocity = b != 0;
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

                if (x < 0.5f || x > Width + 0.5f || yPos < 0.5f || yPos > Height + 0.5f)
                {
                    d[idx] = 0f;
                    continue;
                }

                x = Math.Clamp(x, 0.5f, Width + 0.5f);
                yPos = Math.Clamp(yPos, 0.5f, Height + 0.5f);

                if (!float.IsFinite(x) || !float.IsFinite(yPos))
                {
                    d[idx] = 0f;
                    continue;
                }

                float midX = (xPos + x) * 0.5f;
                float midY = (y + yPos) * 0.5f;
                int midXi = (int)MathF.Floor(midX);
                int midYi = (int)MathF.Floor(midY);
                midXi = Math.Clamp(midXi, 0, Width + 1);
                midYi = Math.Clamp(midYi, 0, Height + 1);
                if (solid[Idx(midXi, midYi)])
                {
                    d[idx] = isVelocity ? 0f : d0[idx];
                    continue;
                }

                int i0 = (int)MathF.Floor(x);
                int i1 = i0 + 1;
                int j0 = (int)MathF.Floor(yPos);
                int j1 = j0 + 1;
                i0 = Math.Clamp(i0, 0, Width + 1);
                i1 = Math.Clamp(i1, 0, Width + 1);
                j0 = Math.Clamp(j0, 0, Height + 1);
                j1 = Math.Clamp(j1, 0, Height + 1);

                float s1 = x - i0;
                float s0 = 1f - s1;
                float t1 = yPos - j0;
                float t0 = 1f - t1;

                int idx00 = Idx(i0, j0);
                int idx10 = Idx(i1, j0);
                int idx01 = Idx(i0, j1);
                int idx11 = Idx(i1, j1);

                bool solid00 = solid[idx00];
                bool solid10 = solid[idx10];
                bool solid01 = solid[idx01];
                bool solid11 = solid[idx11];
                if (solid00 || solid10 || solid01 || solid11)
                {
                    d[idx] = isVelocity ? 0f : d0[idx];
                    continue;
                }

                float d00 = d0[idx00];
                float d10 = d0[idx10];
                float d01 = d0[idx01];
                float d11 = d0[idx11];

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

        ApplyObstacleBoundaries(u, v);

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

    private void ApplyObstacleBoundaries(float[] u, float[] v)
    {
        var solid = _fields.Obstacles.Data;

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

                if (solid[idx - 1] || solid[idx + 1])
                {
                    u[idx] = 0f;
                }

                if (solid[idx - Stride] || solid[idx + Stride])
                {
                    v[idx] = 0f;
                }
            }
        }
    }

    // Prevent extreme velocities from destabilizing advection/projection.
    private void ClampVelocity(float[] u, float[] v)
    {
        float max = MaxVelocity;
        if (max <= 0f)
        {
            return;
        }

        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                int idx = row + x;
                u[idx] = Math.Clamp(u[idx], -max, max);
                v[idx] = Math.Clamp(v[idx], -max, max);
            }
        }
    }

    private void SetBounds(int b, float[] x)
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
