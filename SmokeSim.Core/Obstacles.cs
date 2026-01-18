namespace SmokeSim.Core;

public sealed class Obstacles
{
    private readonly bool[] _solid;
    private bool _borderSolid = true;

    public Obstacles(int width, int height)
    {
        Width = width;
        Height = height;
        Stride = width + 2;
        _solid = new bool[(width + 2) * (height + 2)];
        SetBorderSolid(true);
    }

    public int Width { get; }
    public int Height { get; }
    public int Stride { get; }

    public bool[] Data => _solid;

    public bool IsSolid(int x, int y) => _solid[Idx(x, y)];

    public void SetSolid(int x, int y, bool value)
    {
        if (x < 1 || x > Width || y < 1 || y > Height)
        {
            return;
        }

        _solid[Idx(x, y)] = value;
    }

    public void ClearInterior()
    {
        for (int y = 1; y <= Height; y++)
        {
            int row = y * Stride;
            for (int x = 1; x <= Width; x++)
            {
                _solid[row + x] = false;
            }
        }

        SetBorderSolid(_borderSolid);
    }

    public void SetBorderSolid(bool solid)
    {
        _borderSolid = solid;
        for (int x = 0; x <= Width + 1; x++)
        {
            _solid[Idx(x, 0)] = solid;
            _solid[Idx(x, Height + 1)] = solid;
        }

        for (int y = 0; y <= Height + 1; y++)
        {
            _solid[Idx(0, y)] = solid;
            _solid[Idx(Width + 1, y)] = solid;
        }
    }

    public int Idx(int x, int y) => x + Stride * y;
}
