namespace SmokeSim.Core;

public sealed class Obstacles
{
    private readonly bool[] _solid;

    public Obstacles(int width, int height)
    {
        Width = width;
        Height = height;
        Stride = width + 2;
        _solid = new bool[(width + 2) * (height + 2)];
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
    }

    public int Idx(int x, int y) => x + Stride * y;
}
