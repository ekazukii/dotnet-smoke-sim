namespace SmokeSim.Core;

public sealed class FluidFields
{
    public FluidFields(int width, int height)
    {
        Width = width;
        Height = height;
        Stride = width + 2;
        Size = (width + 2) * (height + 2);

        Density = new float[Size];
        DensityPrev = new float[Size];
        VelocityU = new float[Size];
        VelocityV = new float[Size];
        VelocityUPrev = new float[Size];
        VelocityVPrev = new float[Size];
        Pressure = new float[Size];
        Divergence = new float[Size];
        Curl = new float[Size];
        Scratch = new float[Size];

        Obstacles = new Obstacles(width, height);
    }

    public int Width { get; }
    public int Height { get; }
    public int Stride { get; }
    public int Size { get; }

    public float[] Density;
    public float[] DensityPrev;
    public float[] VelocityU;
    public float[] VelocityV;
    public float[] VelocityUPrev;
    public float[] VelocityVPrev;
    public float[] Pressure;
    public float[] Divergence;
    public float[] Curl;
    public float[] Scratch;

    public Obstacles Obstacles { get; }
}
