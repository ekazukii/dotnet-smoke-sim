namespace SmokeSim.Core;

public sealed class SimulationParameters
{
    public float Viscosity { get; set; } = 0.0001f;
    public float Diffusion { get; set; } = 0.0005f;
    public float Vorticity { get; set; } = 15.0f;
    public float Buoyancy { get; set; } = 1.0f;
    public float Dissipation { get; set; } = 0.10f;
    public int SolverIterations { get; set; } = 16;
    public BoundaryMode BoundaryMode { get; set; } = BoundaryMode.Bounce;
}
