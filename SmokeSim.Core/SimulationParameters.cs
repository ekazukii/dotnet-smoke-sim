namespace SmokeSim.Core;

public sealed class SimulationParameters
{
    public float Viscosity { get; set; } = 0.0004f;
    public float Diffusion { get; set; } = 0.0002f;
    public float Vorticity { get; set; } = 12.0f;
    public float Buoyancy { get; set; } = 4.0f;
    public float Dissipation { get; set; } = 0.05f;
    public int SolverIterations { get; set; } = 16;
    public BoundaryMode BoundaryMode { get; set; } = BoundaryMode.Bounce;
}
