namespace SmokeSim.Core;

public readonly record struct SolverDiagnostics(
    float MaxU,
    float MaxV,
    float MaxVelocity,
    float Cfl,
    float MinDensity,
    float MaxDensity,
    float MaxDivergence);
