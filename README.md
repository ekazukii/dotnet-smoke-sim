# SmokeSim

Cross-platform .NET 9 desktop app that simulates 2D smoke using a grid-based Stable Fluids solver, rendered in real time with Avalonia.

## How to run

```bash
dotnet restore
dotnet run --project SmokeSim.App
```

## Projects

- `SmokeSim.Core` - Solver engine (no UI dependencies).
- `SmokeSim.App` - Avalonia UI, rendering, and input.

## Controls

- Left-drag uses the selected tool (Smoke, Fan, Wall, Erase Wall).
- Right-drag applies a fan force based on mouse movement.
- Use sliders to tweak viscosity, diffusion, vorticity, buoyancy, dissipation, brush size, and strength.
- Solver Iterations trades accuracy for speed; lower values run faster.
- The simulation uses open boundaries; smoke can leave the domain at the edges.

## Solver overview

The solver is a 2D incompressible grid simulation based on the Stable Fluids method:

1. Apply external sources (density and velocity).
2. Apply buoyancy.
3. Diffuse velocity.
4. Project to enforce incompressibility.
5. Advect velocity.
6. Project again.
7. Add vorticity confinement.
8. Diffuse (optional) and advect density, then apply dissipation.

Obstacles are stored as a solid mask. Solid cells zero velocity and density, and boundary conditions are enforced during diffusion, advection, and projection.
