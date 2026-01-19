# SmokeSim

Cross-platform .NET 9 desktop app that simulates 2D smoke using a grid-based Stable Fluids solver, rendered in real time with Avalonia.

## How to run

```bash
dotnet restore
dotnet run -c Release --project SmokeSim.App
```

## Projects

- `SmokeSim.Core` - Solver engine (no UI dependencies).
- `SmokeSim.App` - Avalonia UI, rendering, and input.

## Controls

- Left-drag uses the selected tool (Smoke, Fan, Wall, Erase Wall).
- Right-drag applies a fan force based on mouse movement.
- Use sliders to tweak viscosity, diffusion, vorticity, buoyancy, dissipation, brush size, and strength.
- Solver Iterations trades accuracy for speed; lower values run faster.
- Sim Speed scales the effective timestep.
- The simulation uses open boundaries; smoke can leave the domain at the edges.

## Math model

The simulation uses a 2D incompressible flow with density d and velocity u = (u, v).

Continuity (incompressibility):
```
div u = 0
```

Momentum (Stable Fluids / Boussinesq form):
```
du/dt = -grad p + nu * laplacian u + f
f = (0, -buoyancy * d)
```

Density transport:
```
dd/dt + u · grad d = kappa * laplacian d
```

Where nu is viscosity and kappa is density diffusion.

## Discretization (implementation)

Grid:
- 2D collocated grid with ghost cells (size = (W+2) x (H+2)).
- All fields are float arrays on the CPU.

Per-step sequence:
1. Apply external sources (velocity).
2. Apply buoyancy to v.
3. Diffuse velocity (Gauss-Seidel iterations).
4. Project to enforce div u = 0.
5. Advect velocity (semi-Lagrangian).
6. Project again.
7. Add vorticity confinement.
8. Diffuse (optional) and advect density, then apply dissipation.

Advection:
- Backtrace with Euler: x' = x - dt * u(x).
- Bilinear sample from the previous field.

Projection:
- Compute divergence using centered differences.
- Solve laplacian p = div using Gauss-Seidel.
- Subtract grad p from velocity.

Obstacles:
- Solid cells are masked out and set to zero.
- Neighbor checks prevent sampling through solids.
- Open boundaries set ghost cells to zero and clear interior edge cells.

Stability:
- CFL is estimated with max(|u|, |v|).
- The solver substeps when CFL would exceed ~0.9.
