using System;
using System.Collections.Generic;
using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using SmokeSim.App.Models;
using SmokeSim.App.Rendering;
using SmokeSim.Core;

namespace SmokeSim.App.ViewModels;

public partial class MainWindowViewModel : ViewModelBase
{
    private readonly FluidSolver2D _solver;
    private readonly SmokeRenderer _renderer;
    private readonly DispatcherTimer _timer;
    private readonly Stopwatch _frameTimer = new();
    private readonly Stopwatch _fpsTimer = new();
    private Point? _lastPointer;
    private bool _isLeftDown;
    private bool _isRightDown;
    private int _frames;

    public event Action? RenderRequested;

    public MainWindowViewModel()
    {
        _solver = new FluidSolver2D(240, 240);
        _renderer = new SmokeRenderer(_solver.Width, _solver.Height);

        ToolModes = Enum.GetValues<ToolMode>();
        SelectedTool = ToolMode.Smoke;
        BoundaryModes = Enum.GetValues<BoundaryMode>();

        Viscosity = _solver.Parameters.Viscosity;
        Diffusion = _solver.Parameters.Diffusion;
        Vorticity = _solver.Parameters.Vorticity;
        Buoyancy = _solver.Parameters.Buoyancy;
        Dissipation = _solver.Parameters.Dissipation;
        BoundaryMode = _solver.Parameters.BoundaryMode;
        SolverIterations = _solver.Parameters.SolverIterations;

        BrushSize = 8f;
        BrushStrength = 120f;

        _timer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(16),
        };
        _timer.Tick += (_, _) => OnTick();

        IsPaused = true;
        Render();
        _frameTimer.Start();
        _fpsTimer.Start();
        _timer.Start();

        Bitmap = _renderer.Bitmap;
    }

    public IReadOnlyList<ToolMode> ToolModes { get; }
    public IReadOnlyList<BoundaryMode> BoundaryModes { get; }

    [ObservableProperty]
    private WriteableBitmap _bitmap = null!;

    [ObservableProperty]
    private ToolMode _selectedTool;

    [ObservableProperty]
    private float _brushSize;

    [ObservableProperty]
    private float _brushStrength;

    [ObservableProperty]
    private float _viscosity;

    [ObservableProperty]
    private float _diffusion;

    [ObservableProperty]
    private float _vorticity;

    [ObservableProperty]
    private float _buoyancy;

    [ObservableProperty]
    private float _dissipation;

    [ObservableProperty]
    private bool _isPaused;

    [ObservableProperty]
    private double _fps;

    [ObservableProperty]
    private BoundaryMode _boundaryMode;

    [ObservableProperty]
    private int _solverIterations;

    public string PauseResumeText => IsPaused ? "Resume" : "Pause";

    public void HandlePointerPressed(Point position, Size surfaceSize, bool leftPressed, bool rightPressed)
    {
        _lastPointer = position;
        _isLeftDown = leftPressed;
        _isRightDown = rightPressed;
        ApplyInput(position, surfaceSize, leftPressed, rightPressed, default);
        if (IsPaused)
        {
            Render();
        }
    }

    public void HandlePointerMoved(Point position, Size surfaceSize)
    {
        var delta = _lastPointer.HasValue ? position - _lastPointer.Value : default;
        _lastPointer = position;

        ApplyInput(position, surfaceSize, _isLeftDown, _isRightDown, delta);
        if (IsPaused)
        {
            Render();
        }
    }

    public void HandlePointerReleased()
    {
        _lastPointer = null;
        _isLeftDown = false;
        _isRightDown = false;
    }

    [RelayCommand]
    private void TogglePause()
    {
        IsPaused = !IsPaused;
        if (!IsPaused)
        {
            _frameTimer.Restart();
            _fpsTimer.Restart();
            _frames = 0;
        }
    }

    [RelayCommand]
    private void Step()
    {
        _solver.Step(1f / 60f);
        Render();
    }

    [RelayCommand]
    private void Reset()
    {
        _solver.Clear();
        _solver.ResetObstacles();
    }

    [RelayCommand]
    private void Preset()
    {
        _presetIndex = (_presetIndex + 1) % _presets.Length;
        ApplyPreset(_presets[_presetIndex]);
    }

    partial void OnIsPausedChanged(bool value)
    {
        OnPropertyChanged(nameof(PauseResumeText));
    }

    partial void OnViscosityChanged(float value)
    {
        _solver.Parameters.Viscosity = value;
    }

    partial void OnDiffusionChanged(float value)
    {
        _solver.Parameters.Diffusion = value;
    }

    partial void OnVorticityChanged(float value)
    {
        _solver.Parameters.Vorticity = value;
    }

    partial void OnBuoyancyChanged(float value)
    {
        _solver.Parameters.Buoyancy = value;
    }

    partial void OnDissipationChanged(float value)
    {
        _solver.Parameters.Dissipation = value;
    }

    partial void OnBoundaryModeChanged(BoundaryMode value)
    {
        _solver.ApplyBoundaryMode(value);
    }

    partial void OnSolverIterationsChanged(int value)
    {
        if (value < 1)
        {
            value = 1;
        }

        _solver.Parameters.SolverIterations = value;
    }

    private int _presetIndex;
    private readonly SolverPreset[] _presets =
    [
        new("Calm", 0.0002f, 0.0001f, 6f, 2f, 0.03f, 12),
        new("Swirl", 0.0004f, 0.0002f, 12f, 4f, 0.05f, 16),
        new("Turbulent", 0.0006f, 0.00025f, 18f, 6f, 0.07f, 20),
    ];

    private void ApplyPreset(SolverPreset preset)
    {
        Viscosity = preset.Viscosity;
        Diffusion = preset.Diffusion;
        Vorticity = preset.Vorticity;
        Buoyancy = preset.Buoyancy;
        Dissipation = preset.Dissipation;
        SolverIterations = preset.Iterations;
    }

    private void ApplyInput(Point position, Size surfaceSize, bool leftPressed, bool rightPressed, Vector delta)
    {
        if (surfaceSize.Width <= 1 || surfaceSize.Height <= 1)
        {
            return;
        }

        int x = (int)Math.Clamp(position.X / surfaceSize.Width * _solver.Width, 1, _solver.Width);
        int y = (int)Math.Clamp(position.Y / surfaceSize.Height * _solver.Height, 1, _solver.Height);
        int radius = (int)MathF.Max(1f, BrushSize);

        if (rightPressed)
        {
            ApplyFan(x, y, surfaceSize, delta);
        }

        if (!leftPressed)
        {
            return;
        }

        switch (SelectedTool)
        {
            case ToolMode.Smoke:
                _solver.AddDensityCircle(x, y, BrushStrength, radius);
                break;
            case ToolMode.Fan:
                ApplyFan(x, y, surfaceSize, delta);
                break;
            case ToolMode.Wall:
                _solver.SetObstacleCircle(x, y, radius, true);
                break;
            case ToolMode.EraseWall:
                _solver.SetObstacleCircle(x, y, radius, false);
                break;
            default:
                break;
        }
    }

    private void ApplyFan(int x, int y, Size surfaceSize, Vector delta)
    {
        if (delta == default)
        {
            return;
        }

        float scaleX = _solver.Width / (float)surfaceSize.Width;
        float scaleY = _solver.Height / (float)surfaceSize.Height;
        float forceX = (float)delta.X * scaleX * BrushStrength;
        float forceY = (float)delta.Y * scaleY * BrushStrength;
        _solver.AddVelocityCircle(x, y, forceX, forceY, (int)MathF.Max(1f, BrushSize));
    }

    private void OnTick()
    {
        if (IsPaused)
        {
            if (_fpsTimer.Elapsed.TotalSeconds >= 0.25)
            {
                Fps = 0;
                _fpsTimer.Restart();
            }

            return;
        }

        float dt = (float)_frameTimer.Elapsed.TotalSeconds;
        _frameTimer.Restart();

        if (dt > 0.05f)
        {
            dt = 0.05f;
        }

        if (!IsPaused)
        {
            _solver.Step(dt);
            Render();
        }

        _frames++;
        if (_fpsTimer.Elapsed.TotalSeconds >= 1.0)
        {
            Fps = _frames / _fpsTimer.Elapsed.TotalSeconds;
            _frames = 0;
            _fpsTimer.Restart();
        }
    }


    private readonly record struct SolverPreset(
        string Name,
        float Viscosity,
        float Diffusion,
        float Vorticity,
        float Buoyancy,
        float Dissipation,
        int Iterations);

    private void Render()
    {
        _renderer.Update(_solver);
        OnPropertyChanged(nameof(Bitmap));
        RenderRequested?.Invoke();
    }
}
