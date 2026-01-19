using System;
using Avalonia.Controls;
using Avalonia.Input;
using SmokeSim.App.ViewModels;

namespace SmokeSim.App.Views;

public partial class MainWindow : Window
{
    private MainWindowViewModel? _viewModel;

    public MainWindow()
    {
        InitializeComponent();
        DataContextChanged += OnDataContextChanged;
    }

    private void OnDataContextChanged(object? sender, EventArgs e)
    {
        if (_viewModel != null)
        {
            _viewModel.RenderRequested -= OnRenderRequested;
        }

        _viewModel = DataContext as MainWindowViewModel;
        if (_viewModel != null)
        {
            _viewModel.RenderRequested += OnRenderRequested;
        }
    }

    private void OnRenderRequested()
    {
        RenderImage?.InvalidateVisual();
    }

    private void OnPointerPressed(object? sender, PointerPressedEventArgs e)
    {
        if (DataContext is not MainWindowViewModel viewModel || sender is not Control control)
        {
            return;
        }

        e.Pointer.Capture(control);
        var point = e.GetPosition(control);
        var current = e.GetCurrentPoint(control);
        var props = current.Properties;
        bool leftPressed = props.IsLeftButtonPressed || props.PointerUpdateKind == PointerUpdateKind.LeftButtonPressed;
        bool rightPressed = props.IsRightButtonPressed || props.PointerUpdateKind == PointerUpdateKind.RightButtonPressed;
        if (!leftPressed && !rightPressed)
        {
            leftPressed = true;
        }
        viewModel.HandlePointerPressed(point, control.Bounds.Size, leftPressed, rightPressed);
        e.Handled = true;
    }

    private void OnPointerMoved(object? sender, PointerEventArgs e)
    {
        if (DataContext is not MainWindowViewModel viewModel || sender is not Control control)
        {
            return;
        }

        var point = e.GetPosition(control);
        var current = e.GetCurrentPoint(control);
        var props = current.Properties;
        bool leftPressed = props.IsLeftButtonPressed || props.PointerUpdateKind == PointerUpdateKind.LeftButtonPressed;
        bool rightPressed = props.IsRightButtonPressed || props.PointerUpdateKind == PointerUpdateKind.RightButtonPressed;
        viewModel.HandlePointerMoved(point, control.Bounds.Size, leftPressed, rightPressed);
    }

    private void OnPointerReleased(object? sender, PointerReleasedEventArgs e)
    {
        if (DataContext is not MainWindowViewModel viewModel || sender is not Control control)
        {
            return;
        }

        e.Pointer.Capture(null);
        viewModel.HandlePointerReleased();
        e.Handled = true;
    }

    private void OnPointerCaptureLost(object? sender, PointerCaptureLostEventArgs e)
    {
        if (DataContext is not MainWindowViewModel viewModel)
        {
            return;
        }

        viewModel.HandlePointerReleased();
    }
}
