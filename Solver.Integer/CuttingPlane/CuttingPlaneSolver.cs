using System;
using System.Collections.Generic;
using System.Linq;
using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Integer.CuttingPlane.Controllers;

namespace Solver.Integer.CuttingPlane;

/// <summary>
/// Clean, basic implementation of Gomory's cutting plane algorithm using MVC pattern.
/// Focuses on solving LP relaxation with simplex and applying cutting planes.
/// </summary>
public sealed class CuttingPlaneSolver : ISolver
{
    public string Name => "Cutting Plane (Gomory)";
    
    private readonly CuttingPlaneController _controller;
    
    public CuttingPlaneSolver()
    {
        _controller = new CuttingPlaneController();
    }

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        return _controller.Solve(model, options);
    }
}

