using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Models;

namespace Solver.Core.Interfaces;

public interface ISolver
{
    string Name { get; }
    SolveResult Solve(LpModel model, SolverOptions? options = null);
}

public sealed class SolverOptions
{
    public bool Verbose { get; init; } = true;
    public string? OutputFile { get; init; }
    public int MaxIterations { get; init; } = 50;
}

