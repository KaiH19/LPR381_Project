using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Solver.Core.Models;

public enum SolveStatus { Optimal, Infeasible, Unbounded, Feasible, NotSolved }

public sealed class SolveResult
{
    public SolveStatus Status { get; init; } = SolveStatus.NotSolved;
    public double[]? X { get; init; }
    public double? ObjectiveValue { get; init; }
    public int[]? Basis { get; init; }
    public double[]? ReducedCosts { get; init; }
    public double[]? Duals { get; init; }
    public string LogPath { get; init; } = string.Empty;
}
