using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Interfaces;
using Solver.Core.Models;

namespace Solver.Simplex.Primal;

public sealed class PrimalSimplexSolver : ISolver
{
    public string Name => "Primal Simplex (Tableau)";

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
        => new() { Status = SolveStatus.NotSolved };
}

