using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Solver.Core.Models;

public enum Sense { Max, Min }
public enum RelOp { Le, Ge, Eq }
public enum VarSign { Plus, Minus, Urs, Int, Bin }

public sealed class LpModel
{
    public required Sense ProblemSense { get; init; }
    public required double[] C { get; init; }
    public required double[][] A { get; init; }
    public required RelOp[] RelOps { get; init; }
    public required double[] B { get; init; }
    public required VarSign[] VarSigns { get; init; }
    public string? Name { get; init; }
}

