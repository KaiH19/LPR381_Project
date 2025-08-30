using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Solver.Core.Models;

namespace Solver.Sensitivity;

/// <summary>
/// Build the classical dual for a primal in standard form (all constraints <=, x>=0).
/// You can expand this for general cases as needed.
/// </summary>
public static class DualBuilder
{
    public static LpModel BuildDual(LpModel primal)
    {
        // Assumes primal is Max, A x <= b, x >= 0
        int m = primal.B.Length;
        int n = primal.C.Length;

        var A = primal.A;
        var AT = new double[n][];
        for (int j = 0; j < n; j++)
        {
            AT[j] = new double[m];
            for (int i = 0; i < m; i++) AT[j][i] = A[i][j];
        }

        return new LpModel
        {
            Name = (primal.Name ?? "Primal") + "_Dual",
            ProblemSense = Sense.Min,
            C = (double[])primal.B.Clone(),   // minimize b^T y
            A = AT,                            // A^T y >= c
            RelOps = Enumerable.Repeat(RelOp.Ge, n).ToArray(),
            B = (double[])primal.C.Clone(),
            VarSigns = Enumerable.Repeat(VarSign.Plus, m).ToArray()
        };
    }
}

