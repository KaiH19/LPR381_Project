using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Models;
using Solver.Simplex.Common; // CanonicalFormConverter

namespace Solver.Sensitivity;

/// <summary>
/// Sensitivity analysis using the optimal BASIS returned by Simplex.
/// - Variable ranges (non-basic): allowable coef decrease = reduced cost; increase = +∞ (Max problems).
/// - RHS ranges (one-at-a-time): computed with B^{-1} so x_B stays >= 0 with current basis.
/// - Helpers to apply coefficient / RHS changes to a model.
/// Works with current Primal/Revised Simplex outputs (Basis, ReducedCosts, Duals).
/// </summary>
public static class SensitivityAnalysis
{
    public sealed record VariableRange(
        int VarIndex, double ReducedCost, double AllowableDecrease, double AllowableIncrease);

    public sealed record RhsRange(
        int ConstraintIndex, double AllowableDecrease, double AllowableIncrease);

    /// <summary>
    /// Basic variable/dual context extracted from canonical form + current basis.
    /// </summary>
    private sealed class BasisContext
    {
        public required double[,] A;          // canonical A (m x n)
        public required double[] b;           // canonical b (m)
        public required double[] c;           // canonical c (n) (already sign-adjusted by converter)
        public required int m;                // #constraints
        public required int n;                // #variables in canonical
        public required int[] Basis;          // length m, indices into columns of A
        public required int OrigVars;         // # original decision vars
        public required bool IsMax;           // sense
    }

    // ENTRYPOINT #1: Variable sensitivity (non-basic ranges)

    public static VariableRange[] VariableRanges(LpModel original, SolveResult opt)
    {
        // Reduced costs (from solver) are already in canonical sign convention.
        // We only report for ORIGINAL variables (ignore slacks/surplus added by converter).
        var canon = CanonicalFormConverter.Convert(original);
        // opt.ReducedCosts length may be >= original var count (it includes added vars).
        var r = opt.ReducedCosts ?? Array.Empty<double>();
        int n0 = Math.Min(canon.OriginalVariables, r.Length);

        var ranges = new VariableRange[n0];
        for (int j = 0; j < n0; j++)
        {
            double rc = r[j];
            // For a MAX problem in canonical form:
            // - Non-basic vars have rc >= 0 at optimum.
            //   Allowable decrease until rc hits 0 = rc; increase is +∞.
            // - If a var ended basic (rc == 0 but positive level), full basic-var range
            //   requires B^{-1}; we keep a TODO note and return (0, +∞) as placeholder.
            double dec, inc;
            if (rc > 1e-10)
            {
                dec = rc;
                inc = double.PositiveInfinity;
            }
            else
            {
                // basic or numerically zero reduced cost: full range requires basis math; placeholder
                dec = 0.0;
                inc = double.PositiveInfinity;
            }

            ranges[j] = new VariableRange(j, rc, dec, inc);
        }
        return ranges;
    }

    // ENTRYPOINT #2: RHS ranges (one constraint at a time)

    public static RhsRange[] RhsRanges(LpModel original, SolveResult opt)
    {
        var ctx = BuildBasisContext(original, opt);
        // Build B from A columns in Basis and invert it
        var B = ExtractB(ctx.A, ctx.Basis);
        var Binv = Invert(B);

        // Current basic solution x_B = B^{-1} b
        var xB = Multiply(Binv, ctx.b); // length m

        var ranges = new RhsRange[ctx.m];

        // Change RHS of a single constraint k by delta: b' = b + delta * e_k
        // New basic solution: x_B' = x_B + delta * (B^{-1} e_k) = x_B + delta * u(:,k)
        // Feasibility requires x_B' >= 0  => for each i, xB[i] + delta * u[i] >= 0
        // => delta bounds:
        //    if u[i] > 0:   delta >= -xB[i]/u[i]  (lower bound candidate)
        //    if u[i] < 0:   delta <= -xB[i]/u[i]  (upper bound candidate)
        for (int k = 0; k < ctx.m; k++)
        {
            var u = Column(Binv, k); // effect vector for unit change in b_k

            double lower = double.NegativeInfinity;
            double upper = double.PositiveInfinity;

            for (int i = 0; i < ctx.m; i++)
            {
                if (u[i] > 1e-12)
                    lower = Math.Max(lower, -xB[i] / u[i]);
                else if (u[i] < -1e-12)
                    upper = Math.Min(upper, -xB[i] / u[i]);
            }

            ranges[k] = new RhsRange(k, AllowableDecrease: -lower, AllowableIncrease: upper);
        }

        return ranges;
    }

    // ENTRYPOINT #3: Apply changes 

    public static LpModel ApplyCoeffChange(LpModel model, int varIndex, double delta)
    {
        var c2 = (double[])model.C.Clone();
        c2[varIndex] += delta;
        return new LpModel
        {
            Name = model.Name,
            ProblemSense = model.ProblemSense,
            C = c2,
            A = model.A,
            B = model.B,
            RelOps = model.RelOps,
            VarSigns = model.VarSigns
        };
    }

    public static LpModel ApplyRhsChange(LpModel model, int constraintIndex, double delta)
    {
        var b2 = (double[])model.B.Clone();
        b2[constraintIndex] += delta;
        return new LpModel
        {
            Name = model.Name,
            ProblemSense = model.ProblemSense,
            C = model.C,
            A = model.A,
            B = b2,
            RelOps = model.RelOps,
            VarSigns = model.VarSigns
        };
    }

    // helpers / linear algebra 

    private static BasisContext BuildBasisContext(LpModel original, SolveResult opt)
    {
        if (opt.Basis == null || opt.Basis.Length == 0)
            throw new InvalidOperationException("Sensitivity requires an optimal Basis from Simplex.");

        var canon = CanonicalFormConverter.Convert(original); // fields we need are public in CanonicalForm
        return new BasisContext
        {
            A = canon.A,
            b = canon.B,
            c = canon.C,
            m = canon.OriginalConstraints,
            n = canon.A.GetLength(1),
            Basis = opt.Basis,
            OrigVars = canon.OriginalVariables,
            IsMax = canon.IsMaximization
        };
    }

    private static double[,] ExtractB(double[,] A, int[] basis)
    {
        int m = basis.Length;
        var B = new double[m, m];
        for (int i = 0; i < m; i++)
            for (int j = 0; j < m; j++)
                B[i, j] = A[i, basis[j]];
        return B;
    }

    private static double[] Multiply(double[,] M, double[] v)
    {
        int r = M.GetLength(0), c = M.GetLength(1);
        var y = new double[r];
        for (int i = 0; i < r; i++)
        {
            double s = 0;
            for (int j = 0; j < c; j++) s += M[i, j] * v[j];
            y[i] = s;
        }
        return y;
    }

    private static double[] Column(double[,] M, int col)
    {
        int r = M.GetLength(0);
        var y = new double[r];
        for (int i = 0; i < r; i++) y[i] = M[i, col];
        return y;
    }

    // Simple Gauss-Jordan inversion (small m x m)
    private static double[,] Invert(double[,] B)
    {
        int n = B.GetLength(0);
        var A = (double[,])B.Clone();
        var I = new double[n, n];
        for (int i = 0; i < n; i++) I[i, i] = 1.0;

        for (int p = 0; p < n; p++)
        {
            // pivot
            double piv = A[p, p];
            if (Math.Abs(piv) < 1e-12) throw new InvalidOperationException("Basis matrix is singular.");
            double inv = 1.0 / piv;

            for (int j = 0; j < n; j++) { A[p, j] *= inv; I[p, j] *= inv; }

            // eliminate other rows
            for (int i = 0; i < n; i++)
            {
                if (i == p) continue;
                double f = A[i, p];
                if (Math.Abs(f) < 1e-16) continue;
                for (int j = 0; j < n; j++)
                {
                    A[i, j] -= f * A[p, j];
                    I[i, j] -= f * I[p, j];
                }
            }
        }
        return I;
    }
}
