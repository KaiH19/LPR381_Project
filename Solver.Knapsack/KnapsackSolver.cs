using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Core.Utils;
using System.Text;

namespace Solver.Knapsack;

/// <summary>
/// 0/1 Knapsack Branch & Bound (single <= capacity constraint, all variables Bin).
/// Uses ratio-sort and fractional-relaxation upper bound.
/// </summary>
public sealed class KnapsackSolver : ISolver
{
    public string Name => "Knapsack (0/1) Branch & Bound";

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        options ??= new SolverOptions();

        // ---- Validate model --------------------------------------------------
        if (model.ProblemSense != Sense.Max)
            return Fail("Knapsack solver expects a Max problem.", options);

        if (model.VarSigns.Any(v => v != VarSign.Bin))
            return Fail("All variables must be binary (bin).", options);

        // Find a single <= capacity constraint
        int capIdx = Array.FindIndex(model.RelOps, r => r == RelOp.Le);
        if (capIdx < 0)
            return Fail("Need at least one '<=' capacity constraint.", options);

        var w = model.A[capIdx];
        var C = model.B[capIdx];

        if (w.Any(x => x < 0) || C < 0)
            return Fail("Weights and capacity must be non-negative.", options);

        int n = model.C.Length;
        if (w.Length != n)
            return Fail("Capacity row length != number of variables.", options);

        var p = model.C;

        // ---- Prepare order (ratio sort) -------------------------------------
        var order = Enumerable.Range(0, n)
                              .Select(i => new { i, ratio = (w[i] > 0 ? p[i] / w[i] : double.PositiveInfinity) })
                              .OrderByDescending(t => t.ratio)
                              .Select(t => t.i)
                              .ToArray();

        // Arrays in ratio order
        var wOrd = order.Select(i => w[i]).ToArray();
        var pOrd = order.Select(i => p[i]).ToArray();

        // ---- Branch & Bound (DFS) -------------------------------------------
        var bestProfit = 0.0;
        var bestTakeOrd = new int[n]; // 0/1 in "ratio" order
        var take = new int[n];

        var log = new StringBuilder();
        int nodeId = 0;

        // Precompute prefix sums for fast bound
        var prefixW = new double[n + 1];
        var prefixP = new double[n + 1];
        for (int i = 0; i < n; i++)
        {
            prefixW[i + 1] = prefixW[i] + wOrd[i];
            prefixP[i + 1] = prefixP[i] + pOrd[i];
        }

        // fractional bound from level k with remaining capacity capRem
        double Bound(int k, double profit, double capRem)
        {
            // Greedy take from k..n-1 (possibly fractional last)
            double ub = profit;
            for (int i = k; i < n && capRem > 1e-12; i++)
            {
                if (wOrd[i] <= capRem)
                {
                    ub += pOrd[i];
                    capRem -= wOrd[i];
                }
                else
                {
                    ub += pOrd[i] * (capRem / wOrd[i]); // fractional
                    break;
                }
            }
            return ub;
        }

        void Dfs(int level, double profit, double weight, double capRem)
        {
            nodeId++;
            var ub = Bound(level, profit, capRem);
            log.AppendLine($"Node {nodeId,4}: lvl={level,2} w={Fmt(weight)} z={Fmt(profit)} UB={Fmt(ub)} capRem={Fmt(capRem)}");

            if (ub <= bestProfit + 1e-9) { log.AppendLine("  PRUNE (bound)"); return; }
            if (level == n)
            {
                if (profit > bestProfit)
                {
                    bestProfit = profit;
                    Array.Copy(take, bestTakeOrd, n);
                    log.AppendLine("  ** NEW INCUMBENT **");
                }
                return;
            }

            // Branch 1: include item (if fits)
            if (wOrd[level] <= capRem)
            {
                take[level] = 1;
                Dfs(level + 1, profit + pOrd[level], weight + wOrd[level], capRem - wOrd[level]);
                take[level] = 0;
            }
            else
            {
                log.AppendLine("  skip include (does not fit)");
            }

            // Branch 2: exclude item
            take[level] = 0;
            Dfs(level + 1, profit, weight, capRem);
        }

        Dfs(0, 0.0, 0.0, C);

        // map back to original variable order
        var xOrd = bestTakeOrd.Select(v => (double)v).ToArray();
        var x = new double[n];
        for (int pos = 0; pos < n; pos++)
            x[order[pos]] = xOrd[pos];

        // ---- Output ----------------------------------------------------------
        var outLines = new List<string>
        {
            $"Solver: {Name}",
            $"Objective (best): {Fmt(bestProfit)}",
            $"Chosen items (1-indexed): {string.Join(", ", Enumerable.Range(0,n).Where(i => x[i] > 0.5).Select(i => i+1))}",
            $"x = [{string.Join(" ", x.Select(v => ((int)v).ToString()))}]",
            ""
        };
        outLines.AddRange(log.ToString().Split(Environment.NewLine, StringSplitOptions.None));

        Logger.WriteLines(options.OutputFile, outLines);

        return new SolveResult
        {
            Status = SolveStatus.Optimal,
            X = x,
            ObjectiveValue = bestProfit,
            LogPath = options.OutputFile ?? ""
        };
    }

    private static SolveResult Fail(string msg, SolverOptions opt)
    {
        Logger.WriteLines(opt.OutputFile, new[] { $"[{nameof(KnapsackSolver)}] {msg}" });
        return new SolveResult { Status = SolveStatus.NotSolved };
    }

    private static string Fmt(double v) => v.ToString("0.000");
}


