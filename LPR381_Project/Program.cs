using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Globalization;
using Solver.Core.IO;
using Solver.Core.Models;
using Solver.Core.Interfaces;
using Solver.Simplex.Primal;
using Solver.Simplex.Revised;
using Solver.Integer.BranchAndBound;
using Solver.Integer.CuttingPlane;
using Solver.Knapsack;
using Solver.Sensitivity;   // SensitivityAnalysis, ModelEdit, DualBuilder (in this project)

Console.OutputEncoding = Encoding.UTF8;

/* Register available solvers (key → label → implementation) */
var solvers = new (string Key, ISolver Impl, string Label)[]
{
    ("2", new PrimalSimplexSolver(),  "Primal Simplex (Tableau)"),
    ("3", new RevisedSimplexSolver(), "Revised Primal Simplex"),
    ("4", new BbSimplexSolver(),      "Branch & Bound (Simplex-based)"),
    ("5", new CuttingPlaneSolver(),   "Cutting Plane (Gomory)"),
    ("7", new KnapsackSolver(),       "Knapsack (0/1) Branch & Bound")
};

LpModel? currentModel = null;
string currentModelName = "—";

// Keep the most recent Simplex result so Sensitivity can use the basis/duals
SolveResult? lastSimplexResult = null;
string? lastSimplexLabel = null;
ISolver? lastSimplexSolver = null;

// cache the most recent dual we built so we can export it in [13]
LpModel? lastDualModel = null;

while (true)
{
    Console.WriteLine("=== LPR381 Solver (Console) ===");
    Console.WriteLine($"Current model: {currentModelName}");
    Console.WriteLine("[1]  Load model from input file");
    Console.WriteLine("[6]  Load sample toy model (built-in)");
    Console.WriteLine("[2]  Solve with Primal Simplex (Tableau)");
    Console.WriteLine("[3]  Solve with Revised Primal Simplex");
    Console.WriteLine("[4]  Solve with Branch & Bound (Simplex-based)");
    Console.WriteLine("[5]  Solve with Cutting Plane (Gomory)");
    Console.WriteLine("[7]  Solve Knapsack (0/1) B&B");
    Console.WriteLine("---- Sensitivity ----");
    Console.WriteLine("[8]  Show variable sensitivity (reduced costs & allowable Δc)");
    Console.WriteLine("[9]  Show RHS ranges (allowable Δb)");
    Console.WriteLine("[10] Apply Δc to a variable and re-solve (uses last Simplex)");
    Console.WriteLine("[11] Apply Δb to a constraint and re-solve (uses last Simplex)");
    Console.WriteLine("[14] Show shadow prices (dual variables)");
    Console.WriteLine("[15] Add new variable (column) and re-solve (Simplex)");
    Console.WriteLine("[16] Add new constraint (row) and re-solve (Simplex)");
    Console.WriteLine("---- Duality ----");
    Console.WriteLine("[12] Build & solve Dual (check strong/weak duality)");
    Console.WriteLine("[13] Export last-built Dual model to file");
    Console.WriteLine("[0]  Exit");
    Console.Write("Select: ");

    var choice = (Console.ReadLine() ?? "").Trim();
    if (choice == "0") break;

    try
    {
        switch (choice)
        {
            case "1":
                Console.Write("Enter input file path: ");
                {
                    var input = (Console.ReadLine() ?? "").Trim('"', ' ');
                    var path = ResolvePath(input);
                    currentModel = ModelParser.ParseInputFile(path);
                    currentModelName = currentModel.Name ?? Path.GetFileName(path);
                    lastSimplexResult = null; lastSimplexLabel = null; lastSimplexSolver = null;
                    lastDualModel = null;
                    Console.WriteLine($"Loaded '{currentModelName}' with {currentModel.C.Length} vars and {currentModel.B.Length} constraints.\n");
                }
                break;

            case "6":
                currentModel = LoadToyModel();
                currentModelName = currentModel.Name ?? "Toy";
                lastSimplexResult = null; lastSimplexLabel = null; lastSimplexSolver = null;
                lastDualModel = null;
                Console.WriteLine("Loaded built-in toy model.\n");
                break;

            case "2":
            case "3":
            case "4":
            case "5":
            case "7":
                if (currentModel is null)
                {
                    Console.WriteLine("No model loaded. Use [1] or [6] first.\n");
                    break;
                }
                {
                    var solver = solvers.First(s => s.Key == choice);
                    Console.Write($"Output log path (e.g., outputs\\run.txt) or leave blank: ");
                    var outPath = (Console.ReadLine() ?? "").Trim('"', ' ');
                    var options = new SolverOptions
                    {
                        Verbose = true,
                        OutputFile = string.IsNullOrWhiteSpace(outPath) ? null : outPath
                    };

                    Console.WriteLine($"\nRunning: {solver.Label}\n");
                    var res = solver.Impl.Solve(currentModel, options);
                    Console.WriteLine($"Status: {res.Status}");
                    if (res.ObjectiveValue is double z) Console.WriteLine($"Objective: {F3(z)}");
                    if (!string.IsNullOrWhiteSpace(outPath) && File.Exists(outPath))
                        Console.WriteLine($"Log written to: {outPath}");
                    Console.WriteLine();

                    if (choice is "2" or "3")
                    {
                        lastSimplexResult = res;
                        lastSimplexLabel = solver.Label;
                        lastSimplexSolver = solver.Impl;
                    }
                    else
                    {
                        lastSimplexResult = null; lastSimplexLabel = null; lastSimplexSolver = null;
                    }
                }
                break;

            case "8": // Variable sensitivity
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    var ranges = SensitivityAnalysis.VariableRanges(currentModel!, lastSimplexResult!);
                    Console.WriteLine("\nVar  ReducedCost  AllowableDecrease  AllowableIncrease");
                    for (int j = 0; j < ranges.Length; j++)
                    {
                        var r = ranges[j];
                        Console.WriteLine($"{j + 1,3}  {F3(r.ReducedCost),11}  {F3(r.AllowableDecrease),17}  {FmtInf(r.AllowableIncrease),17}");
                    }
                    Console.WriteLine();
                }
                break;

            case "9": // RHS ranges
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    var ranges = SensitivityAnalysis.RhsRanges(currentModel!, lastSimplexResult!);
                    Console.WriteLine("\nCon  AllowableDecrease  AllowableIncrease");
                    for (int k = 0; k < ranges.Length; k++)
                    {
                        var r = ranges[k];
                        Console.WriteLine($"{k + 1,3}  {F3(r.AllowableDecrease),17}  {FmtInf(r.AllowableIncrease),17}");
                    }
                    Console.WriteLine();
                }
                break;

            case "10": // Apply Δc to variable j and re-solve
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    int j = ReadInt("Variable index (1-based): ", min: 1, max: currentModel!.C.Length) - 1;
                    double delta = ReadDouble("Δc (e.g., -0.5 or 1.2): ");
                    var changed = SensitivityAnalysis.ApplyCoeffChange(currentModel!, j, delta);
                    currentModel = changed;
                    currentModelName = (currentModelName ?? "Model") + $" [Δc@{j + 1}={F3(delta)}]";
                    Console.WriteLine($"Re-solving with last Simplex: {lastSimplexLabel} ...\n");
                    var res = lastSimplexSolver!.Solve(currentModel, new SolverOptions { Verbose = true });
                    lastSimplexResult = res;
                    Console.WriteLine($"Status: {res.Status}");
                    if (res.ObjectiveValue is double z2) Console.WriteLine($"Objective: {F3(z2)}");
                    Console.WriteLine();
                }
                break;

            case "11": // Apply Δb to constraint k and re-solve
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    int k = ReadInt("Constraint index (1-based): ", min: 1, max: currentModel!.B.Length) - 1;
                    double delta = ReadDouble("Δb (e.g., -1 or 2.5): ");
                    var changed = SensitivityAnalysis.ApplyRhsChange(currentModel!, k, delta);
                    currentModel = changed;
                    currentModelName = (currentModelName ?? "Model") + $" [Δb@{k + 1}={F3(delta)}]";
                    Console.WriteLine($"Re-solving with last Simplex: {lastSimplexLabel} ...\n");
                    var res = lastSimplexSolver!.Solve(currentModel, new SolverOptions { Verbose = true });
                    lastSimplexResult = res;
                    Console.WriteLine($"Status: {res.Status}");
                    if (res.ObjectiveValue is double z3) Console.WriteLine($"Objective: {F3(z3)}");
                    Console.WriteLine();
                }
                break;

            case "14": // Shadow prices
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    var duals = lastSimplexResult!.Duals ?? Array.Empty<double>();
                    Console.WriteLine("\nConstraint  ShadowPrice");
                    for (int i = 0; i < duals.Length; i++)
                        Console.WriteLine($"{i + 1,10}  {F3(duals[i])}");
                    Console.WriteLine();
                }
                break;

            case "15": // Add variable (column)
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    int mrows = currentModel!.B.Length;
                    Console.WriteLine($"Enter {mrows} coefficients for the new column (space-separated):");
                    var toks = (Console.ReadLine() ?? "").Split(' ', StringSplitOptions.RemoveEmptyEntries);
                    if (toks.Length != mrows) { Console.WriteLine("Count mismatch.\n"); break; }
                    var col = toks.Select(t => double.Parse(t, CultureInfo.InvariantCulture)).ToArray();

                    double cj = ReadDouble("Objective coef c_j: ");
                    var newModel = ModelEdit.AddVariable(currentModel!, col, cj, VarSign.Plus);
                    currentModel = newModel;
                    currentModelName = (currentModelName ?? "Model") + " [+var]";

                    Console.WriteLine($"Re-solving with last Simplex: {lastSimplexLabel} ...\n");
                    var res = lastSimplexSolver!.Solve(currentModel, new SolverOptions { Verbose = true });
                    lastSimplexResult = res;
                    Console.WriteLine($"Status: {res.Status}");
                    if (res.ObjectiveValue is double z) Console.WriteLine($"Objective: {F3(z)}");
                    Console.WriteLine();
                }
                break;

            case "16": // Add constraint (row)
                if (!EnsureSensitivityReady(currentModel, lastSimplexResult, lastSimplexLabel)) break;
                {
                    int nvars = currentModel!.C.Length;
                    Console.WriteLine($"Enter {nvars} coefficients for the new row (space-separated):");
                    var toks = (Console.ReadLine() ?? "").Split(' ', StringSplitOptions.RemoveEmptyEntries);
                    if (toks.Length != nvars) { Console.WriteLine("Count mismatch.\n"); break; }
                    var row = toks.Select(t => double.Parse(t, CultureInfo.InvariantCulture)).ToArray();

                    Console.Write("Relation (<=, >=, =): ");
                    var relTok = (Console.ReadLine() ?? "").Trim();
                    var rel = relTok switch { "<=" => RelOp.Le, ">=" => RelOp.Ge, "=" => RelOp.Eq, _ => RelOp.Le };

                    double rhs = ReadDouble("RHS b: ");

                    var newModel = ModelEdit.AddConstraint(currentModel!, row, rel, rhs);
                    currentModel = newModel;
                    currentModelName = (currentModelName ?? "Model") + " [+con]";

                    Console.WriteLine($"Re-solving with last Simplex: {lastSimplexLabel} ...\n");
                    var res = lastSimplexSolver!.Solve(currentModel, new SolverOptions { Verbose = true });
                    lastSimplexResult = res;
                    Console.WriteLine($"Status: {res.Status}");
                    if (res.ObjectiveValue is double z) Console.WriteLine($"Objective: {F3(z)}");
                    Console.WriteLine();
                }
                break;

            case "12": // Build & solve dual, mapping + strong duality check + prompt to export
                if (currentModel is null)
                {
                    Console.WriteLine("No model loaded. Use [1] or [6] first.\n");
                    break;
                }
                {
                    Console.WriteLine("\nBuilding Dual model from current primal...");
                    var dual = DualBuilder.BuildDual(currentModel);
                    lastDualModel = dual; // cache for [13]
                    Console.WriteLine($"Dual built: {dual.Name ?? "Dual"} with {dual.C.Length} vars and {dual.B.Length} constraints.");

                    // Pretty mapping (assumes primal Max, Ax<=b, x>=0)
                    PrintPrimalDualMapping(currentModel, dual);

                    // Solve primal if needed
                    double? primalZ = null;
                    var primalStatus = SolveStatus.NotSolved;
                    if (lastSimplexResult is not null && lastSimplexResult.Status == SolveStatus.Optimal)
                    {
                        primalZ = lastSimplexResult.ObjectiveValue;
                        primalStatus = lastSimplexResult.Status;
                        Console.WriteLine($"\nUsing last primal ({lastSimplexLabel}) result: z_P = {F3(primalZ ?? 0)}");
                    }
                    else
                    {
                        Console.WriteLine("\nSolving primal with Revised Simplex for comparison...");
                        var pr = new RevisedSimplexSolver().Solve(currentModel, new SolverOptions { Verbose = true });
                        primalZ = pr.ObjectiveValue;
                        primalStatus = pr.Status;
                        Console.WriteLine($"Primal status: {pr.Status}, z_P = {(pr.ObjectiveValue is double zp ? F3(zp) : "n/a")}");
                    }

                    // Solve dual with Revised Simplex
                    Console.WriteLine("\nSolving dual with Revised Simplex...");
                    var dr = new RevisedSimplexSolver().Solve(dual, new SolverOptions { Verbose = true });
                    Console.WriteLine($"Dual status: {dr.Status}, z_D = {(dr.ObjectiveValue is double zd ? F3(zd) : "n/a")}");

                    // Check duality
                    if (primalStatus == SolveStatus.Optimal && dr.Status == SolveStatus.Optimal
                        && primalZ is double zp2 && dr.ObjectiveValue is double zd2)
                    {
                        var gap = Math.Abs(zp2 - zd2);
                        Console.WriteLine($"|z_P - z_D| = {F3(gap)}");
                        if (gap <= 1e-6)
                            Console.WriteLine("✅ Strong duality holds (objectives match within tolerance).");
                        else
                            Console.WriteLine("⚠️ Objectives differ: investigate formulation/sign conventions.");
                    }
                    else
                    {
                        Console.WriteLine("⚠️ Strong duality cannot be verified because one or both problems are not optimal/feasible.");
                    }
                    Console.WriteLine();

                    // Offer quick export
                    Console.Write("Export this Dual model now? (y/n): ");
                    var yn = (Console.ReadLine() ?? "").Trim().ToLowerInvariant();
                    if (yn == "y" || yn == "yes")
                    {
                        var defaultPath = $"outputs\\{(currentModelName ?? "model")}_dual.txt";
                        Console.Write($"Save path [{defaultPath}]: ");
                        var p = Console.ReadLine();
                        var savePath = string.IsNullOrWhiteSpace(p) ? defaultPath : p!.Trim('"', ' ');
                        WriteModelToFile(dual, savePath);
                        Console.WriteLine($"Dual saved: {savePath}\n");
                    }
                }
                break;

            case "13": // Export last-built dual
                if (lastDualModel is null)
                {
                    Console.WriteLine("No dual model cached. Use [12] to build it first.\n");
                    break;
                }
                {
                    var defaultPath = $"outputs\\{(currentModelName ?? "model")}_dual.txt";
                    Console.Write($"Save path [{defaultPath}]: ");
                    var p = Console.ReadLine();
                    var savePath = string.IsNullOrWhiteSpace(p) ? defaultPath : p!.Trim('"', ' ');
                    WriteModelToFile(lastDualModel, savePath);
                    Console.WriteLine($"Dual saved: {savePath}\n");
                }
                break;

            default:
                Console.WriteLine("Invalid choice.\n");
                break;
        }
    }
    catch (Exception ex)
    {
        Console.WriteLine($"Error: {ex.Message}\n");
    }
}

/* -------------------- Helpers -------------------- */

static bool EnsureSensitivityReady(LpModel? model, SolveResult? last, string? label)
{
    if (model is null)
    {
        Console.WriteLine("No model loaded. Use [1] or [6] first.\n");
        return false;
    }
    if (last is null || last.Status != SolveStatus.Optimal)
    {
        Console.WriteLine("Run a Simplex solver ([2] or [3]) to optimality first; sensitivity needs the basis/duals.\n");
        return false;
    }
    return true;
}

static int ReadInt(string prompt, int min, int max)
{
    while (true)
    {
        Console.Write(prompt);
        var s = Console.ReadLine();
        if (int.TryParse(s, out var v) && v >= min && v <= max) return v;
        Console.WriteLine($"Please enter an integer between {min} and {max}.");
    }
}

static double ReadDouble(string prompt)
{
    while (true)
    {
        Console.Write(prompt);
        var s = Console.ReadLine();
        if (double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v)) return v;
        Console.WriteLine("Please enter a number (e.g., -1.5 or 2.0). Use '.' as decimal separator.");
    }
}

static string ResolvePath(string input)
{
    if (Path.IsPathRooted(input)) return input;

    // 1) bin/Debug/net8.0
    var cand1 = Path.Combine(AppContext.BaseDirectory, input);
    if (File.Exists(cand1)) return cand1;

    // 2) project directory (two levels up from bin)
    var projDir = Directory.GetParent(AppContext.BaseDirectory)!.Parent!.Parent!.FullName;
    var cand2 = Path.Combine(projDir, input);
    if (File.Exists(cand2)) return cand2;

    // fallback to original (will throw later if not found)
    return input;
}

static LpModel LoadToyModel()
{
    // Max z = 3x1 + 2x2
    // s.t.  x1 + x2 <= 4
    //       x1       <= 2
    //       x2       <= 3
    // x >= 0
    return new LpModel
    {
        Name = "Toy",
        ProblemSense = Sense.Max,
        C = new[] { 3.0, 2.0 },
        A = new[]
        {
            new[] { 1.0, 1.0 },
            new[] { 1.0, 0.0 },
            new[] { 0.0, 1.0 }
        },
        RelOps = new[] { RelOp.Le, RelOp.Le, RelOp.Le },
        B = new[] { 4.0, 2.0, 3.0 },
        VarSigns = new[] { VarSign.Plus, VarSign.Plus }
    };
}

static string F3(double v) => v.ToString("0.000", CultureInfo.InvariantCulture);
static string FmtInf(double v) => double.IsPositiveInfinity(v) ? "inf" : F3(v);

/* ---------- Pretty mapping + serialization for Dual ---------- */

static void PrintPrimalDualMapping(LpModel primal, LpModel dual)
{
    int m = primal.B.Length;
    int n = primal.C.Length;

    Console.WriteLine("\nPrimal ↔ Dual mapping (assuming Max, Ax<=b, x>=0):");
    Console.WriteLine("- Dual variables y_i correspond to primal constraints a_i.");
    Console.WriteLine("- Dual constraints (≥) correspond to primal variables x_j.\n");

    Console.WriteLine("Primal constraints  a_i  →  Dual variables  y_i (y_i ≥ 0):");
    for (int i = 1; i <= m; i++)
        Console.WriteLine($"  a{i}  →  y{i}");

    Console.WriteLine("\nPrimal variables  x_j  →  Dual constraints (A^T y ≥ c):");
    for (int j = 1; j <= n; j++)
        Console.WriteLine($"  x{j}  →  dual constraint #{j}");

    Console.WriteLine();
}

static void WriteModelToFile(LpModel model, string path)
{
    var txt = SerializeModel(model);
    var dir = Path.GetDirectoryName(path);
    if (!string.IsNullOrEmpty(dir) && !Directory.Exists(dir)) Directory.CreateDirectory(dir!);
    File.WriteAllText(path, txt);
}

static string SerializeModel(LpModel model)
{
    // Same flat text format your parser reads:
    // Line 1: sense + c[]
    // Next m lines: A[i,*] <rel> B[i]
    // Last line: sign restrictions for each var
    var sb = new StringBuilder();
    sb.Append(model.ProblemSense == Sense.Max ? "max" : "min");
    sb.Append(' ');
    sb.AppendLine(string.Join(' ', model.C.Select(s => Signed(s))));

    for (int i = 0; i < model.B.Length; i++)
    {
        sb.Append(string.Join(' ', model.A[i].Select(Signed)));
        sb.Append(' ');
        sb.Append(Rel(model.RelOps[i]));
        sb.Append(' ');
        sb.AppendLine(Signed(model.B[i]));
    }

    sb.AppendLine(string.Join(' ', model.VarSigns.Select(SignTok)));
    return sb.ToString();

    static string Signed(double v)
    {
        var s = v.ToString("0.#######", CultureInfo.InvariantCulture);
        if (!s.StartsWith("-")) s = "+" + s;
        return s;
    }
    static string Rel(RelOp r) => r switch
    {
        RelOp.Le => "<=",
        RelOp.Ge => ">=",
        _ => "="
    };
    static string SignTok(VarSign v) => v switch
    {
        VarSign.Plus => "+",
        VarSign.Minus => "-",
        VarSign.Urs => "urs",
        VarSign.Int => "int",
        VarSign.Bin => "bin",
        _ => "+"
    };
}
