using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Integer.BranchAndBound;
using Solver.Integer.CuttingPlane;
using Solver.Simplex.Primal;
using Solver.Simplex.Revised;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

Console.OutputEncoding = System.Text.Encoding.UTF8;

var solvers = new (string Key, ISolver Impl, string Label)[]
{
    ("2", new PrimalSimplexSolver(), "Primal Simplex (Tableau)"),
    ("3", new RevisedSimplexSolver(), "Revised Primal Simplex"),
    ("4", new BbSimplexSolver(), "Branch & Bound (Simplex-based)"),
    ("5", new CuttingPlaneSolver(), "Cutting Plane (Gomory)")
};

LpModel? currentModel = null;
string currentModelName = "—";

while (true)
{
    Console.WriteLine("=== LPR381 Solver (Console) ===");
    Console.WriteLine($"Current model: {currentModelName}");
    Console.WriteLine("[1] Load model from input file");
    Console.WriteLine("[6] Load sample toy model (built-in)");
    Console.WriteLine("[2] Solve with Primal Simplex (Tableau)");
    Console.WriteLine("[3] Solve with Revised Primal Simplex");
    Console.WriteLine("[4] Solve with Branch & Bound (Simplex-based)");
    Console.WriteLine("[5] Solve with Cutting Plane (Gomory)");
    Console.WriteLine("[0] Exit");
    Console.Write("Select: ");

    var choice = (Console.ReadLine() ?? "").Trim();
    if (choice == "0") break;

    try
    {
        switch (choice)
        {
            case "1":
                Console.Write("Enter input file path: ");
                var path = (Console.ReadLine() ?? "").Trim('"', ' ');
                currentModel = ParseInputFile(path);
                currentModelName = currentModel.Name ?? System.IO.Path.GetFileName(path);
                Console.WriteLine($"Loaded '{currentModelName}' with {currentModel.C.Length} vars and {currentModel.B.Length} constraints.\n");
                break;

            case "6":
                currentModel = LoadToyModel();
                currentModelName = currentModel.Name ?? "Toy";
                Console.WriteLine("Loaded built-in toy model.\n");
                break;

            case "2":
            case "3":
            case "4":
            case "5":
                if (currentModel is null)
                {
                    Console.WriteLine("No model loaded. Use [1] or [6] first.\n");
                    break;
                }
                var solver = solvers.First(s => s.Key == choice);
                Console.Write($"Output log path (e.g., outputs\\run.txt) or leave blank: ");
                var outPath = (Console.ReadLine() ?? "").Trim('"', ' ');
                var options = new SolverOptions { Verbose = true, OutputFile = string.IsNullOrWhiteSpace(outPath) ? null : outPath };

                Console.WriteLine($"\nRunning: {solver.Label}\n");
                var res = solver.Impl.Solve(currentModel, options);
                Console.WriteLine($"Status: {res.Status}");
                if (res.ObjectiveValue is double z) Console.WriteLine($"Objective: {F3(z)}");
                if (!string.IsNullOrWhiteSpace(outPath) && File.Exists(outPath))
                    Console.WriteLine($"Log written to: {outPath}");
                Console.WriteLine();
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

// Helpers

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

static LpModel ParseInputFile(string path)
{
    if (!File.Exists(path)) throw new FileNotFoundException("Input file not found.", path);
    var lines = File.ReadAllLines(path)
                    .Where(l => !string.IsNullOrWhiteSpace(l))
                    .Select(l => l.Trim())
                    .ToList();
    if (lines.Count == 0) throw new InvalidOperationException("Empty input file.");

    // Line 1: sense + objective coeffs (signed)
    var t0 = Tokenize(lines[0]);
    if (t0.Count < 2) throw new InvalidOperationException("Invalid objective line.");
    var sense = t0[0].ToLower() switch
    {
        "max" => Sense.Max,
        "min" => Sense.Min,
        _ => throw new InvalidOperationException("First token must be 'max' or 'min'.")
    };
    var c = t0.Skip(1).Select(ParseSigned).ToArray();
    var n = c.Length;

    // Constraints until sign restriction line
    var A = new List<double[]>();
    var rels = new List<RelOp>();
    var B = new List<double>();

    int i = 1;
    for (; i < lines.Count; i++)
    {
        var tokens = Tokenize(lines[i]);
        if (IsSignRestrictionLine(tokens)) break;
        if (tokens.Count < n + 2) throw new InvalidOperationException($"Constraint {i} has too few tokens.");

        var coeffs = new double[n];
        int k = 0;
        for (; k < n; k++) coeffs[k] = ParseSigned(tokens[k]);

        var rel = tokens[k++] switch
        {
            "<=" => RelOp.Le,
            ">=" => RelOp.Ge,
            "=" => RelOp.Eq,
            _ => throw new InvalidOperationException($"Bad relation in constraint {i}.")
        };
        if (k >= tokens.Count) throw new InvalidOperationException($"Missing RHS in constraint {i}.");
        var rhs = ParseSigned(tokens[k]);

        A.Add(coeffs);
        rels.Add(rel);
        B.Add(rhs);
    }

    if (i >= lines.Count) throw new InvalidOperationException("Missing sign restrictions line.");
    var signTokens = Tokenize(lines[i]);
    if (signTokens.Count != n)
        throw new InvalidOperationException($"Sign restriction count ({signTokens.Count}) != variable count ({n}).");
    var varSigns = signTokens.Select(ParseVarSign).ToArray();

    return new LpModel
    {
        Name = Path.GetFileNameWithoutExtension(path),
        ProblemSense = sense,
        C = c,
        A = A.ToArray(),
        RelOps = rels.ToArray(),
        B = B.ToArray(),
        VarSigns = varSigns
    };
}

static List<string> Tokenize(string line)
    => line.Split(' ', StringSplitOptions.RemoveEmptyEntries).ToList();

static bool IsSignRestrictionLine(List<string> tokens)
{
    if (tokens.Count == 0) return false;
    var ok = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { "+", "-", "urs", "int", "bin" };
    return tokens.All(t => ok.Contains(t));
}

static VarSign ParseVarSign(string t) => t.ToLower() switch
{
    "+" => VarSign.Plus,
    "-" => VarSign.Minus,
    "urs" => VarSign.Urs,
    "int" => VarSign.Int,
    "bin" => VarSign.Bin,
    _ => throw new InvalidOperationException($"Unknown sign restriction: {t}")
};

static double ParseSigned(string token)
{
    if (double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
        return v;

    if (token.Length > 1 && (token[0] == '+' || token[0] == '-'))
    {
        if (double.TryParse(token[1..], NumberStyles.Float, CultureInfo.InvariantCulture, out var mag))
            return token[0] == '-' ? -mag : mag;
    }
    throw new InvalidOperationException($"Bad numeric token: {token}");
}

static string F3(double v) => v.ToString("0.000", CultureInfo.InvariantCulture);

