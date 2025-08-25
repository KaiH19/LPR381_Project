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
using Solver.Core.IO;


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
                var fileParser = new ModelParser(); // obj for ModelParser class
                currentModel = ParseInputFile(path); // method to upload file is invoked here
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

static string F3(double v) => v.ToString("0.000", CultureInfo.InvariantCulture);

