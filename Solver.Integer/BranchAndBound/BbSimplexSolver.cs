using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Integer.BranchAndBound.Simplex;
using Solver.Integer.BranchAndBound.Dual;
using System.IO;

namespace Solver.Integer.BranchAndBound;

public sealed class FixedBbSimplexSolver : ISolver
{
    public string Name => "Fixed Branch & Bound (Simplex-based)";

    private readonly StringBuilder _logBuilder = new();
    private int _nodeCount = 0;
    private double _bestIntegerValue = double.NegativeInfinity;
    private double[]? _bestIntegerSolution;
    private readonly Stack<string> _branchPath = new();

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        options ??= new SolverOptions();
        _logBuilder.Clear();
        _nodeCount = 0;
        _bestIntegerValue = model.ProblemSense == Sense.Max ? double.NegativeInfinity : double.PositiveInfinity;
        _bestIntegerSolution = null;
        _branchPath.Clear();

        try
        {
            Console.ForegroundColor = ConsoleColor.Cyan;
            LogMessage("╔══════════════════════════════════════════════════════════════╗");
            LogMessage("║           BRANCH AND BOUND ALGORITHM                          ║");
            LogMessage("╚══════════════════════════════════════════════════════════════╝");
            Console.ResetColor();
            
            LogMessage($"Problem sense: {model.ProblemSense}");
            LogMessage($"Number of variables: {model.C.Length}");
            LogMessage($"Started at: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            LogMessage();

            // Create the LP relaxation model with proper bounds
            var relaxedModel = CreateLPRelaxationWithBounds(model);
            
            Console.ForegroundColor = ConsoleColor.Yellow;
            LogMessage("═══════════════════════════════════════════════════════════");
            LogMessage("STEP 1: SOLVING INITIAL LP RELAXATION");
            LogMessage("═══════════════════════════════════════════════════════════");
            Console.ResetColor();

            var lpResult = new PrimalSimplexSolver().Solve(relaxedModel, options);

            if (lpResult.Status != SolveStatus.Optimal)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                LogMessage($"✗ LP relaxation failed with status: {lpResult.Status}");
                Console.ResetColor();
                return CreateResult(lpResult.Status, options);
            }

            LogMessage($"✓ LP Relaxation Optimal: Z = {lpResult.ObjectiveValue?.ToString("F4") ?? "N/A"}");
            LogMessage("  Variables: " + FormatSolution(lpResult.X!, model.C.Length));

            // Validate LP solution bounds
            if (!ValidateSolutionBounds(lpResult.X!, model.C.Length))
            {
                Console.ForegroundColor = ConsoleColor.Red;
                LogMessage("✗ ERROR: LP relaxation solution violates bounds!");
                Console.ResetColor();
                return CreateResult(SolveStatus.NotSolved, options);
            }

            // Check if LP solution is already integer
            if (IsIntegerSolution(lpResult.X!))
            {
                Console.ForegroundColor = ConsoleColor.Green;
                LogMessage("★ LP SOLUTION IS ALREADY INTEGER! OPTIMAL SOLUTION FOUND ★");
                Console.ResetColor();
                _bestIntegerValue = lpResult.ObjectiveValue ?? 0.0;
                _bestIntegerSolution = lpResult.X != null ? (double[])lpResult.X.Clone() : new double[0];
                return CreateResult(SolveStatus.Optimal, _bestIntegerValue, _bestIntegerSolution, options);
            }

            Console.ForegroundColor = ConsoleColor.Yellow;
            LogMessage("\n═══════════════════════════════════════════════════════════");
            LogMessage("STEP 2: STARTING BRANCH AND BOUND TREE EXPLORATION");
            LogMessage("═══════════════════════════════════════════════════════════");
            Console.ResetColor();

            var result = ExecuteBranchAndBound(relaxedModel, options, lpResult.ObjectiveValue ?? 0.0);

            Console.ForegroundColor = ConsoleColor.Cyan;
            LogMessage("\n╔══════════════════════════════════════════════════════════════╗");
            LogMessage($"║ Algorithm completed: {_nodeCount} nodes explored              ║");
            LogMessage($"║ Finished at: {DateTime.Now:yyyy-MM-dd HH:mm:ss}             ║");
            LogMessage("╚══════════════════════════════════════════════════════════════╝");
            Console.ResetColor();

            if (!string.IsNullOrWhiteSpace(options.OutputFile))
            {
                WriteLogToFile(options.OutputFile);
                result = new SolveResult
                {
                    Status = result.Status,
                    X = result.X,
                    ObjectiveValue = result.ObjectiveValue ?? 0.0,
                    LogPath = options.OutputFile
                };
            }

            return result;
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            LogMessage($"✗ ERROR: {ex.Message}");
            Console.ResetColor();
            return CreateResult(SolveStatus.NotSolved, options);
        }
    }

    private SolveResult ExecuteBranchAndBound(LpModel baseModel, SolverOptions options, double lpBound)
    {
        var candidateQueue = new Queue<BranchNode>();
        candidateQueue.Enqueue(new BranchNode(baseModel, new List<BranchConstraint>(), 0, "ROOT"));

        var maxIterations = Math.Min(options.MaxIterations, 15); // Limit to 15 iterations max
        var iteration = 0;
        var startTime = DateTime.Now;
        var maxTime = TimeSpan.FromMinutes(1); // Reduce timeout to 1 minute

        LogMessage($"\nInitial LP bound: {lpBound:F4}");
        LogMessage($"Best integer value: {(_bestIntegerSolution == null ? "None" : _bestIntegerValue.ToString("F4"))}");

        while (candidateQueue.Count > 0 && iteration < maxIterations && (DateTime.Now - startTime) < maxTime)
        {
            var currentNode = candidateQueue.Dequeue();
            _nodeCount++;
            iteration++;

            // Display branch path
            Console.ForegroundColor = ConsoleColor.Cyan;
            LogMessage($"\n┌─── NODE {_nodeCount} ───────────────────────────────────────");
            LogMessage($"│ Branch Path: {currentNode.BranchPath}");
            LogMessage($"│ Depth: {currentNode.Depth} | Queue Size: {candidateQueue.Count}");
            Console.ResetColor();

            if (currentNode.Constraints.Count > 0)
            {
                LogMessage("│ Constraints:");
                foreach (var c in currentNode.Constraints)
                {
                    var op = c.Operator == "<=" ? "≤" : "≥";
                    LogMessage($"│   • x{c.VariableIndex + 1} {op} {c.Value}");
                }
            }

            // Determine which solver to use based on the last constraint
            string solverType = "Primal Simplex";
            if (currentNode.Constraints.Count > 0)
            {
                var lastConstraint = currentNode.Constraints.Last();
                if (lastConstraint.Operator == ">=")
                {
                    solverType = "Dual Simplex";
                }
            }

            Console.ForegroundColor = ConsoleColor.Magenta;
            LogMessage($"│ Solving with: {solverType}");
            Console.ResetColor();

            // Solve the current subproblem
            var solution = SolveSubproblem(currentNode.Model, currentNode.Constraints, options, solverType == "Dual Simplex");

            if (solution.Status != SolveStatus.Optimal)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                LogMessage($"│ ✗ Subproblem infeasible or unbounded");
                LogMessage("└────────────────────────────────────────────────");
                Console.ResetColor();
                continue;
            }

            if (!solution.ObjectiveValue.HasValue || solution.X == null)
            {
                Console.ForegroundColor = ConsoleColor.Red;
                LogMessage("│ ✗ Invalid solution from subproblem");
                LogMessage("└────────────────────────────────────────────────");
                Console.ResetColor();
                continue;
            }

            LogMessage($"│ Solution: Z = {solution.ObjectiveValue.Value:F4}");
            LogMessage($"│ Variables: {FormatSolution(solution.X, baseModel.C.Length)}");

            // Validate solution bounds
            if (!ValidateSolutionBounds(solution.X, baseModel.C.Length))
            {
                Console.ForegroundColor = ConsoleColor.Red;
                LogMessage("│ ✗ Solution violates bounds");
                LogMessage("└────────────────────────────────────────────────");
                Console.ResetColor();
                continue;
            }

            // Check bounding
            if (_bestIntegerSolution != null)
            {
                bool shouldPrune = baseModel.ProblemSense == Sense.Max
                    ? solution.ObjectiveValue.Value <= _bestIntegerValue + 1e-6
                    : solution.ObjectiveValue.Value >= _bestIntegerValue - 1e-6;

                if (shouldPrune)
                {
                    Console.ForegroundColor = ConsoleColor.Yellow;
                    LogMessage($"│ ✂ Node pruned by bound (Z = {solution.ObjectiveValue.Value:F4} vs Best = {_bestIntegerValue:F4})");
                    LogMessage("└────────────────────────────────────────────────");
                    Console.ResetColor();
                    continue;
                }
            }

            // Check if solution is integer
            var originalVars = solution.X.Take(baseModel.C.Length).ToArray();
            if (IsIntegerSolution(originalVars))
            {
                Console.ForegroundColor = ConsoleColor.Green;
                LogMessage("│ ★★★ INTEGER SOLUTION FOUND! ★★★");
                
                bool isBetter = _bestIntegerSolution == null ||
                    (baseModel.ProblemSense == Sense.Max ? solution.ObjectiveValue.Value > _bestIntegerValue + 1e-6
                                                        : solution.ObjectiveValue.Value < _bestIntegerValue - 1e-6);

                if (isBetter)
                {
                    _bestIntegerValue = solution.ObjectiveValue.Value;
                    _bestIntegerSolution = (double[])originalVars.Clone();
                    LogMessage($"│ ✓ NEW BEST INTEGER SOLUTION: Z = {_bestIntegerValue:F4}");
                    LogMessage($"│   Variables: {FormatIntegerSolution(_bestIntegerSolution)}");
                }
                else
                {
                    LogMessage($"│   Not better than current best ({_bestIntegerValue:F4})");
                }
                LogMessage("└────────────────────────────────────────────────");
                Console.ResetColor();
                continue;
            }

            // Branch on most fractional variable
            var branchVar = ChooseMostFractionalVariable(originalVars);
            if (branchVar == -1)
            {
                LogMessage("│ No fractional variables found");
                LogMessage("└────────────────────────────────────────────────");
                continue;
            }

            var fractionalValue = originalVars[branchVar];
            var floor = (int)Math.Floor(fractionalValue);
            var ceil = (int)Math.Ceiling(fractionalValue);

            Console.ForegroundColor = ConsoleColor.Yellow;
            LogMessage($"│");
            LogMessage($"│ 🌳 BRANCHING on x{branchVar + 1} = {fractionalValue:F4}");
            LogMessage($"│    ├─ Branch LEFT:  x{branchVar + 1} ≤ {floor}");
            LogMessage($"│    └─ Branch RIGHT: x{branchVar + 1} ≥ {ceil}");
            LogMessage("└────────────────────────────────────────────────");
            Console.ResetColor();

            // Create left branch (≤ floor)
            var leftConstraints = new List<BranchConstraint>(currentNode.Constraints)
            {
                new BranchConstraint(branchVar, "<=", floor)
            };
            var leftPath = $"{currentNode.BranchPath} → x{branchVar + 1}≤{floor}";
            candidateQueue.Enqueue(new BranchNode(baseModel, leftConstraints, currentNode.Depth + 1, leftPath));

            // Create right branch (≥ ceil)
            var rightConstraints = new List<BranchConstraint>(currentNode.Constraints)
            {
                new BranchConstraint(branchVar, ">=", ceil)
            };
            var rightPath = $"{currentNode.BranchPath} → x{branchVar + 1}≥{ceil}";
            candidateQueue.Enqueue(new BranchNode(baseModel, rightConstraints, currentNode.Depth + 1, rightPath));

            if (candidateQueue.Count > 25) // Reduced to 25 for faster termination
            {
                Console.ForegroundColor = ConsoleColor.Red;
                LogMessage("\n✗ Queue too large (>25), terminating to prevent excessive runtime");
                Console.ResetColor();
                break;
            }
        }

        if (iteration >= maxIterations)
        {
            Console.ForegroundColor = ConsoleColor.Yellow;
            LogMessage($"\n⚠ Maximum iterations reached ({maxIterations})");
            Console.ResetColor();
        }

        if ((DateTime.Now - startTime) >= maxTime)
        {
            Console.ForegroundColor = ConsoleColor.Yellow;
            LogMessage($"\n⚠ Timeout reached ({maxTime.TotalMinutes:F1} minutes)");
            Console.ResetColor();
        }

        if (_bestIntegerSolution != null)
        {
            Console.ForegroundColor = ConsoleColor.Green;
            LogMessage("\n╔══════════════════════════════════════════════════════════════╗");
            LogMessage("║                  OPTIMAL SOLUTION FOUND                       ║");
            LogMessage("╠══════════════════════════════════════════════════════════════╣");
            LogMessage($"║ Objective value: Z = {_bestIntegerValue:F4}");
            LogMessage($"║ Variables: {FormatIntegerSolution(_bestIntegerSolution)}");
            LogMessage("╚══════════════════════════════════════════════════════════════╝");
            Console.ResetColor();
            return CreateResult(SolveStatus.Optimal, _bestIntegerValue, _bestIntegerSolution, options);
        }
        else
        {
            Console.ForegroundColor = ConsoleColor.Red;
            LogMessage("\n✗ No integer solution found");
            Console.ResetColor();
            return CreateResult(SolveStatus.Infeasible, options);
        }
    }

    private SolveResult SolveSubproblem(LpModel baseModel, List<BranchConstraint> constraints, SolverOptions options, bool useDual)
    {
        try
        {
            var modifiedModel = CreateConstrainedModel(baseModel, constraints);

            if (useDual)
            {
                return new DualSimplexSolver().Solve(modifiedModel, options);
            }
            else
            {
                return new PrimalSimplexSolver().Solve(modifiedModel, options);
            }
        }
        catch (Exception ex)
        {
            LogMessage($"ERROR in SolveSubproblem: {ex.Message}");
            return new SolveResult { Status = SolveStatus.NotSolved };
        }
    }

    private string FormatSolution(double[] solution, int numVars)
    {
        var formatted = new StringBuilder();
        for (int i = 0; i < Math.Min(numVars, solution.Length); i++)
        {
            if (i > 0) formatted.Append(", ");
            formatted.Append($"x{i + 1}={solution[i]:F3}");
        }
        return formatted.ToString();
    }

    private string FormatIntegerSolution(double[] solution)
    {
        var formatted = new StringBuilder();
        for (int i = 0; i < solution.Length; i++)
        {
            if (i > 0) formatted.Append(", ");
            formatted.Append($"x{i + 1}={(int)Math.Round(solution[i])}");
        }
        return formatted.ToString();
    }

    private LpModel CreateLPRelaxationWithBounds(LpModel originalModel)
    {
        var originalRows = originalModel.A.Length;
        var numVars = originalModel.C.Length;

        // Add 2 * numVars constraints for bounds: xi >= 0 and xi <= 1
        var newRows = originalRows + 2 * numVars;
        var newA = new double[newRows][];
        var newRelOps = new RelOp[newRows];
        var newB = new double[newRows];

        // Copy original constraints
        for (int i = 0; i < originalRows; i++)
        {
            newA[i] = (double[])originalModel.A[i].Clone();
            newRelOps[i] = originalModel.RelOps[i];
            newB[i] = originalModel.B[i];
        }

        // Add lower bounds: xi >= 0
        for (int i = 0; i < numVars; i++)
        {
            var rowIndex = originalRows + i;
            newA[rowIndex] = new double[numVars];
            newA[rowIndex][i] = 1.0;
            newRelOps[rowIndex] = RelOp.Ge;
            newB[rowIndex] = 0.0;
        }

        // Add upper bounds: xi <= 1  
        for (int i = 0; i < numVars; i++)
        {
            var rowIndex = originalRows + numVars + i;
            newA[rowIndex] = new double[numVars];
            newA[rowIndex][i] = 1.0;
            newRelOps[rowIndex] = RelOp.Le;
            newB[rowIndex] = 1.0;
        }

        return new LpModel
        {
            ProblemSense = originalModel.ProblemSense,
            C = originalModel.C,
            A = newA,
            RelOps = newRelOps,
            B = newB,
            VarSigns = originalModel.VarSigns,
            Name = originalModel.Name
        };
    }

    private bool ValidateSolutionBounds(double[] solution, int numOriginalVars)
    {
        const double tolerance = 1e-6;
        for (int i = 0; i < numOriginalVars; i++)
        {
            if (solution[i] < -tolerance || solution[i] > 1.0 + tolerance)
            {
                return false;
            }
        }
        return true;
    }

    private LpModel CreateConstrainedModel(LpModel baseModel, List<BranchConstraint> constraints)
    {
        if (constraints.Count == 0)
            return baseModel;

        var newRows = baseModel.A.Length + constraints.Count;
        var newA = new double[newRows][];
        var newRelOps = new RelOp[newRows];
        var newB = new double[newRows];

        // Copy existing constraints
        for (int i = 0; i < baseModel.A.Length; i++)
        {
            newA[i] = (double[])baseModel.A[i].Clone();
            newRelOps[i] = baseModel.RelOps[i];
            newB[i] = baseModel.B[i];
        }

        // Add branch constraints
        for (int i = 0; i < constraints.Count; i++)
        {
            var constraint = constraints[i];
            var constraintRow = baseModel.A.Length + i;

            newA[constraintRow] = new double[baseModel.C.Length];
            newA[constraintRow][constraint.VariableIndex] = 1.0;

            newRelOps[constraintRow] = constraint.Operator == "<=" ? RelOp.Le : RelOp.Ge;
            newB[constraintRow] = constraint.Value;
        }

        return new LpModel
        {
            ProblemSense = baseModel.ProblemSense,
            C = baseModel.C,
            A = newA,
            RelOps = newRelOps,
            B = newB,
            VarSigns = baseModel.VarSigns,
            Name = baseModel.Name
        };
    }

    private bool IsIntegerSolution(double[] solution)
    {
        const double tolerance = 1e-6;
        return solution.All(x => Math.Abs(x - Math.Round(x)) < tolerance);
    }

    private int ChooseMostFractionalVariable(double[] solution)
    {
        int bestVar = -1;
        double maxFractionalPart = 0;

        for (int i = 0; i < solution.Length; i++)
        {
            double fractionalPart = Math.Abs(solution[i] - Math.Round(solution[i]));
            if (fractionalPart > maxFractionalPart && fractionalPart > 1e-6)
            {
                maxFractionalPart = fractionalPart;
                bestVar = i;
            }
        }

        return bestVar;
    }

    private void LogMessage(string message = "")
    {
        _logBuilder.AppendLine(message);
        Console.WriteLine(message);
    }

    private void WriteLogToFile(string filePath)
    {
        try
        {
            // Strip ANSI color codes for file output
            var cleanLog = System.Text.RegularExpressions.Regex.Replace(_logBuilder.ToString(), @"\x1B\[[^@-~]*[@-~]", "");
            File.WriteAllText(filePath, cleanLog);
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Warning: Could not write log to file: {ex.Message}");
        }
    }

    private SolveResult CreateResult(SolveStatus status, SolverOptions options)
    {
        return new SolveResult
        {
            Status = status,
            ObjectiveValue = null,
            X = null,
            LogPath = options.OutputFile ?? string.Empty
        };
    }

    private SolveResult CreateResult(SolveStatus status, double? objectiveValue, double[]? solution, SolverOptions options)
    {
        return new SolveResult
        {
            Status = status,
            ObjectiveValue = objectiveValue,
            X = solution,
            LogPath = options.OutputFile ?? string.Empty
        };
    }
}

internal class BranchNode
{
    public LpModel Model { get; }
    public List<BranchConstraint> Constraints { get; }
    public int Depth { get; }
    public string BranchPath { get; }

    public BranchNode(LpModel model, List<BranchConstraint> constraints, int depth, string branchPath)
    {
        Model = model;
        Constraints = constraints;
        Depth = depth;
        BranchPath = branchPath;
    }
}

internal class BranchConstraint
{
    public int VariableIndex { get; }
    public string Operator { get; }
    public double Value { get; }

    public BranchConstraint(int variableIndex, string op, double value)
    {
        VariableIndex = variableIndex;
        Operator = op;
        Value = value;
    }
}

