using System;
using System.Linq;
using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Integer.CuttingPlane.Models;
using Solver.Integer.CuttingPlane.Views;
using Solver.Simplex.Primal;
using System.IO; // Added for StringWriter
using System.Text; // Added for StringBuilder

namespace Solver.Integer.CuttingPlane.Controllers;

/// <summary>
/// Controller that orchestrates the cutting plane algorithm
/// </summary>
public class CuttingPlaneController
{
    private readonly CuttingPlaneView _view;
    private readonly PrimalSimplexSolver _simplexSolver;
    private readonly StringBuilder _logBuilder = new(); // Added for file output
    
    public CuttingPlaneController()
    {
        _view = new CuttingPlaneView();
        _simplexSolver = new PrimalSimplexSolver();
    }
    
    /// <summary>
    /// Main method to solve the integer programming problem using cutting planes
    /// </summary>
    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        options ??= new SolverOptions();
        _logBuilder.Clear(); // Clear previous log
        
        _view.ShowHeader();
        LogMessage("=== CUTTING PLANE METHOD (GOMORY) ===");
        LogMessage();

        // Validate integer variables
        if (!model.VarSigns.Any(sign => sign == VarSign.Int || sign == VarSign.Bin))
        {
            _view.ShowError("No integer variables found in the model.");
            LogMessage("ERROR: No integer variables found in the model.");
            return CreateResult(SolveStatus.NotSolved);
        }

        // Create and initialize the model
        var cuttingPlaneModel = new CuttingPlaneModel(model);

        // Process 1: Solve initial LP relaxation
        _view.ShowLPRelaxationStep();
        LogMessage("Process 1 - Relaxation IP");
        LogMessage("------------------------");
        
        var lpResult = _simplexSolver.Solve(cuttingPlaneModel.CurrentModel, new SolverOptions { Verbose = false });

        if (lpResult.Status != SolveStatus.Optimal)
        {
            _view.ShowLPRelaxationFailed(lpResult.Status);
            LogMessage($"LP relaxation failed with status: {lpResult.Status}");
            return CreateResult(lpResult.Status);
        }

        // Update the model with the initial solution
        cuttingPlaneModel.CurrentSolution = lpResult;
        _view.ShowInitialSolution(lpResult, model.VarSigns);
        LogInitialSolution(lpResult, model.VarSigns);

        // Check if already integer
        if (cuttingPlaneModel.CheckIntegerSolution())
        {
            _view.ShowAlreadyIntegerSolution();
            LogMessage("Solution is already integer-feasible!");
            
            return CreateResultWithFileOutput(SolveStatus.Optimal, 
                cuttingPlaneModel.GetOriginalVariables(), 
                lpResult.ObjectiveValue!.Value,
                options);
        }

        // Process 2: Iteratively add cutting planes
        _view.ShowCuttingPlanesStep();
        LogMessage("Process 2 - Cutting Plane");
        LogMessage("------------------------");

        while (cuttingPlaneModel.IterationCount < options.MaxIterations)
        {
            // Generate Gomory cut
            var cut = GenerateGomoryCut(cuttingPlaneModel);
            if (cut == null)
            {
                _view.ShowNoMoreCuts();
                LogMessage("No more cuts can be generated.");
                break;
            }

            _view.ShowCutIteration(cuttingPlaneModel.IterationCount + 1);
            LogMessage($"\n--- CUT {cuttingPlaneModel.IterationCount + 1}: GOMORY CUT ---");
            
            _view.ShowGeneratedCut(cut);
            LogMessage($"Generated cut: {FormatCut(cut)}");

            // Add cut to model
            cuttingPlaneModel.AddCut(cut);

            // Solve new LP with cut - suppress simplex output during cutting plane iterations
            var newResult = SolveWithSuppressedOutput(cuttingPlaneModel.CurrentModel);

            if (newResult.Status != SolveStatus.Optimal)
            {
                _view.ShowProblemInfeasible(newResult.Status);
                LogMessage($"Problem became {newResult.Status} after adding cut.");
                break;
            }

            // Update the model with the new solution
            cuttingPlaneModel.CurrentSolution = newResult;
            
            // Show the essential information for this cut iteration
            ShowCutIterationSummary(cuttingPlaneModel.CurrentModel, newResult, cut);
            LogCutIterationSummary(cuttingPlaneModel.CurrentModel, newResult, cut);

            // Check if integer solution found
            if (cuttingPlaneModel.CheckIntegerSolution())
            {
                _view.ShowIntegerSolutionFound();
                LogMessage("Integer solution found!");
                
                return CreateResultWithFileOutput(SolveStatus.Optimal, 
                    cuttingPlaneModel.GetOriginalVariables(), 
                    newResult.ObjectiveValue!.Value,
                    options);
            }
        }

        _view.ShowAlgorithmTermination(cuttingPlaneModel.IterationCount);
        LogMessage($"\nAlgorithm terminated after {cuttingPlaneModel.IterationCount} iterations.");
        
        // Return best solution found
        if (cuttingPlaneModel.CheckIntegerSolution())
        {
            return CreateResultWithFileOutput(SolveStatus.Optimal, 
                cuttingPlaneModel.GetOriginalVariables(), 
                cuttingPlaneModel.CurrentSolution!.ObjectiveValue!.Value,
                options);
        }
        else
        {
            return CreateResultWithFileOutput(SolveStatus.Feasible, 
                cuttingPlaneModel.GetOriginalVariables(), 
                cuttingPlaneModel.CurrentSolution!.ObjectiveValue!.Value,
                options);
        }
    }
    
    /// <summary>
    /// Generates a Gomory cut from the current fractional solution
    /// </summary>
    private GomoryCut? GenerateGomoryCut(CuttingPlaneModel model)
    {
        const double INTEGER_TOLERANCE = 1e-9;
        
        // Find most fractional integer variable
        int mostFractionalIndex = -1;
        double maxFractionalPart = 0;

        for (int i = 0; i < model.CurrentSolution!.X!.Length && i < model.OriginalModel.VarSigns.Length; i++)
        {
            if (model.OriginalModel.VarSigns[i] == VarSign.Int || model.OriginalModel.VarSigns[i] == VarSign.Bin)
            {
                double fractionalPart = Math.Abs(model.CurrentSolution.X[i] - Math.Round(model.CurrentSolution.X[i]));
                if (fractionalPart > maxFractionalPart && fractionalPart > INTEGER_TOLERANCE)
                {
                    maxFractionalPart = fractionalPart;
                    mostFractionalIndex = i;
                }
            }
        }

        if (mostFractionalIndex == -1)
            return null;

        _view.ShowMostFractionalVariable(mostFractionalIndex, model.CurrentSolution.X[mostFractionalIndex]);

        // Generate simple Gomory cut: x_i <= floor(x_i)
        var cutCoefficients = new double[model.OriginalModel.C.Length];
        cutCoefficients[mostFractionalIndex] = 1.0;
        double cutRhs = Math.Floor(model.CurrentSolution.X[mostFractionalIndex]);

        return new GomoryCut
        {
            Coefficients = cutCoefficients,
            RightHandSide = cutRhs,
            VariableIndex = mostFractionalIndex,
            CutType = "Simple Rounding"
        };
    }
    
    /// <summary>
    /// Creates a result object with optional file output
    /// </summary>
    private SolveResult CreateResultWithFileOutput(SolveStatus status, double[] solution, double objectiveValue, SolverOptions options)
    {
        // Write log to file if specified
        if (!string.IsNullOrWhiteSpace(options.OutputFile))
        {
            WriteLogToFile(options.OutputFile);
            return new SolveResult 
            { 
                Status = status,
                X = solution, 
                ObjectiveValue = objectiveValue,
                LogPath = options.OutputFile 
            };
        }
        
        return CreateResult(status, solution, objectiveValue);
    }

    /// <summary>
    /// Creates a result object
    /// </summary>
    private SolveResult CreateResult(SolveStatus status, double[]? solution = null, double? objectiveValue = null)
    {
        return new SolveResult
        {
            Status = status,
            X = solution,
            ObjectiveValue = objectiveValue
        };
    }

    /// <summary>
    /// Solves an LP problem with suppressed output during cutting plane iterations
    /// </summary>
    private SolveResult SolveWithSuppressedOutput(LpModel model)
    {
        // Temporarily redirect console output to suppress simplex solver output
        var originalOut = Console.Out;
        var stringWriter = new StringWriter();
        Console.SetOut(stringWriter);
        
        try
        {
            // Solve the problem (output will be captured but not displayed)
            var result = _simplexSolver.Solve(model, new SolverOptions { Verbose = false });
            return result;
        }
        finally
        {
            // Restore console output
            Console.SetOut(originalOut);
            stringWriter.Dispose();
        }
    }

    private void ShowCutIterationSummary(LpModel model, SolveResult result, GomoryCut cut)
    {
        // Only show the optimal solution and algorithm completion details
        // Remove canonical form to avoid duplication
        _view.ShowOptimalSolution(result.X!, result.ObjectiveValue!.Value);
        _view.ShowAlgorithmCompletionDetails(model.VarSigns, result.X!, result.ObjectiveValue!.Value);
    }

    private void LogMessage(string message = "")
    {
        _logBuilder.AppendLine(message);
        Console.WriteLine(message); // Also print to console for immediate feedback
    }

    private void LogInitialSolution(SolveResult result, VarSign[] varSigns)
    {
        LogMessage("Initial LP Relaxation Solution:");
        LogMessage($"Objective Value: {result.ObjectiveValue}");
        LogMessage("Variables:");
        for (int i = 0; i < result.X!.Length; i++)
        {
            LogMessage($"x{i}: {result.X[i]} (Sign: {varSigns[i]})");
        }
        LogMessage();
    }

    private string FormatCut(GomoryCut cut)
    {
        var sb = new StringBuilder();
        sb.Append($"{cut.VariableIndex}: ");
        for (int i = 0; i < cut.Coefficients.Length; i++)
        {
            if (cut.Coefficients[i] != 0)
            {
                sb.Append($"{cut.Coefficients[i]}x{i}");
                if (i < cut.Coefficients.Length - 1)
                {
                    sb.Append(" + ");
                }
            }
        }
        sb.Append($" <= {cut.RightHandSide}");
        return sb.ToString();
    }

    private void LogCutIterationSummary(LpModel model, SolveResult result, GomoryCut cut)
    {
        LogMessage($"Cut {cut.CutType} (Variable: x{cut.VariableIndex}):");
        LogMessage($"Objective Value: {result.ObjectiveValue}");
        LogMessage("Variables:");
        for (int i = 0; i < result.X!.Length; i++)
        {
            LogMessage($"x{i}: {result.X[i]} (Sign: {model.VarSigns[i]})");
        }
        LogMessage();
    }

    private void WriteLogToFile(string filePath)
    {
        File.WriteAllText(filePath, _logBuilder.ToString());
        LogMessage($"Log written to {filePath}");
    }
}
