using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Simplex.Common;

namespace Solver.Simplex.Primal;


public sealed class PrimalSimplexSolver : ISolver
{
    public string Name => "Primal Simplex (Tableau)";

    private readonly StringBuilder _logBuilder = new();
    private int _iterationCount = 0;

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        options ??= new SolverOptions();
        _logBuilder.Clear();
        _iterationCount = 0;

        try
        {
            LogMessage("=== PRIMAL SIMPLEX METHOD (TABLEAU) ===");
            LogMessage($"Started at: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            LogMessage();

            // Step 1: Convert to canonical form
            LogMessage("STEP 1: Converting to Canonical Form");
            LogMessage(new string('-', 50));
            
            var canonicalForm = CanonicalFormConverter.Convert(model);
            LogMessage(canonicalForm.FormattedProblem);

            // Step 2: Check for feasible initial solution
            if (!HasFeasibleInitialSolution(canonicalForm))
            {
                LogMessage("ERROR: No feasible initial basic solution found.");
                LogMessage("The problem requires Phase I of the Two-Phase method (not implemented in basic version).");
                return CreateResult(SolveStatus.Infeasible, options);
            }

            // Step 3: Create initial tableau
            LogMessage("STEP 2: Creating Initial Tableau");
            LogMessage(new string('-', 50));
            
            var tableau = CreateInitialTableau(canonicalForm);
            LogMessage(tableau.ToFormattedString(_iterationCount));

            // Step 4: Solve using simplex iterations
            LogMessage("STEP 3: Simplex Iterations");
            LogMessage(new string('-', 50));

            var result = SolveTableau(tableau, canonicalForm, options);
            
            LogMessage($"Algorithm completed in {_iterationCount} iterations.");
            LogMessage($"Finished at: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");

            // Write log to file if specified
            if (!string.IsNullOrWhiteSpace(options.OutputFile))
            {
                WriteLogToFile(options.OutputFile);
                result = new SolveResult 
                { 
                    Status = result.Status,
                    X = result.X,
                    ObjectiveValue = result.ObjectiveValue,
                    Basis = result.Basis,
                    ReducedCosts = result.ReducedCosts,
                    Duals = result.Duals,
                    LogPath = options.OutputFile 
                };
            }

            return result;
        }
        catch (Exception ex)
        {
            LogMessage($"ERROR: {ex.Message}");
            return CreateResult(SolveStatus.NotSolved, options);
        }
    }

    private bool HasFeasibleInitialSolution(CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        // Check if we have enough slack variables to form an initial basic feasible solution
        // This is a simplified check - a full implementation would include Phase I
        var rows = canonicalForm.A.GetLength(0);
        var cols = canonicalForm.A.GetLength(1);
        
        // All RHS values should be non-negative (handled by canonical form converter)
        for (int i = 0; i < canonicalForm.B.Length; i++)
        {
            if (canonicalForm.B[i] < -1e-10)
                return false;
        }

        // Check if we have identity matrix columns for slack variables
        int identityColumns = 0;
        for (int j = canonicalForm.OriginalVariables; j < cols; j++)
        {
            bool isIdentityColumn = false;
            int nonZeroRow = -1;
            
            for (int i = 0; i < rows; i++)
            {
                if (Math.Abs(canonicalForm.A[i, j] - 1.0) < 1e-10)
                {
                    if (nonZeroRow == -1)
                    {
                        nonZeroRow = i;
                        isIdentityColumn = true;
                    }
                    else
                    {
                        isIdentityColumn = false;
                        break;
                    }
                }
                else if (Math.Abs(canonicalForm.A[i, j]) > 1e-10)
                {
                    isIdentityColumn = false;
                    break;
                }
            }
            
            if (isIdentityColumn) identityColumns++;
        }

        return identityColumns >= rows;
    }

    private SimplexTableau CreateInitialTableau(CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        var rows = canonicalForm.A.GetLength(0);
        var cols = canonicalForm.A.GetLength(1);
        
        // Create tableau: [A | b]
        //                [c | 0] (objective row)
        var tableau = new double[rows + 1, cols + 1];
        
        // Copy constraint matrix and RHS
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                tableau[i, j] = canonicalForm.A[i, j];
            }
            tableau[i, cols] = canonicalForm.B[i]; // RHS
        }
        
        // Set up objective row
        for (int j = 0; j < cols; j++)
        {
            tableau[rows, j] = canonicalForm.IsMaximization ? -canonicalForm.C[j] : canonicalForm.C[j];
        }
        tableau[rows, cols] = 0; // Initial objective value

        // Find initial basis (identity columns)
        var basis = new int[rows];
        for (int i = 0; i < rows; i++)
        {
            basis[i] = -1; // Initially unassigned
            
            // Look for identity column starting from slack variables
            for (int j = canonicalForm.OriginalVariables; j < cols; j++)
            {
                bool isIdentityColumn = true;
                for (int k = 0; k < rows; k++)
                {
                    double expected = (k == i) ? 1.0 : 0.0;
                    if (Math.Abs(tableau[k, j] - expected) > 1e-10)
                    {
                        isIdentityColumn = false;
                        break;
                    }
                }
                
                if (isIdentityColumn)
                {
                    basis[i] = j;
                    break;
                }
            }
        }

        return new SimplexTableau(tableau, basis, canonicalForm.VariableNames, canonicalForm.IsMaximization);
    }

    private SolveResult SolveTableau(SimplexTableau tableau, CanonicalFormConverter.CanonicalForm canonicalForm, SolverOptions options)
    {
        while (_iterationCount < options.MaxIterations)
        {
            // Check optimality
            if (tableau.IsOptimal())
            {
                LogMessage("*** OPTIMAL SOLUTION FOUND ***");
                LogMessage();
                
                var (solution, objectiveValue) = tableau.GetBasicSolution();
                var reducedCosts = tableau.GetReducedCosts();
                var dualValues = tableau.GetDualValues(canonicalForm.OriginalConstraints);
                
                LogSolution(solution, objectiveValue, reducedCosts, dualValues, canonicalForm);
                
                // Extract original variables only
                var originalSolution = new double[canonicalForm.OriginalVariables];
                Array.Copy(solution, originalSolution, canonicalForm.OriginalVariables);
                
                return new SolveResult
                {
                    Status = SolveStatus.Optimal,
                    X = originalSolution,
                    ObjectiveValue = objectiveValue,
                    Basis = tableau.Basis,
                    ReducedCosts = reducedCosts,
                    Duals = dualValues
                };
            }

            // Find entering variable
            int enteringCol = tableau.FindEnteringVariable();
            if (enteringCol == -1)
            {
                LogMessage("ERROR: Could not find entering variable (algorithm error)");
                return CreateResult(SolveStatus.NotSolved, options);
            }

            LogMessage($"Entering variable: {tableau.Variables[enteringCol]}");

            // Check for unboundedness
            if (tableau.IsUnbounded(enteringCol))
            {
                LogMessage("*** PROBLEM IS UNBOUNDED ***");
                LogMessage($"The entering variable {tableau.Variables[enteringCol]} can increase indefinitely.");
                return CreateResult(SolveStatus.Unbounded, options);
            }

            // Find leaving variable
            int leavingRow = tableau.FindLeavingVariable(enteringCol);
            if (leavingRow == -1)
            {
                LogMessage("ERROR: Could not find leaving variable (ratio test failed)");
                return CreateResult(SolveStatus.Unbounded, options);
            }

            int leavingCol = tableau.Basis[leavingRow];
            LogMessage($"Leaving variable: {tableau.Variables[leavingCol]}");
            LogMessage($"Pivot element: ({leavingRow + 1}, {enteringCol + 1}) = {tableau[leavingRow, enteringCol]:0.000}");
            LogMessage();

            // Perform pivot operation
            tableau.Pivot(leavingRow, enteringCol);
            _iterationCount++;

            // Log new tableau
            LogMessage(tableau.ToFormattedString(_iterationCount));
        }

        LogMessage($"*** MAXIMUM ITERATIONS REACHED ({options.MaxIterations}) ***");
        return CreateResult(SolveStatus.NotSolved, options);
    }

    private void LogSolution(double[] solution, double objectiveValue, double[] reducedCosts, 
        double[] dualValues, CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        LogMessage("OPTIMAL SOLUTION:");
        LogMessage(new string('=', 30));
        
        LogMessage($"Objective Value: {FormatNumber(objectiveValue)}");
        LogMessage();
        
        LogMessage("Variable Values:");
        for (int i = 0; i < Math.Min(solution.Length, canonicalForm.OriginalVariables); i++)
        {
            LogMessage($"  {canonicalForm.VariableNames[i]} = {FormatNumber(solution[i])}");
        }
        LogMessage();
        
        LogMessage("Basic Variables:");
        var basis = new bool[solution.Length];
        for (int i = 0; i < canonicalForm.OriginalConstraints && i < canonicalForm.VariableNames.Length; i++)
        {
            if (Math.Abs(solution[i]) > 1e-10)
            {
                basis[i] = true;
                LogMessage($"  {canonicalForm.VariableNames[i]} = {FormatNumber(solution[i])} (basic)");
            }
        }
        LogMessage();
        
        LogMessage("Non-Basic Variables:");
        for (int i = 0; i < Math.Min(solution.Length, canonicalForm.OriginalVariables); i++)
        {
            if (!basis[i])
            {
                LogMessage($"  {canonicalForm.VariableNames[i]} = {FormatNumber(solution[i])} (non-basic)");
            }
        }
        LogMessage();
        
        LogMessage("Reduced Costs:");
        for (int i = 0; i < Math.Min(reducedCosts.Length, canonicalForm.OriginalVariables); i++)
        {
            LogMessage($"  {canonicalForm.VariableNames[i]}: {FormatNumber(reducedCosts[i])}");
        }
        LogMessage();
        
        LogMessage("Shadow Prices (Dual Values):");
        for (int i = 0; i < dualValues.Length; i++)
        {
            LogMessage($"  Constraint {i + 1}: {FormatNumber(dualValues[i])}");
        }
        LogMessage();
    }

    private SolveResult CreateResult(SolveStatus status, SolverOptions options)
    {
        if (!string.IsNullOrWhiteSpace(options.OutputFile))
        {
            WriteLogToFile(options.OutputFile);
            return new SolveResult { Status = status, LogPath = options.OutputFile };
        }
        return new SolveResult { Status = status };
    }

    private void LogMessage(string message = "")
    {
        _logBuilder.AppendLine(message);
        if (message.Length > 0)
            Console.WriteLine(message);
    }

    private void WriteLogToFile(string filePath)
    {
        try
        {
            var directory = Path.GetDirectoryName(filePath);
            if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                Directory.CreateDirectory(directory);
                
            File.WriteAllText(filePath, _logBuilder.ToString());
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Warning: Could not write log to {filePath}: {ex.Message}");
        }
    }

    private static string FormatNumber(double value)
    {
        return Math.Abs(value) < 1e-10 ? "0.000" : value.ToString("0.000", CultureInfo.InvariantCulture);
    }
}

