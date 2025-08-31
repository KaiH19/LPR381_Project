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

namespace Solver.Simplex.Revised;


public sealed class RevisedSimplexSolver : ISolver
{
    public string Name => "Revised Primal Simplex";
    private const double TOL = 1e-9;
    private readonly StringBuilder _logBuilder = new();
    private int _iterationCount = 0;


    private sealed class RevisedSimplexState
    {
        public required double[,] A { get; init; }
        public required double[] B { get; init; }
        public required double[] C { get; init; }
        public required string[] VariableNames { get; init; }
        public required int[] Basis { get; set; }
        public required double[,] BInverse { get; set; }
        public required bool IsMaximization { get; init; }
        public required int OriginalVariables { get; init; }
        public required int OriginalConstraints { get; init; }
    }

    private sealed class ElementaryMatrix
    {
        public int PivotColumn { get; init; }
        public double[] EtaVector { get; init; }
        
        public ElementaryMatrix(int pivotColumn, double[] etaVector)
        {
            PivotColumn = pivotColumn;
            EtaVector = (double[])etaVector.Clone();
        }
    }

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        options ??= new SolverOptions();
        _logBuilder.Clear();
        _iterationCount = 0;

        try
        {
            LogMessage("=== REVISED PRIMAL SIMPLEX METHOD ===");
            LogMessage($"Started at: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            LogMessage();

            // Step 1: Convert to canonical form
            LogMessage("STEP 1: Converting to Canonical Form");
            LogMessage(new string('-', 50));
            
            var canonicalForm = CanonicalFormConverter.Convert(model);
            LogMessage(canonicalForm.FormattedProblem);

            // Step 2: Initialize revised simplex state
            var state = InitializeRevisedSimplex(canonicalForm);
            
            // Step 3: Solve using revised simplex iterations
            LogMessage("STEP 2: Revised Simplex Iterations");
            LogMessage(new string('-', 50));

            var result = SolveRevised(state, options);
            
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

    private RevisedSimplexState InitializeRevisedSimplex(CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        var rows = canonicalForm.A.GetLength(0);
        var cols = canonicalForm.A.GetLength(1);

        // Find initial basis (identity columns from slack variables)
        var basis = new int[rows];
        var bInverse = new double[rows, rows];
        
        LogMessage("Finding Initial Basic Feasible Solution:");
        LogMessage("Looking for identity columns in slack variables...");
        
        // Initialize B^-1 as identity matrix (for slack variable basis)
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < rows; j++)
            {
                bInverse[i, j] = (i == j) ? 1.0 : 0.0;
            }
        }

        // Find basis variables (assuming slack variables form identity)
        for (int i = 0; i < rows; i++)
        {
            basis[i] = canonicalForm.OriginalVariables + i; // Start with slack variables
        }

        LogMessage($"Initial basis: [{string.Join(", ", basis.Select(b => canonicalForm.VariableNames[b]))}]");
        LogMessage();

        return new RevisedSimplexState
        {
            A = canonicalForm.A,
            B = canonicalForm.B,
            C = canonicalForm.C,
            VariableNames = canonicalForm.VariableNames,
            Basis = basis,
            BInverse = bInverse,
            IsMaximization = canonicalForm.IsMaximization,
            OriginalVariables = canonicalForm.OriginalVariables,
            OriginalConstraints = canonicalForm.OriginalConstraints
        };
    }

    private SolveResult SolveRevised(RevisedSimplexState state, SolverOptions options)
    {
        var elementaryMatrices = new List<ElementaryMatrix>();

        while (_iterationCount < options.MaxIterations)
        {
            _iterationCount++;
            LogMessage($"=== ITERATION {_iterationCount} ===");
            LogMessage();

            // Step 1: Calculate current basic solution
            var xB = CalculateBasicSolution(state);
            var objectiveValue = CalculateObjectiveValue(state, xB);
            
            LogMessage($"Current objective value: {FormatNumber(objectiveValue)}");
            LogMessage($"Basic solution: xB = [{string.Join(", ", xB.Select(FormatNumber))}]");
            LogMessage();

            // Step 2: Price-out step - calculate reduced costs
            LogMessage("PRICE-OUT STEP:");
            LogMessage("Calculating π = cB^T * B^-1");
            
            var pi = CalculateDualValues(state);
            LogMessage($"π = [{string.Join(", ", pi.Select(FormatNumber))}]");
            
            LogMessage("Calculating reduced costs for non-basic variables:");
            var (enteringCol, bestReducedCost) = FindEnteringVariable(state, pi);

            if (enteringCol == -1)
            {
                LogMessage("*** OPTIMAL SOLUTION FOUND ***");
                if (state.IsMaximization)
                    LogMessage("All reduced costs are ≤ 0 (within tolerance) for maximization");
                else
                    LogMessage("All reduced costs are ≥ 0 (within tolerance) for minimization");
                LogMessage();

                return CreateOptimalResult(state, xB, objectiveValue, pi);
            }

            LogMessage($"Entering variable: {state.VariableNames[enteringCol]} (reduced cost: {FormatNumber(bestReducedCost)})");
            LogMessage();

            // Step 3: Direction finding - solve B * d = A_j where j is entering variable
            LogMessage("DIRECTION FINDING:");
            LogMessage($"Solving B * d = A_{enteringCol + 1} (column of entering variable)");
            
            var enteringColumn = GetColumn(state.A, enteringCol);
            var direction = SolveLinearSystem(state.BInverse, enteringColumn);
            
            LogMessage($"Direction vector d = [{string.Join(", ", direction.Select(FormatNumber))}]");

            // Check for unboundedness
            if (direction.All(d => d <= 1e-10))
            {
                LogMessage("*** PROBLEM IS UNBOUNDED ***");
                LogMessage("All direction components are non-positive.");
                return CreateResult(SolveStatus.Unbounded, options);
            }

            // Step 4: Ratio test
            LogMessage();
            LogMessage("RATIO TEST:");
            var (leavingRow, minRatio) = PerformRatioTest(xB, direction);
            
            if (leavingRow == -1)
            {
                LogMessage("*** PROBLEM IS UNBOUNDED ***");
                LogMessage("No positive direction component found.");
                return CreateResult(SolveStatus.Unbounded, options);
            }

            int leavingCol = state.Basis[leavingRow];
            LogMessage($"Leaving variable: {state.VariableNames[leavingCol]} (row {leavingRow + 1}, ratio: {FormatNumber(minRatio)})");
            LogMessage();

            // Step 5: Update basis and B^-1 using product form
            LogMessage("UPDATING B^-1 using Product Form:");
            UpdateBasisAndInverse(state, enteringCol, leavingRow, direction, elementaryMatrices);
            
            LogMessage($"New basis: [{string.Join(", ", state.Basis.Select(b => state.VariableNames[b]))}]");
            LogMessage();
        }

        LogMessage($"*** MAXIMUM ITERATIONS REACHED ({options.MaxIterations}) ***");
        return CreateResult(SolveStatus.NotSolved, options);
    }

    private double[] CalculateBasicSolution(RevisedSimplexState state)
    {
        // xB = B^-1 * b
        return SolveLinearSystem(state.BInverse, state.B);
    }

    private double CalculateObjectiveValue(RevisedSimplexState state, double[] xB)
    {
        double obj = 0;
        for (int i = 0; i < state.Basis.Length; i++)
        {
            int basisVar = state.Basis[i];
            if (basisVar < state.C.Length)
            {
                double coeff = state.IsMaximization ? state.C[basisVar] : -state.C[basisVar];
                obj += coeff * xB[i];
            }
        }
        return obj;
    }

    private double[] CalculateDualValues(RevisedSimplexState state)
    {
        // π = cB^T * B^-1
        var cB = new double[state.Basis.Length];
        for (int i = 0; i < state.Basis.Length; i++)
        {
            int basisVar = state.Basis[i];
            if (basisVar < state.C.Length)
            {
                cB[i] = state.C[basisVar];
            }
        }

        return MultiplyVector(cB, state.BInverse);
    }

    private (int enteringCol, double bestReducedCost) FindEnteringVariable(RevisedSimplexState state, double[] pi)
    {
        int enteringCol = -1;
        double bestReducedCost = state.IsMaximization ? TOL : -TOL; // threshold

        int rows = state.A.GetLength(0);
        int cols = state.A.GetLength(1);

        for (int j = 0; j < cols; j++)
        {
            if (state.Basis.Contains(j)) continue;

            // r_j = c_j - π^T a_j
            double piAj = 0.0;
            for (int i = 0; i < rows; i++)
                piAj += pi[i] * state.A[i, j];

            double cj = (j < state.C.Length) ? state.C[j] : 0.0;
            double rc = cj - piAj;

            LogMessage($"  Variable {state.VariableNames[j]}: c_j - π^T*A_j = {FormatNumber(cj)} - {FormatNumber(piAj)} = {FormatNumber(rc)}");

            if (state.IsMaximization)
            {
                // MAX: enter if rc is the most positive, strictly > TOL
                if (rc > bestReducedCost)
                {
                    bestReducedCost = rc;
                    enteringCol = j;
                }
            }
            else
            {
                // MIN: enter if rc is the most negative, strictly < -TOL
                if (rc < bestReducedCost)
                {
                    bestReducedCost = rc;
                    enteringCol = j;
                }
            }
        }

        return (enteringCol, bestReducedCost);
    }

    private (int leavingRow, double minRatio) PerformRatioTest(double[] xB, double[] direction)
    {
        int leavingRow = -1;
        double minRatio = double.PositiveInfinity;

        for (int i = 0; i < direction.Length; i++)
        {
            if (direction[i] > 1e-10) // Only consider positive direction components
            {
                double ratio = xB[i] / direction[i];
                LogMessage($"  Row {i + 1}: {FormatNumber(xB[i])} / {FormatNumber(direction[i])} = {FormatNumber(ratio)}");
                
                if (ratio >= 0 && ratio < minRatio)
                {
                    minRatio = ratio;
                    leavingRow = i;
                }
            }
        }

        return (leavingRow, minRatio);
    }

    private void UpdateBasisAndInverse(RevisedSimplexState state, int enteringCol, int leavingRow, 
        double[] direction, List<ElementaryMatrix> elementaryMatrices)
    {
        // Create eta vector for elementary matrix E^-1
        var etaVector = new double[direction.Length];
        double pivotElement = direction[leavingRow];
        
        for (int i = 0; i < etaVector.Length; i++)
        {
            if (i == leavingRow)
                etaVector[i] = 1.0 / pivotElement;
            else
                etaVector[i] = -direction[i] / pivotElement;
        }

        LogMessage($"Eta vector: [{string.Join(", ", etaVector.Select(FormatNumber))}]");

        // Create elementary matrix
        var elementaryMatrix = new ElementaryMatrix(leavingRow, etaVector);
        elementaryMatrices.Add(elementaryMatrix);

        // Update B^-1 = E^-1 * B^-1
        ApplyElementaryMatrix(state.BInverse, elementaryMatrix);
        
        // Update basis
        state.Basis[leavingRow] = enteringCol;

        LogMessage("Updated B^-1:");
        LogBInverse(state.BInverse);
    }

    private void ApplyElementaryMatrix(double[,] bInverse, ElementaryMatrix elementary)
    {
        var rows = bInverse.GetLength(0);
        var cols = bInverse.GetLength(1);
        var newBInverse = new double[rows, cols];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (i == elementary.PivotColumn)
                {
                    // Row elementary.PivotColumn gets replaced
                    newBInverse[i, j] = elementary.EtaVector[i] * bInverse[elementary.PivotColumn, j];
                }
                else
                {
                    // Other rows get updated
                    newBInverse[i, j] = bInverse[i, j] + elementary.EtaVector[i] * bInverse[elementary.PivotColumn, j];
                }
            }
        }

        // Copy back
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                bInverse[i, j] = newBInverse[i, j];
            }
        }
    }

    private SolveResult CreateOptimalResult(RevisedSimplexState state, double[] xB, double objectiveValue, double[] pi)
    {
        // Construct full solution vector
        var solution = new double[state.VariableNames.Length];
        for (int i = 0; i < state.Basis.Length; i++)
        {
            solution[state.Basis[i]] = xB[i];
        }

        // Calculate reduced costs for all variables
        var reducedCosts = new double[state.VariableNames.Length];
        var rows = state.A.GetLength(0);
        
        for (int j = 0; j < state.VariableNames.Length; j++)
        {
            if (j < state.A.GetLength(1))
            {
                var columnJ = GetColumn(state.A, j);
                double piAj = 0;
                for (int i = 0; i < pi.Length; i++)
                {
                    piAj += pi[i] * columnJ[i];
                }
                
                double cj = (j < state.C.Length) ? state.C[j] : 0;
                reducedCosts[j] = cj - piAj;
            }
        }

        LogSolution(solution, objectiveValue, reducedCosts, pi, state);

        // Extract original variables only
        var originalSolution = new double[state.OriginalVariables];
        Array.Copy(solution, originalSolution, state.OriginalVariables);

        return new SolveResult
        {
            Status = SolveStatus.Optimal,
            X = originalSolution,
            ObjectiveValue = objectiveValue,
            Basis = (int[])state.Basis.Clone(),
            ReducedCosts = reducedCosts,
            Duals = pi
        };
    }

    private void LogSolution(double[] solution, double objectiveValue, double[] reducedCosts, 
        double[] dualValues, RevisedSimplexState state)
    {
        LogMessage("OPTIMAL SOLUTION:");
        LogMessage(new string('=', 30));
        
        LogMessage($"Objective Value: {FormatNumber(objectiveValue)}");
        LogMessage();
        
        LogMessage("Variable Values:");
        for (int i = 0; i < Math.Min(solution.Length, state.OriginalVariables); i++)
        {
            LogMessage($"  {state.VariableNames[i]} = {FormatNumber(solution[i])}");
        }
        LogMessage();
        
        LogMessage("Basic Variables:");
        for (int i = 0; i < state.Basis.Length; i++)
        {
            int varIndex = state.Basis[i];
            LogMessage($"  {state.VariableNames[varIndex]} = {FormatNumber(solution[varIndex])} (basic)");
        }
        LogMessage();
        
        LogMessage("Reduced Costs:");
        for (int i = 0; i < Math.Min(reducedCosts.Length, state.OriginalVariables); i++)
        {
            LogMessage($"  {state.VariableNames[i]}: {FormatNumber(reducedCosts[i])}");
        }
        LogMessage();
        
        LogMessage("Shadow Prices (Dual Values):");
        for (int i = 0; i < dualValues.Length; i++)
        {
            LogMessage($"  Constraint {i + 1}: {FormatNumber(dualValues[i])}");
        }
        LogMessage();
    }

    private void LogBInverse(double[,] bInverse)
    {
        var rows = bInverse.GetLength(0);
        var cols = bInverse.GetLength(1);
        
        for (int i = 0; i < rows; i++)
        {
            var row = new string[cols];
            for (int j = 0; j < cols; j++)
            {
                row[j] = FormatNumber(bInverse[i, j]).PadLeft(8);
            }
            LogMessage($"  [{string.Join(" ", row)}]");
        }
        LogMessage();
    }

    // Helper methods
    private double[] GetColumn(double[,] matrix, int col)
    {
        var rows = matrix.GetLength(0);
        var column = new double[rows];
        for (int i = 0; i < rows; i++)
        {
            column[i] = matrix[i, col];
        }
        return column;
    }

    private double[] SolveLinearSystem(double[,] bInverse, double[] rhs)
    {
        var n = rhs.Length;
        var result = new double[n];
        
        for (int i = 0; i < n; i++)
        {
            result[i] = 0;
            for (int j = 0; j < n; j++)
            {
                result[i] += bInverse[i, j] * rhs[j];
            }
        }
        
        return result;
    }

    private double[] MultiplyVector(double[] vector, double[,] matrix)
    {
        var cols = matrix.GetLength(1);
        var result = new double[cols];
        
        for (int j = 0; j < cols; j++)
        {
            result[j] = 0;
            for (int i = 0; i < vector.Length; i++)
            {
                result[j] += vector[i] * matrix[i, j];
            }
        }
        
        return result;
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

