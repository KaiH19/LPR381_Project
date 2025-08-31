using System;
using System.Collections.Generic;
using System.Linq;
using System.Globalization;
using Solver.Core.Interfaces;
using Solver.Core.Models;
using Solver.Simplex.Common;

namespace Solver.Integer.BranchAndBound.Dual;

public sealed class DualSimplexSolver : ISolver
{
    public string Name => "Dual Simplex (Branch & Bound)";

    public SolveResult Solve(LpModel model, SolverOptions? options = null)
    {
        options ??= new SolverOptions();
        
        try
        {
            // Convert to canonical form using Solver.Simplex
            var canonicalForm = CanonicalFormConverter.Convert(model);
            
            // Check for feasible initial solution
            if (!HasFeasibleInitialSolution(canonicalForm))
            {
                if (options.Verbose)
                    Console.WriteLine("ERROR: No feasible initial basic solution found.");
                return new SolveResult { Status = SolveStatus.Infeasible };
            }
            
            // Create initial tableau
            var tableau = CreateInitialTableau(canonicalForm);
            
            // Display initial tableau if verbose
            if (options.Verbose)
            {
                Console.WriteLine("STEP 3: Initial Tableau");
                Console.WriteLine(new string('-', 50));
                Console.WriteLine(tableau.ToFormattedString(0));
                Console.WriteLine();
            }
            
            // Solve using dual simplex iterations
            var result = SolveTableau(tableau, canonicalForm, options);
            
            return result;
        }
        catch (Exception)
        {
            return new SolveResult
            {
                Status = SolveStatus.NotSolved,
                ObjectiveValue = null,
                X = null
            };
        }
    }

    private bool HasFeasibleInitialSolution(CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        // Check if we have enough slack variables to form an initial basic feasible solution
        var rows = canonicalForm.A.GetLength(0);
        var cols = canonicalForm.A.GetLength(1);
        
        // All RHS values should be non-negative (handled by canonical form converter)
        for (int i = 0; i < canonicalForm.B.Length; i++)
        {
            if (canonicalForm.B[i] < -1e-10)
                return false;
        }
        
        // Check if we can form an identity matrix for basic variables
        // This is a simplified check - a full implementation would include Phase I
        return true;
    }
    
    private bool IsFeasible(SimplexTableau tableau)
    {
        var rows = tableau.Rows;
        var cols = tableau.Cols;
        
        for (int i = 0; i < rows - 1; i++)
        {
            if (tableau[i, cols - 1] < -1e-10)
                return false;
        }
        return true;
    }
    
    private int FindLeavingVariable(SimplexTableau tableau)
    {
        var rows = tableau.Rows;
        var cols = tableau.Cols;
        
        int leavingRow = -1;
        double minRHS = 0;
        
        for (int i = 0; i < rows - 1; i++)
        {
            if (tableau[i, cols - 1] < minRHS)
            {
                minRHS = tableau[i, cols - 1];
                leavingRow = i;
            }
        }
        
        return leavingRow;
    }
    
    private int FindEnteringVariable(SimplexTableau tableau, int leavingRow)
    {
        var rows = tableau.Rows;
        var cols = tableau.Cols;
        
        int enteringCol = -1;
        double minRatio = double.MaxValue;
        
        for (int j = 0; j < cols - 1; j++)
        {
            if (tableau[leavingRow, j] < -1e-10)
            {
                double ratio = tableau[rows - 1, j] / tableau[leavingRow, j];
                if (ratio < minRatio)
                {
                    minRatio = ratio;
                    enteringCol = j;
                }
            }
        }
        
        return enteringCol;
    }

    private SimplexTableau CreateInitialTableau(CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        var rows = canonicalForm.A.GetLength(0);
        var cols = canonicalForm.A.GetLength(1);
        
        var tableau = new double[rows + 1, cols + 1];
        
        // Copy constraint coefficients
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                tableau[i, j] = canonicalForm.A[i, j];
            }
            tableau[i, cols] = canonicalForm.B[i]; // RHS
        }
        
        // Add objective row
        for (int j = 0; j < cols; j++)
        {
            tableau[rows, j] = -canonicalForm.C[j];
        }
        
        // Create initial basis (slack variables)
        var basis = new int[rows];
        for (int i = 0; i < rows; i++)
        {
            basis[i] = canonicalForm.OriginalVariables + i; // Slack variables start after original variables
        }
        
        return new SimplexTableau(tableau, basis, canonicalForm.VariableNames, canonicalForm.IsMaximization);
    }

    private SolveResult SolveTableau(SimplexTableau tableau, CanonicalFormConverter.CanonicalForm canonicalForm, SolverOptions options)
    {
        var maxIterations = Math.Min(options.MaxIterations, 20); // Limit to 20 iterations max
        var iteration = 0;
        
        while (iteration < maxIterations)
        {
            // Check if feasible (all RHS >= 0)
            if (IsFeasible(tableau))
            {
                if (options.Verbose)
                    Console.WriteLine($"Feasible solution found after {iteration} iterations");
                // If feasible, check if optimal
                if (tableau.IsOptimal())
                {
                    if (options.Verbose)
                    {
                        Console.WriteLine($"Optimal solution found after {iteration} iterations");
                        DisplayOptimalTableau(tableau, iteration, canonicalForm);
                    }
                    return ExtractSolution(tableau, canonicalForm);
                }
                else
                {
                    if (options.Verbose)
                        Console.WriteLine($"Switching to primal simplex after {iteration} iterations");
                    // Switch to primal simplex for final optimization
                    return FinishWithPrimalSimplex(tableau, canonicalForm, options);
                }
            }
            
            // Find leaving variable (most negative RHS)
            var leavingRow = FindLeavingVariable(tableau);
            if (leavingRow == -1)
            {
                if (options.Verbose)
                {
                    Console.WriteLine($"No leaving variable found after {iteration} iterations");
                    DisplayOptimalTableau(tableau, iteration, canonicalForm);
                }
                return ExtractSolution(tableau, canonicalForm);
            }
            
            // Find entering variable
            var enteringCol = FindEnteringVariable(tableau, leavingRow);
            if (enteringCol == -1)
            {
                if (options.Verbose)
                    Console.WriteLine($"Infeasible problem detected after {iteration} iterations");
                return new SolveResult { Status = SolveStatus.Infeasible };
            }
            
            // Pivot using SimplexTableau method
            tableau.Pivot(leavingRow, enteringCol);
            iteration++;
        }
        
        if (options.Verbose)
        {
            Console.WriteLine($"Maximum iterations ({maxIterations}) reached");
            DisplayOptimalTableau(tableau, iteration, canonicalForm);
        }
        return new SolveResult { Status = SolveStatus.NotSolved };
    }











    private SolveResult FinishWithPrimalSimplex(SimplexTableau tableau, CanonicalFormConverter.CanonicalForm canonicalForm, SolverOptions options)
    {
        var maxIterations = Math.Min(options.MaxIterations, 15); // Limit to 15 iterations max
        var iteration = 0;
        
        while (iteration < maxIterations)
        {
            // Check if optimal using SimplexTableau method
            if (tableau.IsOptimal())
            {
                if (options.Verbose)
                {
                    Console.WriteLine($"Primal simplex optimal after {iteration} iterations");
                    DisplayOptimalTableau(tableau, iteration, canonicalForm);
                }
                return ExtractSolution(tableau, canonicalForm);
            }
            
            // Find entering variable using SimplexTableau method
            var enteringCol = tableau.FindEnteringVariable();
            if (enteringCol == -1)
            {
                if (options.Verbose)
                {
                    Console.WriteLine($"No entering variable found after {iteration} iterations");
                    DisplayOptimalTableau(tableau, iteration, canonicalForm);
                }
                return ExtractSolution(tableau, canonicalForm);
            }
            
            // Check if unbounded
            if (tableau.IsUnbounded(enteringCol))
            {
                if (options.Verbose)
                    Console.WriteLine($"Unbounded problem detected after {iteration} iterations");
                return new SolveResult { Status = SolveStatus.Unbounded };
            }
            
            // Find leaving variable using SimplexTableau method
            var leavingRow = tableau.FindLeavingVariable(enteringCol);
            if (leavingRow == -1)
            {
                if (options.Verbose)
                    Console.WriteLine($"No leaving variable found after {iteration} iterations");
                return ExtractSolution(tableau, canonicalForm);
            }
            
            // Pivot using SimplexTableau method
            tableau.Pivot(leavingRow, enteringCol);
            iteration++;
        }
        
        if (options.Verbose)
        {
            Console.WriteLine($"Maximum primal iterations ({maxIterations}) reached");
            DisplayOptimalTableau(tableau, iteration, canonicalForm);
        }
        return new SolveResult { Status = SolveStatus.NotSolved };
    }



    private SolveResult ExtractSolution(SimplexTableau tableau, CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        // Use SimplexTableau's built-in solution extraction
        var (solution, objectiveValue) = tableau.GetBasicSolution();
        
        // Extract only the original variables (not slack variables)
        var originalSolution = new double[canonicalForm.OriginalVariables];
        for (int i = 0; i < canonicalForm.OriginalVariables; i++)
        {
            originalSolution[i] = solution[i];
        }
        
        // Validate that the solution respects bounds
        if (!ValidateSolutionBounds(originalSolution))
        {
            return new SolveResult
            {
                Status = SolveStatus.NotSolved,
                ObjectiveValue = null,
                X = null
            };
        }
        
        return new SolveResult
        {
            Status = SolveStatus.Optimal,
            ObjectiveValue = objectiveValue,
            X = originalSolution
        };
    }

    private bool ValidateSolutionBounds(double[] solution)
    {
        // For binary variables, ensure all values are in [0,1] range
        const double tolerance = 1e-6;
        return solution.All(x => x >= -tolerance && x <= 1.0 + tolerance);
    }

    private void DisplayOptimalTableau(SimplexTableau tableau, int iteration, CanonicalFormConverter.CanonicalForm canonicalForm)
    {
        Console.ForegroundColor = ConsoleColor.Red;
        Console.WriteLine($"OPTIMAL TABLEAU (Iteration {iteration})");
        Console.ResetColor();
        Console.WriteLine(new string('=', 80));
        Console.WriteLine(tableau.ToFormattedString(iteration));
        Console.WriteLine();
    }
    

}
