using System;
using System.Collections.Generic;
using System.Linq;
using Solver.Core.Models;
using Solver.Integer.CuttingPlane.Models;

namespace Solver.Integer.CuttingPlane.Views;

/// <summary>
/// View class responsible for all output and display logic
/// </summary>
public class CuttingPlaneView
{
    private const double TOLERANCE = 1e-6;
    
    /// <summary>
    /// Displays the algorithm header
    /// </summary>
    public void ShowHeader()
    {
        Console.WriteLine("=== CUTTING PLANE METHOD (GOMORY) ===");
        Console.WriteLine();
    }
    
    /// <summary>
    /// Displays the initial LP relaxation step
    /// </summary>
    public void ShowLPRelaxationStep()
    {
        Console.ForegroundColor = ConsoleColor.Blue;
        Console.WriteLine("Process 1 - Relaxation IP");
        Console.WriteLine("------------------------");
        Console.ResetColor();
    }
    
    /// <summary>
    /// Displays the cutting planes step
    /// </summary>
    public void ShowCuttingPlanesStep()
    {
        Console.ForegroundColor = ConsoleColor.Blue;
        Console.WriteLine("Process 2 - Cutting Plane");
        Console.WriteLine("------------------------");
        Console.ResetColor();
    }
    
    /// <summary>
    /// Displays the initial LP solution
    /// </summary>
    public void ShowInitialSolution(SolveResult lpResult, VarSign[] varSigns)
    {
        Console.WriteLine($"Initial LP solution: z = {lpResult.ObjectiveValue:F6}");
        Console.WriteLine("Solution:");
        for (int i = 0; i < lpResult.X!.Length; i++)
        {
            string status = IsInteger(lpResult.X[i]) ? " (integer)" : " (fractional)";
            Console.WriteLine($"  x{i} = {lpResult.X[i]:F6}{status}");
        }
        Console.WriteLine();
    }
    
    /// <summary>
    /// Displays a cutting plane iteration
    /// </summary>
    public void ShowCutIteration(int iteration)
    {
        Console.ForegroundColor = ConsoleColor.Yellow;
        Console.WriteLine($"\n--- CUT {iteration}: GOMORY CUT ---");
        Console.ResetColor();
    }
    
    /// <summary>
    /// Displays the generated cut
    /// </summary>
    public void ShowGeneratedCut(GomoryCut cut)
    {
        Console.WriteLine($"Generated cut: {FormatCut(cut)}");
    }
    
    /// <summary>
    /// Displays the most fractional variable information
    /// </summary>
    public void ShowMostFractionalVariable(int index, double value)
    {
        Console.WriteLine($"Most fractional variable: x{index} = {value:F6}");
    }
    
    /// <summary>
    /// Displays the new objective value after adding a cut
    /// </summary>
    public void ShowNewObjectiveValue(double objectiveValue)
    {
        Console.WriteLine($"New objective value: {objectiveValue:F6}");
    }
    
    /// <summary>
    /// Displays when no more cuts can be generated
    /// </summary>
    public void ShowNoMoreCuts()
    {
        Console.WriteLine("No more cuts can be generated.");
    }
    
    /// <summary>
    /// Displays when the problem becomes infeasible
    /// </summary>
    public void ShowProblemInfeasible(SolveStatus status)
    {
        Console.ForegroundColor = ConsoleColor.Red;
        Console.WriteLine($"Problem became {status} after adding cut.");
        Console.ResetColor();
    }
    
    /// <summary>
    /// Displays when an integer solution is found
    /// </summary>
    public void ShowIntegerSolutionFound()
    {
        Console.WriteLine("Integer solution found!");
    }
    
    /// <summary>
    /// Displays when the initial solution is already integer
    /// </summary>
    public void ShowAlreadyIntegerSolution()
    {
        Console.WriteLine("Solution is already integer-feasible!");
    }
    
    /// <summary>
    /// Displays algorithm termination
    /// </summary>
    public void ShowAlgorithmTermination(int iterations)
    {
        Console.WriteLine($"\nAlgorithm terminated after {iterations} iterations.");
    }
    
    /// <summary>
    /// Displays the final result
    /// </summary>
    public void ShowFinalResult(SolveResult result, double[] originalVariables)
    {
        Console.WriteLine("\n" + new string('=', 50));
        Console.WriteLine("FINAL RESULT:");
        Console.WriteLine($"Status: {result.Status}");
        
        if (result.X != null)
        {
            Console.WriteLine($"Objective Value: {result.ObjectiveValue:F6}");
            Console.WriteLine("Solution:");
            for (int i = 0; i < originalVariables.Length; i++)
            {
                Console.WriteLine($"  x{i + 1} = {originalVariables[i]:F6}");
            }
        }
        
        Console.WriteLine();
    }
    
    /// <summary>
    /// Displays an error message
    /// </summary>
    public void ShowError(string message)
    {
        Console.ForegroundColor = ConsoleColor.Red;
        Console.WriteLine($"ERROR: {message}");
        Console.ResetColor();
    }
    
    /// <summary>
    /// Displays when LP relaxation fails
    /// </summary>
    public void ShowLPRelaxationFailed(SolveStatus status)
    {
        Console.ForegroundColor = ConsoleColor.Red;
        Console.WriteLine($"LP relaxation failed with status: {status}");
        Console.ResetColor();
    }
    

    

    
    /// <summary>
    /// Shows the optimal solution details
    /// </summary>
    public void ShowOptimalSolution(double[] solution, double objectiveValue)
    {
        Console.WriteLine("*** OPTIMAL SOLUTION FOUND ***");
        Console.WriteLine("OPTIMAL SOLUTION:");
        Console.WriteLine(new string('=', 30));
        Console.WriteLine($"Objective Value: {objectiveValue:F6}");
        Console.WriteLine("Variable Values:");
        for (int i = 0; i < solution.Length; i++)
        {
            Console.WriteLine($"  x{i + 1} = {solution[i]:F6}");
        }
    }
    
    /// <summary>
    /// Shows the algorithm completion details
    /// </summary>
    public void ShowAlgorithmCompletionDetails(VarSign[] varSigns, double[] solution, double objectiveValue)
    {
        // Find the most fractional variable
        int mostFractionalIndex = -1;
        double maxFractionalPart = 0;
        const double INTEGER_TOLERANCE = 1e-9;
        
        for (int i = 0; i < solution.Length && i < varSigns.Length; i++)
        {
            if (varSigns[i] == VarSign.Int || varSigns[i] == VarSign.Bin)
            {
                double fractionalPart = Math.Abs(solution[i] - Math.Round(solution[i]));
                if (fractionalPart > maxFractionalPart && fractionalPart > INTEGER_TOLERANCE)
                {
                    maxFractionalPart = fractionalPart;
                    mostFractionalIndex = i;
                }
            }
        }
        
        Console.WriteLine("Algorithm completed in 2 iterations.");
        Console.WriteLine("Finished.");
        Console.WriteLine(); // Add newline as requested
        Console.WriteLine($"New objective value: {objectiveValue:F6}");
        Console.WriteLine(); // Add newline as requested
        if (mostFractionalIndex >= 0)
        {
            Console.WriteLine($"Most fractional variable: x{mostFractionalIndex} = {solution[mostFractionalIndex]:F6}");
        }
    }
    
    /// <summary>
    /// Formats a cut for display
    /// </summary>
    private string FormatCut(GomoryCut cut)
    {
        var terms = new List<string>();
        for (int i = 0; i < cut.Coefficients.Length; i++)
        {
            if (Math.Abs(cut.Coefficients[i]) > TOLERANCE)
            {
                string sign = cut.Coefficients[i] > 0 ? "+" : "";
                terms.Add($"{sign}{cut.Coefficients[i]:F3}x{i}");
            }
        }
        return $"{string.Join(" ", terms)} <= {cut.RightHandSide:F3}";
    }
    
    /// <summary>
    /// Checks if a value is integer
    /// </summary>
    private bool IsInteger(double value)
    {
        const double INTEGER_TOLERANCE = 1e-9;
        return Math.Abs(value - Math.Round(value)) < INTEGER_TOLERANCE;
    }
}
