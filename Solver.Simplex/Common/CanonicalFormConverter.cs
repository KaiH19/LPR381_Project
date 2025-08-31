using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Solver.Core.Models;

namespace Solver.Simplex.Common;


public sealed class CanonicalFormConverter
{
    public sealed class CanonicalForm
    {
        public required double[] C { get; init; }
        public required double[,] A { get; init; }
        public required double[] B { get; init; }
        public required string[] VariableNames { get; init; }
        public required bool IsMaximization { get; init; }
        public required int OriginalVariables { get; init; }
        public required int OriginalConstraints { get; init; }
        public required int[] SlackSurplusMapping { get; init; }
        public required string FormattedProblem { get; init; }
    }


    public static CanonicalForm Convert(LpModel model)
    {
        if (model == null) throw new ArgumentNullException(nameof(model));

        var originalVars = model.C.Length;
        var originalConstraints = model.B.Length;
        var isMaximization = model.ProblemSense == Sense.Max;

        // Step 1: Handle variable sign restrictions
        var (expandedC, expandedA, variableNames, originalVarMapping) = HandleVariableSignRestrictions(model);
        var currentVars = expandedC.Length;

        // Step 2: Add slack/surplus variables and handle constraint types
        var (finalC, finalA, finalB, finalVarNames, slackSurplusMapping) =
            HandleConstraints(expandedC, expandedA, model.B, model.RelOps, variableNames, isMaximization);

        // Step 3: Ensure all RHS values are non-negative
        NormalizeRhsValues(finalA, finalB);

        // Step 4: For minimization, negate objective coefficients
        if (!isMaximization)
        {
            for (int i = 0; i < finalC.Length; i++)
                finalC[i] = -finalC[i];
        }

        // Create formatted problem representation
        var formattedProblem = FormatCanonicalProblem(model, finalC, finalA, finalB, finalVarNames, isMaximization);

        return new CanonicalForm
        {
            C = finalC,
            A = finalA,
            B = finalB,
            VariableNames = finalVarNames,
            IsMaximization = isMaximization,
            OriginalVariables = originalVars,
            OriginalConstraints = originalConstraints,
            SlackSurplusMapping = slackSurplusMapping,
            FormattedProblem = formattedProblem
        };
    }

    private static (double[] C, double[,] A, string[] varNames, int[] mapping)
        HandleVariableSignRestrictions(LpModel model)
    {
        var newC = new List<double>();
        var newA = new List<double[]>();
        var varNames = new List<string>();
        var mapping = new List<int>();

        // Initialize constraint matrix
        for (int i = 0; i < model.B.Length; i++)
        {
            newA.Add(new double[0]); // Will be resized as we add variables
        }

        for (int j = 0; j < model.C.Length; j++)
        {
            var sign = model.VarSigns[j];
            var originalCoeff = model.C[j];

            switch (sign)
            {
                case VarSign.Plus: // x >= 0
                    newC.Add(originalCoeff);
                    varNames.Add($"x{j + 1}");
                    mapping.Add(j);
                    AddVariableColumn(newA, model.A, j);
                    break;

                case VarSign.Minus: // x <= 0, substitute x = -x' where x' >= 0
                    newC.Add(-originalCoeff);
                    varNames.Add($"x{j + 1}'");
                    mapping.Add(j);
                    AddNegativeVariableColumn(newA, model.A, j);
                    break;

                case VarSign.Urs: // x unrestricted, substitute x = x+ - x- where x+, x- >= 0
                    newC.Add(originalCoeff);  // x+
                    newC.Add(-originalCoeff); // x-
                    varNames.Add($"x{j + 1}+");
                    varNames.Add($"x{j + 1}-");
                    mapping.Add(j);
                    mapping.Add(j);
                    AddVariableColumn(newA, model.A, j);      // x+
                    AddNegativeVariableColumn(newA, model.A, j); // x-
                    break;

                case VarSign.Int:
                case VarSign.Bin:
                    // For now, treat as non-negative (integer constraints handled by branch & bound)
                    newC.Add(originalCoeff);
                    varNames.Add($"x{j + 1}");
                    mapping.Add(j);
                    AddVariableColumn(newA, model.A, j);
                    break;
            }
        }

        // Convert list of arrays to 2D array
        var finalA = new double[model.B.Length, newC.Count];
        for (int i = 0; i < model.B.Length; i++)
        {
            for (int j = 0; j < newC.Count; j++)
            {
                finalA[i, j] = newA[i][j];
            }
        }

        return (newC.ToArray(), finalA, varNames.ToArray(), mapping.ToArray());
    }

    private static void AddVariableColumn(List<double[]> A, double[][] originalA, int varIndex)
    {
        for (int i = 0; i < A.Count; i++)
        {
            var oldRow = A[i];
            var newRow = new double[oldRow.Length + 1];
            Array.Copy(oldRow, newRow, oldRow.Length);
            newRow[oldRow.Length] = originalA[i][varIndex];
            A[i] = newRow;
        }
    }

    private static void AddNegativeVariableColumn(List<double[]> A, double[][] originalA, int varIndex)
    {
        for (int i = 0; i < A.Count; i++)
        {
            var oldRow = A[i];
            var newRow = new double[oldRow.Length + 1];
            Array.Copy(oldRow, newRow, oldRow.Length);
            newRow[oldRow.Length] = -originalA[i][varIndex];
            A[i] = newRow;
        }
    }

    private static (double[] C, double[,] A, double[] B, string[] varNames, int[] slackSurplusMapping)
        HandleConstraints(double[] C, double[,] A, double[] B, RelOp[] relOps, string[] varNames, bool isMaximization)
    {
        var rows = A.GetLength(0);
        var cols = A.GetLength(1);
        var slackSurplusCount = 0;
        var slackSurplusMapping = new List<int>();

        // Count additional variables needed
        for (int i = 0; i < rows; i++)
        {
            if (relOps[i] != RelOp.Eq)
                slackSurplusCount++;
        }

        // Create expanded arrays
        var newC = new double[C.Length + slackSurplusCount];
        var newA = new double[rows, cols + slackSurplusCount];
        var newB = new double[rows];
        var newVarNames = new string[varNames.Length + slackSurplusCount];

        // Copy original objective coefficients
        Array.Copy(C, newC, C.Length);

        // Copy original constraint matrix
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                newA[i, j] = A[i, j];
            }
            newB[i] = B[i];
        }

        // Copy original variable names
        Array.Copy(varNames, newVarNames, varNames.Length);

        // Add slack/surplus variables
        int slackIndex = 0;
        for (int i = 0; i < rows; i++)
        {
            switch (relOps[i])
            {
                case RelOp.Le: // Add slack variable
                    newA[i, cols + slackIndex] = 1.0;
                    newVarNames[cols + slackIndex] = $"s{i + 1}";
                    slackSurplusMapping.Add(i);
                    slackIndex++;
                    break;

                case RelOp.Ge: // Add surplus variable (subtract)
                    newA[i, cols + slackIndex] = -1.0;
                    newVarNames[cols + slackIndex] = $"s{i + 1}";
                    slackSurplusMapping.Add(i);
                    slackIndex++;
                    break;

                case RelOp.Eq: // No additional variable
                    slackSurplusMapping.Add(-1); // Indicates no slack/surplus for this constraint
                    break;
            }
        }

        return (newC, newA, newB, newVarNames, slackSurplusMapping.ToArray());
    }

    private static void NormalizeRhsValues(double[,] A, double[] B)
    {
        var rows = A.GetLength(0);
        var cols = A.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            if (B[i] < 0)
            {
                // Multiply entire constraint by -1
                B[i] = -B[i];
                for (int j = 0; j < cols; j++)
                {
                    A[i, j] = -A[i, j];
                }
            }
        }
    }

    private static string FormatCanonicalProblem(LpModel original, double[] C, double[,] A, double[] B,
        string[] varNames, bool isMaximization)
    {
        var sb = new StringBuilder();

        sb.AppendLine("CANONICAL FORM");
        sb.AppendLine(new string('=', 50));

        // Objective function
        sb.Append(isMaximization ? "Maximize: " : "Minimize: ");
        sb.Append("z = ");

        for (int j = 0; j < C.Length; j++)
        {
            if (j > 0)
            {
                sb.Append(C[j] >= 0 ? " + " : " - ");
                sb.Append($"{Math.Abs(C[j]):0.###}{varNames[j]}");
            }
            else
            {
                sb.Append($"{C[j]:0.###}{varNames[j]}");
            }
        }
        sb.AppendLine();
        sb.AppendLine();

        // Constraints
        sb.AppendLine("Subject to:");
        var rows = A.GetLength(0);
        var cols = A.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            sb.Append("  ");
            for (int j = 0; j < cols; j++)
            {
                if (j > 0)
                {
                    sb.Append(A[i, j] >= 0 ? " + " : " - ");
                    sb.Append($"{Math.Abs(A[i, j]):0.###}{varNames[j]}");
                }
                else
                {
                    sb.Append($"{A[i, j]:0.###}{varNames[j]}");
                }
            }
            sb.AppendLine($" = {B[i]:0.###}");
        }

        sb.AppendLine();
        sb.Append("  ");
        for (int j = 0; j < varNames.Length; j++)
        {
            if (j > 0) sb.Append(", ");
            sb.Append($"{varNames[j]} >= 0");
        }
        sb.AppendLine();
        sb.AppendLine();

        return sb.ToString();
    }
}
