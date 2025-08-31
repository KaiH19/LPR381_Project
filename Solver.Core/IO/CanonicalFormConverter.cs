using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Solver.Core.Models;

namespace Solver.Simplex.Common;

public static class CanonicalFormConverter
{
    public sealed class CanonicalForm
    {
        public required double[,] A { get; init; }
        public required double[] B { get; init; }
        public required double[] C { get; init; }
        public required string[] VariableNames { get; init; }
        public required int OriginalVariables { get; init; }
        public required int OriginalConstraints { get; init; }
        public required bool IsMaximization { get; init; }

        public string FormattedProblem
        {
            get
            {
                var sb = new StringBuilder();
                sb.AppendLine($"Problem: {(IsMaximization ? "Maximize" : "Minimize")}");
                sb.Append("z = ");

                for (int j = 0; j < OriginalVariables; j++)
                {
                    if (j > 0 && C[j] >= 0) sb.Append(" + ");
                    else if (C[j] < 0) sb.Append(" - ");
                    sb.Append($"{Math.Abs(C[j]):0.###} {VariableNames[j]}");
                }
                sb.AppendLine();

                sb.AppendLine("Subject to:");
                for (int i = 0; i < OriginalConstraints; i++)
                {
                    bool firstTerm = true;
                    for (int j = 0; j < OriginalVariables; j++)
                    {
                        if (Math.Abs(A[i, j]) > 1e-10)
                        {
                            if (!firstTerm)
                                sb.Append(A[i, j] >= 0 ? " + " : " - ");
                            else
                                firstTerm = false;

                            sb.Append($"{Math.Abs(A[i, j]):0.###} {VariableNames[j]}");
                        }
                    }
                    sb.AppendLine($" <= {B[i]:0.###}");
                }

                sb.Append("Variable restrictions: ");
                for (int j = 0; j < OriginalVariables; j++)
                {
                    sb.Append($"{VariableNames[j]} >= 0");
                    if (j < OriginalVariables - 1) sb.Append(", ");
                }

                return sb.ToString();
            }
        }
    }

    public static CanonicalForm Convert(LpModel model)
    {
        int originalVars = model.C.Length;
        int originalConstraints = model.B.Length;

        // Count additional variables needed
        int slackVars = 0;
        int surplusVars = 0;
        int artificialVars = 0;

        foreach (var relOp in model.RelOps)
        {
            if (relOp == RelOp.Le) slackVars++;
            else if (relOp == RelOp.Ge) { surplusVars++; artificialVars++; }
            else if (relOp == RelOp.Eq) artificialVars++;
        }

        int totalVars = originalVars + slackVars + surplusVars + artificialVars;

        // Create variable names
        var variableNames = new List<string>();
        for (int j = 0; j < originalVars; j++) variableNames.Add($"x{j + 1}");
        for (int j = 0; j < slackVars; j++) variableNames.Add($"s{j + 1}");
        for (int j = 0; j < surplusVars; j++) variableNames.Add($"e{j + 1}");
        for (int j = 0; j < artificialVars; j++) variableNames.Add($"a{j + 1}");

        // Create canonical A matrix and b vector
        var A = new double[originalConstraints, totalVars];
        var B = (double[])model.B.Clone();
        var C = new double[totalVars];

        Array.Copy(model.C, C, originalVars); // Original objective coefficients

        int slackCount = 0, surplusCount = 0, artificialCount = 0;

        for (int i = 0; i < originalConstraints; i++)
        {
            // Copy original coefficients
            for (int j = 0; j < originalVars; j++)
            {
                A[i, j] = model.A[i][j];
            }

            // Add slack/surplus/artificial variables
            switch (model.RelOps[i])
            {
                case RelOp.Le:
                    A[i, originalVars + slackCount] = 1; // Slack variable
                    slackCount++;
                    break;

                case RelOp.Ge:
                    A[i, originalVars + slackVars + surplusCount] = -1; // Surplus variable
                    A[i, originalVars + slackVars + surplusVars + artificialCount] = 1; // Artificial variable
                    surplusCount++;
                    artificialCount++;
                    break;

                case RelOp.Eq:
                    A[i, originalVars + slackVars + surplusVars + artificialCount] = 1; // Artificial variable
                    artificialCount++;
                    break;
            }
        }

        // For minimization problems, we convert to maximization by negating the objective
        bool isMaximization = model.ProblemSense == Sense.Max;
        if (!isMaximization)
        {
            for (int j = 0; j < totalVars; j++)
            {
                C[j] = -C[j];
            }
        }

        return new CanonicalForm
        {
            A = A,
            B = B,
            C = C,
            VariableNames = variableNames.ToArray(),
            OriginalVariables = originalVars,
            OriginalConstraints = originalConstraints,
            IsMaximization = isMaximization
        };
    }
}
