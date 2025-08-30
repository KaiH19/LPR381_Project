using System;
using System.Collections.Generic;
using System.Linq;
using Solver.Core.Models;

namespace Solver.Integer.CuttingPlane.Models;

/// <summary>
/// Model representing the state and data for the cutting plane algorithm
/// </summary>
public class CuttingPlaneModel
{
    public LpModel OriginalModel { get; set; }
    public LpModel CurrentModel { get; set; }
    public SolveResult? CurrentSolution { get; set; }
    public List<GomoryCut> AppliedCuts { get; set; } = new();
    public int IterationCount { get; set; }
    public bool IsIntegerSolution { get; set; }
    
    public CuttingPlaneModel(LpModel originalModel)
    {
        OriginalModel = originalModel;
        CurrentModel = CreateLPRelaxation(originalModel);
    }
    
    /// <summary>
    /// Creates an LP relaxation by removing integer constraints
    /// </summary>
    private LpModel CreateLPRelaxation(LpModel model)
    {
        var relaxedVarSigns = model.VarSigns.Select(sign => 
            sign == VarSign.Int || sign == VarSign.Bin ? VarSign.Plus : sign).ToArray();

        return new LpModel
        {
            ProblemSense = model.ProblemSense,
            C = model.C,
            A = model.A,
            RelOps = model.RelOps,
            B = model.B,
            VarSigns = relaxedVarSigns,
            Name = model.Name
        };
    }
    
    /// <summary>
    /// Checks if the model has any integer variables
    /// </summary>
    public bool HasIntegerVariables()
    {
        return OriginalModel.VarSigns.Any(sign => sign == VarSign.Int || sign == VarSign.Bin);
    }
    
    /// <summary>
    /// Checks if the current solution satisfies integer constraints
    /// </summary>
    public bool CheckIntegerSolution()
    {
        if (CurrentSolution?.X == null) return false;
        
        for (int i = 0; i < Math.Min(CurrentSolution.X.Length, OriginalModel.VarSigns.Length); i++)
        {
            if (OriginalModel.VarSigns[i] == VarSign.Int || OriginalModel.VarSigns[i] == VarSign.Bin)
            {
                if (!IsInteger(CurrentSolution.X[i]))
                    return false;
            }
        }
        return true;
    }
    
    /// <summary>
    /// Checks if a single value is integer
    /// </summary>
    private bool IsInteger(double value)
    {
        const double INTEGER_TOLERANCE = 1e-9;
        return Math.Abs(value - Math.Round(value)) < INTEGER_TOLERANCE;
    }
    
    /// <summary>
    /// Updates the current model with a new cut
    /// </summary>
    public void AddCut(GomoryCut cut)
    {
        AppliedCuts.Add(cut);
        CurrentModel = ExpandModelWithCut(CurrentModel, cut);
        IterationCount++;
    }
    
    /// <summary>
    /// Expands the model by adding a new cutting plane constraint
    /// </summary>
    private LpModel ExpandModelWithCut(LpModel model, GomoryCut cut)
    {
        // Expand constraint matrix
        var newA = new double[model.A.Length + 1][];
        for (int i = 0; i < model.A.Length; i++)
        {
            newA[i] = model.A[i];
        }
        newA[model.A.Length] = cut.Coefficients;

        // Expand RHS vector
        var newB = new double[model.B.Length + 1];
        Array.Copy(model.B, newB, model.B.Length);
        newB[model.B.Length] = cut.RightHandSide;

        // Expand relation operators (cuts are <= constraints)
        var newRelOps = new RelOp[model.RelOps.Length + 1];
        Array.Copy(model.RelOps, newRelOps, model.RelOps.Length);
        newRelOps[model.RelOps.Length] = RelOp.Le;

        return new LpModel
        {
            ProblemSense = model.ProblemSense,
            C = model.C,
            A = newA,
            RelOps = newRelOps,
            B = newB,
            VarSigns = model.VarSigns,
            Name = model.Name
        };
    }
    
    /// <summary>
    /// Truncates solution to original variable count
    /// </summary>
    public double[] GetOriginalVariables()
    {
        if (CurrentSolution?.X == null) return new double[0];
        var truncated = new double[OriginalModel.C.Length];
        Array.Copy(CurrentSolution.X, truncated, Math.Min(CurrentSolution.X.Length, OriginalModel.C.Length));
        return truncated;
    }
}

/// <summary>
/// Represents a Gomory cutting plane
/// </summary>
public class GomoryCut
{
    public required double[] Coefficients { get; init; }
    public required double RightHandSide { get; init; }
    public required int VariableIndex { get; init; }
    public required string CutType { get; init; }
}
