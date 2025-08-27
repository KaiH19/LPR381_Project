using System;
using System.Globalization;
using System.Linq;
using System.Text;

namespace Solver.Simplex.Common;

/// <summary>
/// Represents a simplex tableau for the primal simplex method.
/// Encapsulates tableau operations and formatting for professional output.
/// </summary>
public sealed class SimplexTableau
{
    private readonly double[,] _tableau;
    private readonly int[] _basis;
    private readonly string[] _variables;
    private readonly int _rows;
    private readonly int _cols;
    private readonly bool _isMaximization;

    public SimplexTableau(double[,] tableau, int[] basis, string[] variables, bool isMaximization = true)
    {
        _tableau = tableau ?? throw new ArgumentNullException(nameof(tableau));
        _basis = basis ?? throw new ArgumentNullException(nameof(basis));
        _variables = variables ?? throw new ArgumentNullException(nameof(variables));
        _isMaximization = isMaximization;
        _rows = tableau.GetLength(0);
        _cols = tableau.GetLength(1);
    }

    public double this[int row, int col]
    {
        get => _tableau[row, col];
        set => _tableau[row, col] = value;
    }

    public int Rows => _rows;
    public int Cols => _cols;
    public int[] Basis => (int[])_basis.Clone();
    public string[] Variables => (string[])_variables.Clone();

    /// <summary>
    /// Gets the objective function row (last row).
    /// </summary>
    public double[] ObjectiveRow
    {
        get
        {
            var result = new double[_cols];
            for (int j = 0; j < _cols; j++)
                result[j] = _tableau[_rows - 1, j];
            return result;
        }
    }

    /// <summary>
    /// Gets the RHS column (last column).
    /// </summary>
    public double[] RhsColumn
    {
        get
        {
            var result = new double[_rows];
            for (int i = 0; i < _rows; i++)
                result[i] = _tableau[i, _cols - 1];
            return result;
        }
    }

    /// <summary>
    /// Finds the entering variable using the most negative rule for maximization
    /// or most positive rule for minimization.
    /// </summary>
    public int FindEnteringVariable()
    {
        var objRow = ObjectiveRow;
        int enteringCol = -1;
        double bestValue = 0;

        for (int j = 0; j < _cols - 1; j++) // Exclude RHS column
        {
            double value = objRow[j];
            if (_isMaximization)
            {
                // For maximization, look for most negative (Dantzig rule)
                if (value < bestValue)
                {
                    bestValue = value;
                    enteringCol = j;
                }
            }
            else
            {
                // For minimization, look for most positive
                if (value > bestValue)
                {
                    bestValue = value;
                    enteringCol = j;
                }
            }
        }

        return enteringCol;
    }

    /// <summary>
    /// Finds the leaving variable using minimum ratio test.
    /// </summary>
    public int FindLeavingVariable(int enteringCol)
    {
        if (enteringCol < 0 || enteringCol >= _cols - 1)
            return -1;

        int leavingRow = -1;
        double minRatio = double.PositiveInfinity;

        for (int i = 0; i < _rows - 1; i++) // Exclude objective row
        {
            double pivot = _tableau[i, enteringCol];
            double rhs = _tableau[i, _cols - 1];

            if (pivot > 1e-10) // Only consider positive pivots
            {
                double ratio = rhs / pivot;
                if (ratio >= 0 && ratio < minRatio)
                {
                    minRatio = ratio;
                    leavingRow = i;
                }
            }
        }

        return leavingRow;
    }

    
    public void Pivot(int pivotRow, int pivotCol)
    {
        if (pivotRow < 0 || pivotRow >= _rows || pivotCol < 0 || pivotCol >= _cols)
            throw new ArgumentException("Invalid pivot position");

        double pivotElement = _tableau[pivotRow, pivotCol];
        if (Math.Abs(pivotElement) < 1e-10)
            throw new InvalidOperationException("Cannot pivot on zero element");

        // Normalize pivot row
        for (int j = 0; j < _cols; j++)
            _tableau[pivotRow, j] /= pivotElement;

        // Eliminate column
        for (int i = 0; i < _rows; i++)
        {
            if (i != pivotRow)
            {
                double multiplier = _tableau[i, pivotCol];
                for (int j = 0; j < _cols; j++)
                    _tableau[i, j] -= multiplier * _tableau[pivotRow, j];
            }
        }

        // Update basis
        _basis[pivotRow] = pivotCol;
    }

  
    public bool IsOptimal()
    {
        var objRow = ObjectiveRow;
        if (_isMaximization)
        {
            // For maximization, optimal if no negative values in objective row
            return objRow.Take(_cols - 1).All(x => x >= -1e-10);
        }
        else
        {
            // For minimization, optimal if no positive values in objective row
            return objRow.Take(_cols - 1).All(x => x <= 1e-10);
        }
    }

   
    public bool IsUnbounded(int enteringCol)
    {
        if (enteringCol < 0) return false;

        for (int i = 0; i < _rows - 1; i++)
        {
            if (_tableau[i, enteringCol] > 1e-10)
                return false;
        }
        return true;
    }

  
    public (double[] solution, double objectiveValue) GetBasicSolution()
    {
        var solution = new double[_variables.Length];
        var rhs = RhsColumn;

        // Set basic variables
        for (int i = 0; i < _basis.Length && i < _rows - 1; i++)
        {
            int varIndex = _basis[i];
            if (varIndex < _variables.Length)
                solution[varIndex] = rhs[i];
        }

        double objectiveValue = rhs[_rows - 1];
        if (!_isMaximization) objectiveValue = -objectiveValue;

        return (solution, objectiveValue);
    }

    
    public double[] GetReducedCosts()
    {
        var objRow = ObjectiveRow;
        var reducedCosts = new double[_variables.Length];
        
        for (int j = 0; j < Math.Min(_variables.Length, _cols - 1); j++)
        {
            reducedCosts[j] = objRow[j];
        }

        return reducedCosts;
    }

    public double[] GetDualValues(int originalConstraints)
    {
        var objRow = ObjectiveRow;
        var dualValues = new double[originalConstraints];
        
        // Dual values are in the slack variable columns of the objective row
        int slackStart = _variables.Length;
        for (int i = 0; i < originalConstraints && (slackStart + i) < _cols - 1; i++)
        {
            dualValues[i] = _isMaximization ? -objRow[slackStart + i] : objRow[slackStart + i];
        }

        return dualValues;
    }

   
    public string ToFormattedString(int iteration = 0)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"Iteration {iteration}:");
        sb.AppendLine(new string('=', 80));

        // Header row
        sb.Append("Basis".PadRight(8));
        foreach (var variable in _variables)
        {
            sb.Append(variable.PadLeft(10));
        }
        sb.AppendLine("RHS".PadLeft(10));

        sb.AppendLine(new string('-', 80));

        // Constraint rows
        for (int i = 0; i < _rows - 1; i++)
        {
            string basisVar = i < _basis.Length ? _variables[_basis[i]] : "?";
            sb.Append(basisVar.PadRight(8));
            
            for (int j = 0; j < _cols; j++)
            {
                sb.Append(FormatNumber(_tableau[i, j]).PadLeft(10));
            }
            sb.AppendLine();
        }

        sb.AppendLine(new string('-', 80));

        // Objective row
        sb.Append("z".PadRight(8));
        for (int j = 0; j < _cols; j++)
        {
            sb.Append(FormatNumber(_tableau[_rows - 1, j]).PadLeft(10));
        }
        sb.AppendLine();
        sb.AppendLine();

        return sb.ToString();
    }

    private static string FormatNumber(double value)
    {
        return Math.Abs(value) < 1e-10 ? "0.000" : value.ToString("0.000", CultureInfo.InvariantCulture);
    }

   
    public SimplexTableau Clone()
    {
        var newTableau = new double[_rows, _cols];
        Array.Copy(_tableau, newTableau, _tableau.Length);
        
        return new SimplexTableau(newTableau, (int[])_basis.Clone(), (string[])_variables.Clone(), _isMaximization);
    }
}
