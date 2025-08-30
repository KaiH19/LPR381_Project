using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Models;

namespace Solver.Sensitivity;

public static class ModelEdit
{
    // Add a new decision variable (column) with its column coefficients,
    // objective coefficient cj, and sign restriction (default x>=0).
    public static LpModel AddVariable(LpModel m, double[] columnA, double cj, VarSign sign = VarSign.Plus)
    {
        if (columnA.Length != m.B.Length)
            throw new ArgumentException("Column length must equal number of constraints.");

        var n = m.C.Length;
        var C2 = new double[n + 1];
        Array.Copy(m.C, C2, n);
        C2[n] = cj;

        var A2 = new double[m.A.Length][];
        for (int i = 0; i < m.A.Length; i++)
        {
            A2[i] = new double[n + 1];
            Array.Copy(m.A[i], A2[i], n);
            A2[i][n] = columnA[i];
        }

        var signs2 = new VarSign[n + 1];
        Array.Copy(m.VarSigns, signs2, n);
        signs2[n] = sign;

        return new LpModel
        {
            Name = m.Name,
            ProblemSense = m.ProblemSense,
            C = C2,
            A = A2,
            B = m.B,
            RelOps = m.RelOps,
            VarSigns = signs2
        };
    }

    // Add a new constraint (row) with coefficients rowA, relation, and rhs.
    public static LpModel AddConstraint(LpModel m, double[] rowA, RelOp rel, double rhs)
    {
        if (rowA.Length != m.C.Length)
            throw new ArgumentException("Row length must equal number of variables.");

        var m2 = m.B.Length + 1;

        var A2 = new double[m2][];
        for (int i = 0; i < m.B.Length; i++) A2[i] = (double[])m.A[i].Clone();
        A2[m2 - 1] = (double[])rowA.Clone();

        var B2 = new double[m2];
        Array.Copy(m.B, B2, m.B.Length);
        B2[m2 - 1] = rhs;

        var R2 = new RelOp[m2];
        Array.Copy(m.RelOps, R2, m.RelOps.Length);
        R2[m2 - 1] = rel;

        return new LpModel
        {
            Name = m.Name,
            ProblemSense = m.ProblemSense,
            C = m.C,
            A = A2,
            B = B2,
            RelOps = R2,
            VarSigns = m.VarSigns
        };
    }
}

