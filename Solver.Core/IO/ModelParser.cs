using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Solver.Core.Models;
using System.Globalization;
using System.IO;

namespace Solver.Core.IO
{
    public static class ModelParser
    {
        public static LpModel ParseInputFile(string path)
        {
            if (!File.Exists(path)) throw new FileNotFoundException("Input file not found.", path);
            var lines = File.ReadAllLines(path)
                            .Where(l => !string.IsNullOrWhiteSpace(l))
                            .Select(l => l.Trim())
                            .ToList();
            if (lines.Count == 0) throw new InvalidOperationException("Empty input file.");

            // Line 1: sense + objective coeffs (signed)
            var t0 = Tokenize(lines[0]);
            if (t0.Count < 2) throw new InvalidOperationException("Invalid objective line.");
            var sense = t0[0].ToLower() switch
            {
                "max" => Sense.Max,
                "min" => Sense.Min,
                _ => throw new InvalidOperationException("First token must be 'max' or 'min'.")
            };
            var c = t0.Skip(1).Select(ParseSigned).ToArray();
            var n = c.Length;

            // Constraints until sign restriction line
            var A = new List<double[]>();
            var rels = new List<RelOp>();
            var B = new List<double>();

            int i = 1;
            for (; i < lines.Count; i++)
            {
                var tokens = Tokenize(lines[i]);
                if (IsSignRestrictionLine(tokens)) break;
                if (tokens.Count < n + 2) throw new InvalidOperationException($"Constraint {i} has too few tokens.");

                var coeffs = new double[n];
                int k = 0;
                for (; k < n; k++) coeffs[k] = ParseSigned(tokens[k]);

                var rel = tokens[k++] switch
                {
                    "<=" => RelOp.Le,
                    ">=" => RelOp.Ge,
                    "=" => RelOp.Eq,
                    _ => throw new InvalidOperationException($"Bad relation in constraint {i}.")
                };
                if (k >= tokens.Count) throw new InvalidOperationException($"Missing RHS in constraint {i}.");
                var rhs = ParseSigned(tokens[k]);

                A.Add(coeffs);
                rels.Add(rel);
                B.Add(rhs);
            }

            if (i >= lines.Count) throw new InvalidOperationException("Missing sign restrictions line.");
            var signTokens = Tokenize(lines[i]);
            if (signTokens.Count != n)
                throw new InvalidOperationException($"Sign restriction count ({signTokens.Count}) != variable count ({n}).");
            var varSigns = signTokens.Select(ParseVarSign).ToArray();

            return new LpModel
            {
                Name = Path.GetFileNameWithoutExtension(path),
                ProblemSense = sense,
                C = c,
                A = A.ToArray(),
                RelOps = rels.ToArray(),
                B = B.ToArray(),
                VarSigns = varSigns
            };
        }

        private static List<string> Tokenize(string line)
            => line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries).ToList();

        private static bool IsSignRestrictionLine(List<string> tokens)
        {
            if (tokens.Count == 0) return false;
            var ok = new HashSet<string>(StringComparer.OrdinalIgnoreCase) { "+", "-", "urs", "int", "bin" };
            return tokens.All(t => ok.Contains(t));
        }

        private static VarSign ParseVarSign(string t) => t.ToLower() switch
        {
            "+" => VarSign.Plus,
            "-" => VarSign.Minus,
            "urs" => VarSign.Urs,
            "int" => VarSign.Int,
            "bin" => VarSign.Bin,
            _ => throw new InvalidOperationException($"Unknown sign restriction: {t}")
        };

        private static double ParseSigned(string token)
        {
            if (double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
                return v;

            if (token.Length > 1 && (token[0] == '+' || token[0] == '-'))
            {
                if (double.TryParse(token[1..], NumberStyles.Float, CultureInfo.InvariantCulture, out var mag))
                    return token[0] == '-' ? -mag : mag;
            }
            throw new InvalidOperationException($"Bad numeric token: {token}");
        }



    }
}
