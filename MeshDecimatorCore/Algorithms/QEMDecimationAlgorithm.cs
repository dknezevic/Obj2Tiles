// QEMDecimationAlgorithm.cs
// MIT License
// Quadric Error Metrics (Garland–Heckbert) edge-collapse decimator implementing MeshDecimatorCore.Algorithms.DecimationAlgorithm.
// Quality-focused, suitable for large-ratio simplification with guards against degeneration.
//
// Author: ChatGPT (GPT-5 Thinking)

using System;
using System.Collections.Generic;
using MeshDecimatorCore;
using MeshDecimatorCore.Algorithms;
using MeshDecimatorCore.Math;

namespace MeshDecimatorCore.Algorithms
{
    /// <summary>
    /// Quadric Error Metrics (QEM) decimator with quality guards (normal deviation, dihedral sharp-edge preservation,
    /// min-area validation) and submesh/border handling. Implements <see cref="DecimationAlgorithm"/>.
    /// </summary>
    public sealed class QEMDecimationAlgorithm : DecimationAlgorithm
    {
        // ---------------- Public configuration knobs ----------------

        /// <summary>If true, rejects collapses that would flip face normals beyond <see cref="MaxNormalDeviationDegrees"/>.</summary>
        public bool EnableNormalCheck { get; set; } = true;

        /// <summary>Maximum allowed normal deviation in degrees when <see cref="EnableNormalCheck"/> is true. Default 135.</summary>
        public double MaxNormalDeviationDegrees { get; set; } = 135.0;

        /// <summary>Penalty added to cost when collapsing boundary-to-boundary edges (used if <see cref="PreserveBorders"/> is true).</summary>
        public double BoundaryPenalty { get; set; } = 1e6;

        /// <summary>If true, edges that connect vertices belonging to disjoint submeshes will not be collapsed.</summary>
        public bool RespectSubMeshes { get; set; } = true;

        /// <summary>If true, the output mesh will have normals recalculated.</summary>
        public bool RecomputeNormalsAfter { get; set; } = true;

        /// <summary>If true, isolated vertices (not referenced by any triangle) are removed in the result.</summary>
        public bool RemoveDanglingVertices { get; set; } = true;

        /// <summary>Reject candidate if any resulting triangle area is below this threshold (units^2). 0 disables.</summary>
        public double MinTriangleArea { get; set; } = 0.0;

        /// <summary>Use triangle area to scale face quadrics (recommended).</summary>
        public bool AreaWeightedQuadrics { get; set; } = true;

        /// <summary>Preserve sharp features by dihedral angle threshold (in degrees).</summary>
        public bool PreserveSharpEdges { get; set; } = true;

        /// <summary>Edges with dihedral angle ≥ SharpEdgeAngleDegrees are considered sharp/creases. Default 45°.</summary>
        public double SharpEdgeAngleDegrees { get; set; } = 45.0;

        /// <summary>If true, block collapses across sharp edges. If false, allow but penalize cost by <see cref="SharpEdgePenalty"/>.</summary>
        public bool DisallowSharpEdgeCollapse { get; set; } = true;

        /// <summary>Cost penalty for collapsing across a sharp edge when allowed.</summary>
        public double SharpEdgePenalty { get; set; } = 1e9;

        // ---------------- Internal state ----------------
        private Mesh _inputMesh;
        private Mesh _resultMesh;
        private bool _built;

        private Vertex[] _v;
        private Tri[] _tris;
        private int _activeTriCount;
        private int _activeVertexCount;
        private int _originalTriCount;

        private int[] _triToSubMesh; // triangle index -> submesh index
        private int _subMeshCount;

        private readonly Dictionary<EdgeKey, EdgeRec> _edgeRec = new();
        private readonly MinHeap<EdgeRec> _heap = new();
        private ulong _serial = 1;
        private double _cosNormalThreshold;

        // Edge helpers for sharp-edge detection
        private readonly Dictionary<EdgeKey, List<int>> _edgeFaces = new();
        private readonly HashSet<EdgeKey> _sharpEdges = new();

        #region DecimationAlgorithm implementation
        public override void Initialize(Mesh mesh)
        {
            _inputMesh = mesh ?? throw new ArgumentNullException(nameof(mesh));
            _resultMesh = null;
            _built = false;
            _originalTriCount = mesh.TriangleCount;
        }

        public override void DecimateMesh(int targetTrisCount)
        {
            if (_inputMesh == null) throw new InvalidOperationException("Call Initialize(mesh) first.");

            // Build topology on first run
            if (!_built) BuildTopologyAndInit();

            // Enforce limits
            targetTrisCount = System.Math.Max(targetTrisCount, 1);
            int vertexLimit = this.MaxVertexCount; // 0 means no limit

            int iteration = 0;
            while (_activeTriCount > targetTrisCount && _heap.Count > 0)
            {
                var rec = _heap.Pop();
                if (!_edgeRec.TryGetValue(rec.Key, out var cur) || !ReferenceEquals(rec, cur))
                    continue;
                if (_v[rec.A].Removed || _v[rec.B].Removed)
                    continue;

                if (!IsCollapseValid(rec))
                {
                    // Mark as stale and continue
                    _edgeRec.Remove(rec.Key);
                    continue;
                }

                Collapse(rec);
                RequeueAround(rec.A);
                iteration++;

                // Vertex limit enforcement
                if (vertexLimit > 0 && _activeVertexCount <= vertexLimit)
                    break;

                // Progress report every ~200 collapses
                if ((iteration % 200) == 0)
                {
                    ReportStatus(iteration, _originalTriCount, _activeTriCount, targetTrisCount);
                }
            }

            // Final report
            ReportStatus(iteration, _originalTriCount, _activeTriCount, targetTrisCount);

            _resultMesh = ExportMesh();
        }

        public override void DecimateMeshLossless()
        {
            if (_inputMesh == null) throw new InvalidOperationException("Call Initialize(mesh) first.");
            if (!_built) BuildTopologyAndInit();
            _resultMesh = ExportMesh();
        }

        public override Mesh ToMesh()
        {
            return _resultMesh ?? _inputMesh;
        }
        #endregion

        // -------------------- Build & Initialize --------------------
        private void BuildTopologyAndInit()
        {
            BuildTopology();
            _cosNormalThreshold = System.Math.Cos(MaxNormalDeviationDegrees * System.Math.PI / 180.0);
            InitQuadrics();
            DetectSharpEdges();
            InitEdges();
            _built = true;
        }

        private struct Tri
        {
            public int A, B, C;
            public bool Removed;
            public Vec3d Normal;
            public bool IsDegenerate => (A == B) || (B == C) || (C == A);
        }

        private sealed class Vertex
        {
            public Vec3d P;
            public bool Removed;
            public Quadric Q;
            public readonly HashSet<int> Tris = new();
            public readonly HashSet<int> Neigh = new();
            public bool IsBoundary;
            public ulong Version;
            public int SubMeshMask; // bitmask of submeshes this vertex belongs to
        }

        private void BuildTopology()
        {
            var verts = _inputMesh.Vertices;
            _subMeshCount = _inputMesh.SubMeshCount;
            var subIndices = _inputMesh.GetSubMeshIndices();

            _v = new Vertex[verts.Length];
            for (int i = 0; i < verts.Length; i++)
                _v[i] = new Vertex { P = new Vec3d(verts[i].x, verts[i].y, verts[i].z) };

            var tris = new List<Tri>();
            var triToSM = new List<int>();

            _edgeFaces.Clear();

            for (int sm = 0; sm < subIndices.Length; sm++)
            {
                var idx = subIndices[sm];
                if (idx == null) continue;
                for (int t = 0; t < idx.Length; t += 3)
                {
                    int a = idx[t + 0];
                    int b = idx[t + 1];
                    int c = idx[t + 2];
                    var tri = new Tri { A = a, B = b, C = c };
                    if (tri.IsDegenerate) tri.Removed = true;

                    tris.Add(tri);
                    triToSM.Add(sm);

                    if (!tri.Removed)
                    {
                        int ti = tris.Count - 1;
                        _v[a].Tris.Add(ti);
                        _v[b].Tris.Add(ti);
                        _v[c].Tris.Add(ti);
                        _v[a].Neigh.Add(b); _v[a].Neigh.Add(c);
                        _v[b].Neigh.Add(a); _v[b].Neigh.Add(c);
                        _v[c].Neigh.Add(a); _v[c].Neigh.Add(b);

                        _v[a].SubMeshMask |= (1 << sm);
                        _v[b].SubMeshMask |= (1 << sm);
                        _v[c].SubMeshMask |= (1 << sm);

                        AddEdgeFace(new EdgeKey(a, b), ti);
                        AddEdgeFace(new EdgeKey(b, c), ti);
                        AddEdgeFace(new EdgeKey(c, a), ti);
                    }
                }
            }

            _tris = tris.ToArray();
            _triToSubMesh = triToSM.ToArray();

            for (int i = 0; i < _tris.Length; i++)
            {
                if (_tris[i].Removed) continue;
                _tris[i].Normal = FaceNormal(_tris[i].A, _tris[i].B, _tris[i].C);
            }

            // Mark boundary vertices if requested
            if (this.PreserveBorders)
            {
                var use = new Dictionary<EdgeKey, int>();
                for (int i = 0; i < _tris.Length; i++)
                {
                    if (_tris[i].Removed) continue;
                    var t = _tris[i];
                    Inc(use, new EdgeKey(t.A, t.B));
                    Inc(use, new EdgeKey(t.B, t.C));
                    Inc(use, new EdgeKey(t.C, t.A));
                }
                foreach (var kv in use)
                {
                    if (kv.Value == 1)
                    {
                        _v[kv.Key.A].IsBoundary = true;
                        _v[kv.Key.B].IsBoundary = true;
                    }
                }
            }

            _activeTriCount = CountActiveTris();
            _activeVertexCount = _v.Length;

            static void Inc(Dictionary<EdgeKey, int> d, EdgeKey k)
            {
                d.TryGetValue(k, out int c);
                d[k] = c + 1;
            }

            void AddEdgeFace(EdgeKey k, int ti)
            {
                if (!_edgeFaces.TryGetValue(k, out var list))
                {
                    list = new List<int>(2);
                    _edgeFaces[k] = list;
                }
                list.Add(ti);
            }
        }

        private int CountActiveTris()
        {
            int c = 0;
            for (int i = 0; i < _tris.Length; i++) if (!_tris[i].Removed) c++;
            return c;
        }

        // -------------------- Quadrics --------------------
        private void InitQuadrics()
        {
            for (int i = 0; i < _v.Length; i++) _v[i].Q = Quadric.Zero;

            for (int i = 0; i < _tris.Length; i++)
            {
                if (_tris[i].Removed) continue;
                var t = _tris[i];
                var a = _v[t.A].P;
                var b = _v[t.B].P;
                var c = _v[t.C].P;

                var cross = Vec3d.Cross(b - a, c - a);
                double len = cross.Length();
                if (len < 1e-18) continue;
                double area = 0.5 * len;
                var n = cross / len;
                double d = -Vec3d.Dot(n, a);
                var plane = new Vec4d(n.x, n.y, n.z, d);
                var Kp = Quadric.FromPlane(plane);
                if (AreaWeightedQuadrics) Kp.Scale(area);

                _v[t.A].Q = _v[t.A].Q + Kp;
                _v[t.B].Q = _v[t.B].Q + Kp;
                _v[t.C].Q = _v[t.C].Q + Kp;
            }

            if (this.PreserveBorders)
            {
                var seen = new HashSet<EdgeKey>();
                for (int i = 0; i < _tris.Length; i++)
                {
                    if (_tris[i].Removed) continue;
                    var t = _tris[i];
                    AddIfBoundary(t.A, t.B);
                    AddIfBoundary(t.B, t.C);
                    AddIfBoundary(t.C, t.A);
                }

                void AddIfBoundary(int ia, int ib)
                {
                    var key = new EdgeKey(ia, ib);
                    if (!seen.Add(key)) return;
                    if (_v[key.A].IsBoundary && _v[key.B].IsBoundary)
                    {
                        var p0 = _v[key.A].P;
                        var p1 = _v[key.B].P;
                        var e = p1 - p0;
                        double el = e.Length();
                        if (el < 1e-18) return;
                        e /= el;

                        var t = (System.Math.Abs(e.y) < 0.9) ? new Vec3d(0, 1, 0) : new Vec3d(1, 0, 0);
                        var n = Vec3d.Normalize(Vec3d.Cross(e, t));
                        double d = -Vec3d.Dot(n, p0);
                        var plane = new Vec4d(n.x, n.y, n.z, d);
                        var K = Quadric.FromPlane(plane);
                        K.Scale(BoundaryPenalty);

                        _v[key.A].Q = _v[key.A].Q + K;
                        _v[key.B].Q = _v[key.B].Q + K;
                    }
                }
            }
        }

        // -------------------- Sharp edges --------------------
        private void DetectSharpEdges()
        {
            _sharpEdges.Clear();
            if (!PreserveSharpEdges) return;

            double cosThresh = System.Math.Cos(SharpEdgeAngleDegrees * System.Math.PI / 180.0);

            foreach (var kv in _edgeFaces)
            {
                var faces = kv.Value;
                if (faces.Count != 2) continue; // borders handled via PreserveBorders
                int t0 = faces[0], t1 = faces[1];
                if (_tris[t0].Removed || _tris[t1].Removed) continue;

                double dot = Vec3d.Dot(_tris[t0].Normal, _tris[t1].Normal);
                // clamp
                if (dot > 1.0) dot = 1.0;
                if (dot < -1.0) dot = -1.0;
                // dihedral angle check
                if (dot <= cosThresh)
                    _sharpEdges.Add(kv.Key);
            }
        }

        // -------------------- Edge queue --------------------
        private void InitEdges()
        {
            _heap.Clear();
            _edgeRec.Clear();

            for (int a = 0; a < _v.Length; a++)
            {
                if (_v[a].Removed) continue;
                foreach (var b in _v[a].Neigh)
                {
                    if (b <= a) continue;
                    QueueEdge(new EdgeKey(a, b));
                }
            }
        }

        private void QueueEdge(EdgeKey key)
        {
            if (_v[key.A].Removed || _v[key.B].Removed) return;

            if (RespectSubMeshes)
            {
                int mask = _v[key.A].SubMeshMask & _v[key.B].SubMeshMask;
                if (mask == 0) return; // do not collapse across submesh borders
            }

            // Sharp edge preservation
            bool isSharp = PreserveSharpEdges && _sharpEdges.Contains(key);
            if (isSharp && DisallowSharpEdgeCollapse)
                return;

            var qsum = _v[key.A].Q + _v[key.B].Q;
            Vec3d pos;
            if (!qsum.SolveOptimal(out pos))
            {
                var pa = _v[key.A].P;
                var pb = _v[key.B].P;
                var pm = (pa + pb) * 0.5;
                double ea = qsum.Evaluate(pa);
                double eb = qsum.Evaluate(pb);
                double em = qsum.Evaluate(pm);
                pos = (ea <= eb && ea <= em) ? pa : (eb <= em ? pb : pm);
            }
            double cost = qsum.Evaluate(pos);
            if (this.PreserveBorders && _v[key.A].IsBoundary && _v[key.B].IsBoundary)
                cost += BoundaryPenalty;
            if (isSharp && !DisallowSharpEdgeCollapse)
                cost += SharpEdgePenalty;

            var rec = new EdgeRec
            {
                Key = key,
                Pos = pos,
                Cost = cost,
                VerA = _v[key.A].Version,
                VerB = _v[key.B].Version,
                Serial = _serial++
            };

            _edgeRec[key] = rec;
            _heap.Push(rec);
        }

        // -------------------- Validation & Collapse --------------------
        private bool IsCollapseValid(EdgeRec rec)
        {
            int a = rec.A, b = rec.B;
            if (_v[a].Removed || _v[b].Removed) return false;
            if (_v[a].Version != rec.VerA || _v[b].Version != rec.VerB) return false;

            if (RespectSubMeshes)
            {
                int mask = _v[a].SubMeshMask & _v[b].SubMeshMask;
                if (mask == 0) return false;
            }

            if (PreserveSharpEdges && _sharpEdges.Contains(rec.Key) && DisallowSharpEdgeCollapse)
                return false;

            var affected = new HashSet<int>();
            foreach (var ti in _v[a].Tris) affected.Add(ti);
            foreach (var ti in _v[b].Tris) affected.Add(ti);

            foreach (var ti in affected)
            {
                if (_tris[ti].Removed) continue;

                int oa = _tris[ti].A, ob = _tris[ti].B, oc = _tris[ti].C;
                bool hadA = (oa == a) || (ob == a) || (oc == a);
                bool hadB = (oa == b) || (ob == b) || (oc == b);
                bool isEdgeTriangle = hadA && hadB; // triangle that uses both a and b (will collapse away)

                int ia = oa, ib = ob, ic = oc;
                if (ia == b) ia = a;
                if (ib == b) ib = a;
                if (ic == b) ic = a;

                bool deg = (ia == ib || ib == ic || ic == ia);
                if (deg)
                {
                    if (isEdgeTriangle)
                        continue; // allowed to degenerate; collapse will remove it
                    return false;
                }

                // Area check BEFORE collapse to avoid creating slivers
                if (MinTriangleArea > 0.0)
                {
                    var pa0 = (ia == a) ? rec.Pos : _v[ia].P;
                    var pb0 = (ib == a) ? rec.Pos : _v[ib].P;
                    var pc0 = (ic == a) ? rec.Pos : _v[ic].P;
                    var cp = Vec3d.Cross(pb0 - pa0, pc0 - pa0);
                    double area = 0.5 * cp.Length();
                    if (area < MinTriangleArea)
                        return false;
                }

                if (EnableNormalCheck && (hadA || hadB))
                {
                    var pa = (ia == a) ? rec.Pos : _v[ia].P;
                    var pb = (ib == a) ? rec.Pos : _v[ib].P;
                    var pc = (ic == a) ? rec.Pos : _v[ic].P;

                    var nNew = FaceNormal(pa, pb, pc);
                    var nOld = _tris[ti].Normal;
                    double dp = Vec3d.Dot(nOld, nNew);
                    if (double.IsNaN(dp) || dp < _cosNormalThreshold)
                        return false;
                }
            }

            return true;
        }

        private void Collapse(EdgeRec rec)
        {
            int a = rec.A, b = rec.B;

            _v[a].P = rec.Pos;
            _v[a].Q = _v[a].Q + _v[b].Q;
            _v[a].SubMeshMask |= _v[b].SubMeshMask;

            var trisB = new List<int>(_v[b].Tris);
            foreach (var ti in trisB)
            {
                if (_tris[ti].Removed) continue;
                ref var t = ref _tris[ti];
                bool hadA = (t.A == a) || (t.B == a) || (t.C == a);
                bool hadB = (t.A == b) || (t.B == b) || (t.C == b);

                if (t.A == b) t.A = a;
                if (t.B == b) t.B = a;
                if (t.C == b) t.C = a;

                bool deg = t.IsDegenerate;
                if (!deg && MinTriangleArea > 0.0)
                {
                    var pa = _v[t.A].P; var pb = _v[t.B].P; var pc = _v[t.C].P;
                    var cp = Vec3d.Cross(pb - pa, pc - pa);
                    double area = 0.5 * cp.Length();
                    deg = area < MinTriangleArea;
                }

                if (deg || (hadA && hadB)) // remove triangles along the collapsed edge
                {
                    if (!t.Removed)
                    {
                        t.Removed = true;
                        _activeTriCount--;
                        _v[t.A].Tris.Remove(ti);
                        _v[t.B].Tris.Remove(ti);
                        _v[t.C].Tris.Remove(ti);
                    }
                }
                else
                {
                    t.Normal = FaceNormal(t.A, t.B, t.C);
                    _v[t.A].Tris.Add(ti);
                    _v[t.B].Tris.Add(ti);
                    _v[t.C].Tris.Add(ti);
                }
            }

            foreach (var nb in _v[b].Neigh)
            {
                if (nb == a) continue;
                _v[a].Neigh.Add(nb);
                _v[nb].Neigh.Remove(b);
                _v[nb].Neigh.Add(a);
            }
            _v[a].Neigh.Remove(b);
            _v[b].Neigh.Clear();

            if (!_v[b].Removed)
            {
                _v[b].Removed = true;
                _activeVertexCount--;
            }
            _v[b].Tris.Clear();
            _v[b].Version++;
            _v[a].Version++;
        }

        private void RequeueAround(int a)
        {
            foreach (var nb in _v[a].Neigh)
            {
                if (_v[nb].Removed) continue;
                QueueEdge(new EdgeKey(a, nb));
            }
        }

        // -------------------- Export --------------------
        private Mesh ExportMesh()
        {
            int n = _v.Length;
            int[] map = new int[n];
            int newCount = 0;
            for (int i = 0; i < n; i++)
            {
                if (!_v[i].Removed && (!RemoveDanglingVertices || _v[i].Tris.Count > 0))
                    map[i] = newCount++;
                else
                    map[i] = -1;
            }

            var outVerts = new Vector3d[newCount];
            for (int i = 0; i < n; i++)
            {
                int m = map[i];
                if (m >= 0)
                {
                    var p = _v[i].P;
                    outVerts[m] = new Vector3d(p.x, p.y, p.z);
                }
            }

            var subLists = new List<int>[_subMeshCount];
            int smCount = _subMeshCount;
            for (int sm = 0; sm < smCount; sm++) subLists[sm] = new List<int>();

            for (int ti = 0; ti < _tris.Length; ti++)
            {
                if (_tris[ti].Removed) continue;
                int a = map[_tris[ti].A];
                int b = map[_tris[ti].B];
                int c = map[_tris[ti].C];
                if (a < 0 || b < 0 || c < 0) continue;
                if (a == b || b == c || c == a) continue;

                int sm = _triToSubMesh[ti];
                subLists[sm].Add(a); subLists[sm].Add(b); subLists[sm].Add(c);
            }

            var subIdx = new int[_subMeshCount][];
            for (int sm = 0; sm < _subMeshCount; sm++)
                subIdx[sm] = subLists[sm].ToArray();

            var outMesh = new Mesh(outVerts, subIdx);
            if (RecomputeNormalsAfter)
                outMesh.RecalculateNormals();
            return outMesh;
        }

        // -------------------- Helpers --------------------
        private Vec3d FaceNormal(int ia, int ib, int ic) => FaceNormal(_v[ia].P, _v[ib].P, _v[ic].P);
        private static Vec3d FaceNormal(in Vec3d a, in Vec3d b, in Vec3d c)
        {
            var n = Vec3d.Cross(b - a, c - a);
            double len = n.Length();
            if (len > 0) n /= len;
            return n;
        }

        private readonly struct Vec3d
        {
            public readonly double x, y, z;
            public Vec3d(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
            public static Vec3d operator +(Vec3d a, Vec3d b) => new Vec3d(a.x + b.x, a.y + b.y, a.z + b.z);
            public static Vec3d operator -(Vec3d a, Vec3d b) => new Vec3d(a.x - b.x, a.y - b.y, a.z - b.z);
            public static Vec3d operator *(Vec3d a, double s) => new Vec3d(a.x * s, a.y * s, a.z * s);
            public static Vec3d operator /(Vec3d a, double s) => new Vec3d(a.x / s, a.y / s, a.z / s);
            public static double Dot(in Vec3d a, in Vec3d b) => a.x * b.x + a.y * b.y + a.z * b.z;
            public static Vec3d Cross(in Vec3d a, in Vec3d b)
                => new Vec3d(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
            public double Length() => System.Math.Sqrt(x * x + y * y + z * z);
            public static Vec3d Normalize(in Vec3d v) { double l = v.Length(); return (l > 0) ? v / l : v; }
        }

        private readonly struct Vec4d
        {
            public readonly double x, y, z, w;
            public Vec4d(double x, double y, double z, double w) { this.x = x; this.y = y; this.z = z; this.w = w; }
        }

        private struct Quadric
        {
            public double q11, q12, q13, q14, q22, q23, q24, q33, q34, q44;

            public static Quadric Zero => default;

            public static Quadric FromPlane(in Vec4d p)
            {
                double a = p.x, b = p.y, c = p.z, d = p.w;
                return new Quadric
                {
                    q11 = a * a,
                    q12 = a * b,
                    q13 = a * c,
                    q14 = a * d,
                    q22 = b * b,
                    q23 = b * c,
                    q24 = b * d,
                    q33 = c * c,
                    q34 = c * d,
                    q44 = d * d
                };
            }

            public void Scale(double s)
            {
                q11 *= s; q12 *= s; q13 *= s; q14 *= s;
                q22 *= s; q23 *= s; q24 *= s;
                q33 *= s; q34 *= s; q44 *= s;
            }

            public static Quadric operator +(in Quadric x, in Quadric y)
            {
                return new Quadric
                {
                    q11 = x.q11 + y.q11,
                    q12 = x.q12 + y.q12,
                    q13 = x.q13 + y.q13,
                    q14 = x.q14 + y.q14,
                    q22 = x.q22 + y.q22,
                    q23 = x.q23 + y.q23,
                    q24 = x.q24 + y.q24,
                    q33 = x.q33 + y.q33,
                    q34 = x.q34 + y.q34,
                    q44 = x.q44 + y.q44
                };
            }

            public double Evaluate(in Vec3d v)
            {
                double x = v.x, y = v.y, z = v.z;
                return
                    q11 * x * x +
                    2.0 * q12 * x * y +
                    2.0 * q13 * x * z +
                    2.0 * q14 * x +
                    q22 * y * y +
                    2.0 * q23 * y * z +
                    2.0 * q24 * y +
                    q33 * z * z +
                    2.0 * q34 * z +
                    q44;
            }

            public bool SolveOptimal(out Vec3d v)
            {
                // Solve A v = -b, with Q = [A b; b^T c]
                double a11 = q11, a12 = q12, a13 = q13;
                double a22 = q22, a23 = q23;
                double a33 = q33;
                double b1 = q14, b2 = q24, b3 = q34;

                double det =
                    a11 * (a22 * a33 - a23 * a23) -
                    a12 * (a12 * a33 - a13 * a23) +
                    a13 * (a12 * a23 - a13 * a22);

                if (System.Math.Abs(det) < 1e-18)
                {
                    v = default;
                    return false;
                }

                double inv11 = (a22 * a33 - a23 * a23) / det;
                double inv12 = (a13 * a23 - a12 * a33) / det;
                double inv13 = (a12 * a23 - a13 * a22) / det;
                double inv22 = (a11 * a33 - a13 * a13) / det;
                double inv23 = (a12 * a13 - a11 * a23) / det;
                double inv33 = (a11 * a22 - a12 * a12) / det;

                double vx = -(inv11 * b1 + inv12 * b2 + inv13 * b3);
                double vy = -(inv12 * b1 + inv22 * b2 + inv23 * b3);
                double vz = -(inv13 * b1 + inv23 * b2 + inv33 * b3);

                v = new Vec3d(vx, vy, vz);
                return true;
            }
        }

        private readonly struct EdgeKey : IEquatable<EdgeKey>
        {
            public readonly int A, B; // A < B
            public EdgeKey(int a, int b) { A = (a < b) ? a : b; B = (a < b) ? b : a; }
            public bool Equals(EdgeKey other) => A == other.A && B == other.B;
            public override bool Equals(object obj) => obj is EdgeKey k && Equals(k);
            public override int GetHashCode() => unchecked(A * 73856093 ^ B * 19349663);
        }

        private sealed class EdgeRec : IComparable<EdgeRec>
        {
            public EdgeKey Key;
            public int A => Key.A;
            public int B => Key.B;
            public Vec3d Pos;
            public double Cost;
            public ulong VerA, VerB;
            public ulong Serial;

            public int CompareTo(EdgeRec other)
            {
                int c = Cost.CompareTo(other.Cost);
                if (c != 0) return c;
                return Serial.CompareTo(other.Serial);
            }
        }

        private sealed class MinHeap<T> where T : IComparable<T>
        {
            private readonly List<T> _d = new();
            public int Count => _d.Count;
            public void Clear() => _d.Clear();
            public void Push(T it) { _d.Add(it); SiftUp(_d.Count - 1); }
            public T Pop()
            {
                int last = _d.Count - 1;
                var top = _d[0];
                _d[0] = _d[last];
                _d.RemoveAt(last);
                if (_d.Count > 0) SiftDown(0);
                return top;
            }
            private void SiftUp(int i)
            {
                while (i > 0)
                {
                    int p = (i - 1) >> 1;
                    if (_d[i].CompareTo(_d[p]) >= 0) break;
                    (_d[i], _d[p]) = (_d[p], _d[i]);
                    i = p;
                }
            }
            private void SiftDown(int i)
            {
                int n = _d.Count;
                while (true)
                {
                    int l = i * 2 + 1, r = l + 1, s = i;
                    if (l < n && _d[l].CompareTo(_d[s]) < 0) s = l;
                    if (r < n && _d[r].CompareTo(_d[s]) < 0) s = r;
                    if (s == i) break;
                    (_d[i], _d[s]) = (_d[s], _d[i]);
                    i = s;
                }
            }
        }
    }
}
