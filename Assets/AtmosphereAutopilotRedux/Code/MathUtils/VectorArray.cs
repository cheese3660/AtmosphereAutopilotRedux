/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

/*
Modified by Infinity Irrealis, 2026, as part of AtmosphereAutopilotRedux,
a port of Atmosphere Autopilot to Kerbal Space Program 2.
See the repository history for details of changes.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilotRedux.MathUtils
{

    /// <summary>
    /// Fast matrix-like container, designed for array-like storage of small vectors of doubles.
    /// </summary>
    public class VectorArray
    {
        private int vector_size;
        private int capacity;
        private int data_size;

        public VectorArray(int vsize, int capacity)
        {
            if (vsize < 1)
                throw new ArgumentOutOfRangeException("vsize", "vsize < 1");
            if (capacity < 1)
                throw new ArgumentOutOfRangeException("capacity", "capacity < 1");
            vector_size = vsize;
            this.capacity = capacity;
            data_size = vector_size * capacity;
            data = new double[data_size];
        }

        public double[] data;

        public Vector this[int index]
        {
            get => new(this, index);
            set => value.DeepCopy(this[index]);
        }

        public void Resize(int new_capacity)
        {
            if (new_capacity < 1)
                throw new ArgumentOutOfRangeException("new_capacity", "new_capacity < 1");
            capacity = new_capacity;
            data_size = vector_size * capacity;
            Array.Resize(ref data, data_size);
        }

        public struct Vector
        {
            private VectorArray source;
            private int index;

            public Vector(int size)
            {
                source = new VectorArray(size, 1);
                index = 0;
            }

            public Vector(VectorArray binding, int index)
            {
                source = binding;
                this.index = index;
            }

            public static implicit operator Vector(double[] arr)
            {
                var dim = arr.Length;
                var a = new VectorArray(dim, 1);
                var v = a[0];
                for (var i = 0; i < dim; i++)
                    v[i] = arr[i];
                return v;
            }

            public static implicit operator Vector(double val)
            {
                var a = new VectorArray(1, 1);
                var v = a[0];
                v[0] = val;
                return v;
            }

            public double this[int proj]
            {
                get => source.data[index * source.vector_size + proj];
                set => source.data[index * source.vector_size + proj] = value;
            }

            public void DeepCopy(Vector dest)
            {
                for (var i = 0; i < source.vector_size; i++)
                    dest[i] = this[i];
            }

            public override string ToString()
            {
                if (source == null)
                    return "{}";
                else
                {
                    if (source.data_size == 1)
                        return this[0].ToString("G6");
                    else
                    {
                        var result = "{";
                        for (var i = 0; i < source.vector_size; i++)
                            if (i != source.vector_size - 1)
                                result += this[i].ToString("G6") + ", ";
                            else
                                result += this[i].ToString("G6");
                        result += "}";
                        return result;
                    }
                }
            }

            public string ToString(string format)
            {
                if (source == null)
                    return "{}";
                else
                {
                    if (source.data_size == 1)
                        return this[0].ToString(format);
                    else
                    {
                        var result = "{";
                        for (var i = 0; i < source.vector_size; i++)
                            if (i != source.vector_size - 1)
                                result += this[i].ToString(format) + ", ";
                            else
                                result += this[i].ToString(format);
                        result += "}";
                        return result;
                    }
                }
            }

            public override bool Equals(Object obj)
            {
                return obj is Vector && this == (Vector)obj;
            }
            public override int GetHashCode()
            {
                return source.GetHashCode() ^ index.GetHashCode();
            }
            public static bool operator ==(Vector x, Vector y)
            {
                return x.source == y.source && x.index == y.index;
            }
            public static bool operator !=(Vector x, Vector y)
            {
                return !(x == y);
            }

            public Vector Scale(double s)
            {
                for (var i = 0; i < source.vector_size; i++)
                    this[i] *= s;
                return this;
            }

            public Vector InverseScale(IList<double> s)
            {
                for (var i = 0; i < source.vector_size; i++)
                    this[i] /= s[i];
                return this;
            }

            public static double SqrLength(Vector a, Vector b)
            {
                var res = 0.0;
                for (var i = 0; i < a.source.vector_size; i++)
                {
                    var proj = a[i] - b[i];
                    res += proj * proj;
                }
                return res;
            }

            public static void Add(Vector a, Vector b, Vector res)
            {
                for (var i = 0; i < a.source.vector_size; i++)
                    res[i] = a[i] + b[i];
            }

            public static void Sub(Vector a, Vector b, Vector res)
            {
                for (var i = 0; i < a.source.vector_size; i++)
                    res[i] = a[i] - b[i];
            }

            public double SqrLength()
            {
                var res = 0.0;
                for (var i = 0; i < source.vector_size; i++)
                {
                    var proj = this[i];
                    res += proj * proj;
                }
                return res;
            }

            public static double Length(Vector a, Vector b)
            {
                return Math.Sqrt(SqrLength(a, b));
            }
        }
    }
    
}