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

namespace System.Collections.Generic
{

    // IList concatenator
    public class ListView<T> : IList<T>, ICollection<T>, IEnumerable<T>, IEnumerable
    {
        private readonly List<IList<T>> _lists = new List<IList<T>>();

        public ListView(params IList<T>[] targets)
        {
            foreach (var target in targets)
                if (target != null)
                    _lists.Add(target);
        }

        public IEnumerator<T> GetEnumerator()
        {
            foreach (var list in _lists)
                foreach (var x in list)
                    yield return x;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public int Count
        {
            get
            {
                var count = 0;
                foreach (var l in _lists)
                    count += l.Count;
                return count;
            }
        }

        public bool IsReadOnly => false;

        public void Add(T item) { throw new NotSupportedException("ListView is fixed-size"); }

        public void Clear() { throw new NotSupportedException("ListView is fixed-size"); }

        public bool Contains(T item)
        {
            var comp = EqualityComparer<T>.Default;
            for (var i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return true;
            return false;
        }

        public void CopyTo(T[] array, int arrayIndex)
        {
            for (int i = arrayIndex, j = 0; j < Count; j++, i++)
                array[i] = this[j];
        }

        public bool Remove(T item) { throw new NotSupportedException("ListView is fixed-size"); }

        public T this[int index]
        {
            get
            {
                var i = get_list_index(ref index);
                return _lists[i][index];
            }
            set
            {
                var i = get_list_index(ref index);
                _lists[i][index] = value;
            }
        }

        private int _prevQuery = int.MaxValue;
        private int _prevListIndex = 0;
        private int _prevIndexShift = 0;

        private int get_list_index(ref int linIndex)
        {
            if (_lists.Count == 0)
                throw new InvalidOperationException("ListView is empty");
            var i = 0;
            if (linIndex < _prevQuery)
            {
                _prevQuery = linIndex;
                _prevIndexShift = 0;
                while (_lists[i].Count <= linIndex)
                {
                    _prevIndexShift += _lists[i].Count;
                    linIndex -= _lists[i].Count;
                    i++;
                    if (i >= _lists.Count)
                        throw new InvalidOperationException("index too large");
                }
            }
            else
            {
                i = _prevListIndex;
                _prevQuery = linIndex;
                linIndex -= _prevIndexShift;
                while (_lists[i].Count <= linIndex)
                {
                    _prevIndexShift += _lists[i].Count;
                    linIndex -= _lists[i].Count;
                    i++;
                    if (i >= _lists.Count)
                        throw new InvalidOperationException("index too large");
                }
            }
            _prevListIndex = i;
            return i;
        }

        public void reset_cached_index()
        {
            _prevQuery = int.MaxValue;
            _prevListIndex = 0;
            _prevIndexShift = 0;
        }

        public int IndexOf(T item)
        {
            var comp = EqualityComparer<T>.Default;
            for (var i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return i;
            return -1;
        }

        public void Insert(int index, T item) { throw new NotSupportedException("ListView is fixed-size"); }

        public void RemoveAt(int index) { throw new NotSupportedException("ListView is fixed-size"); }

        public override string ToString()
        {
            var result = "{" + string.Join(", ", this.Select(x => x.ToString()).ToArray()) + "}";
            return result;
        }
    }

}