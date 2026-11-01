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
using System.Collections;
using System.Linq;
using System.Text;

namespace AtmosphereAutopilotRedux.Utils
{

    // IList adapter
    public class ListSelector<St, RT> : IList<RT>, ICollection<RT>, IEnumerable<RT>, IEnumerable
    {
        private readonly IList<St> _source;
        private readonly Func<St, RT> _selector;

        public ListSelector(IList<St> source, Func<St, RT> selector)
        {
            _source = source;
            _selector = selector;
        }

        public IEnumerator<RT> GetEnumerator()
        {
            foreach (var x in _source)
                yield return _selector(x);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public int Count => _source.Count;

        public bool IsReadOnly => true;

        public void Add(RT item) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public void Clear() { throw new NotSupportedException("ListSelector is fixed-size"); }

        public bool Contains(RT item)
        {
            var comp = EqualityComparer<RT>.Default;
            for (var i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return true;
            return false;
        }

        public void CopyTo(RT[] array, int arrayIndex)
        {
            for (int i = arrayIndex, j = 0; j < Count; j++, i++)
                array[i] = this[j];
        }

        public bool Remove(RT item) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public RT this[int index]
        {
            get => _selector(_source[index]);
            set => throw new NotSupportedException("ListSelector is read-only");
        }

        public int IndexOf(RT item)
        {
            var comp = EqualityComparer<RT>.Default;
            for (var i = 0; i < Count; i++)
                if (comp.Equals(this[i], item))
                    return i;
            return -1;
        }

        public void Insert(int index, RT item) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public void RemoveAt(int index) { throw new NotSupportedException("ListSelector is fixed-size"); }

        public override string ToString()
        {
            var result = "{" + string.Join(", ", this.Select(x => x.ToString()).ToArray()) + "}";
            return result;
        }

    }
}
