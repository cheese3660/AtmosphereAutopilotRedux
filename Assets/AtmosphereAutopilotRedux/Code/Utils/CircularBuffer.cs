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

using System.Collections;
using System.Collections.Generic;
using System.Threading;

namespace System.Collections.Generic
{
    public class CircularBufferAA<T> : IList<T>, ICollection<T>, IEnumerable<T>, ICollection, IEnumerable
    {
        private int _capacity;       // underlying array size
        private int _size;           // current count of elements
        private int _head;           // index of first element
        private int _tail;           // index of element behind the last one
        private T[] _buffer;         // underlying array

        [NonSerialized]
        private object _syncRoot;

        public CircularBufferAA(int capacity) : this(capacity, false) { }

        public CircularBufferAA(int capacity, bool allowOverflow)
        {
            if (capacity <= 0)
                throw new ArgumentException("capacity must be greater than zero.",
                    "capacity");

            _capacity = capacity;
            _size = 0;
            _head = 0;
            _tail = 0;
            _buffer = new T[capacity];
            AllowOverflow = allowOverflow;
        }

        public CircularBufferAA(int capacity, bool allowOverflow, T firstValue) : this(capacity, allowOverflow)
        {
            Put(firstValue);   
        }

        public bool AllowOverflow
        {
            get;
            set;
        }

        public int Capacity
        {
            get => _capacity;
            set
            {
                if (value == _capacity)
                    return;

                if (value < _size)
                    throw new ArgumentOutOfRangeException("value",
                        "value must be greater than or equal to the buffer size.");

                var dst = new T[value];
                if (_size > 0)
                    CopyTo(dst);
                _buffer = dst;

                _capacity = value;
            }
        }

        public int Size => _size;

        public bool Contains(T item)
        {
            var bufferIndex = _head;
            var comparer = EqualityComparer<T>.Default;
            for (var i = 0; i < _size; i++, bufferIndex++)
            {
                if (bufferIndex == _capacity)
                    bufferIndex = 0;

                if (item == null && _buffer[bufferIndex] == null)
                    return true;
                else if ((_buffer[bufferIndex] != null) &&
                    comparer.Equals(_buffer[bufferIndex], item))
                    return true;
            }

            return false;
        }

        public void Clear()
        {
            _size = 0;
            _head = 0;
            _tail = 0;
        }

        public int Put(T[] src)
        {
            return Put(src, 0, src.Length);
        }

        public int Put(T[] src, int offset, int count)
        {
            var realCount = AllowOverflow ? count : Math.Min(count, _capacity - _size);
            var srcIndex = offset;
            for (var i = 0; i < realCount; i++, srcIndex++)
            {
                if (_tail == _capacity)
                    _tail = 0;
                _buffer[_tail] = src[srcIndex];
                _tail = (_tail + 1) % _capacity;
            }
            _size = Math.Min(_size + realCount, _capacity);
            return realCount;
        }

        public void Put(T item)
        {
            if (_size >= _capacity)
                if (!AllowOverflow)
                    throw new IndexOutOfRangeException("Buffer is full.");
                else 
                {
                    _buffer[_tail] = item;
                    _tail = (_tail + 1) % _capacity;
                    _head = _tail;
                    return;
                }
            _size++;
            _buffer[_tail] = item;
            _tail = (_tail + 1) % _capacity;
        }

        public T GetWritingCell()
        {
            return _buffer[_tail];
        }

        public T GetFromTail(int shift)
        {
            if (shift >= _size)
                throw new IndexOutOfRangeException("shift >= size = " + _size.ToString());
            return _buffer[_capacity - 1 - ((_capacity - _tail + shift) % _capacity)];
        }

        public T this[int index]
        {
            get => _buffer[(index + _head) % _capacity];
            set => _buffer[(index + _head) % _capacity] = value;
        }

        public T GetLast()
        {
            if (_size <= 0)
                return _buffer[0];
            else
                return this[_size - 1];
        }

        public void Skip(int count)
        {
            _head += count;
            if (_head >= _capacity)
                _head -= _capacity;
        }

        public T[] Get(int count)
        {
            var dst = new T[count];
            Get(dst);
            return dst;
        }

        public int Get(T[] dst)
        {
            return Get(dst, 0, dst.Length);
        }

        public int Get(T[] dst, int offset, int count)
        {
            var realCount = Math.Min(count, _size);
            var dstIndex = offset;
            for (var i = 0; i < realCount; i++, _head++, dstIndex++)
            {
                if (_head == _capacity)
                    _head = 0;
                dst[dstIndex] = _buffer[_head];
            }
            _size -= realCount;
            return realCount;
        }

        public T Get()
        {
            if (_size == 0)
                throw new InvalidOperationException("Buffer is empty.");

            var item = _buffer[_head];
            if (++_head == _capacity)
                _head = 0;
            _size--;
            return item;
        }

        public void CopyTo(T[] array)
        {
            CopyTo(array, 0);
        }

        public void CopyTo(T[] array, int arrayIndex)
        {
            CopyTo(0, array, arrayIndex, _size);
        }

        public void CopyTo(int index, T[] array, int arrayIndex, int count)
        {
            if (count > _size)
                throw new ArgumentOutOfRangeException("count",
                    "count cannot be greater than the buffer size.");

            var bufferIndex = _head;
            for (var i = 0; i < count; i++, bufferIndex++, arrayIndex++)
            {
                if (bufferIndex == _capacity)
                    bufferIndex = 0;
                array[arrayIndex] = _buffer[bufferIndex];
            }
        }

        public IEnumerator<T> GetEnumerator()
        {
            var bufferIndex = _head;
            for (var i = 0; i < _size; i++, bufferIndex++)
            {
                if (bufferIndex == _capacity)
                    bufferIndex = 0;

                yield return _buffer[bufferIndex];
            }
        }

        public T[] GetBuffer()
        {
            return _buffer;
        }

        public T[] ToArray()
        {
            var dst = new T[_size];
            CopyTo(dst);
            return dst;
        }

        # region IList<T> Members

        public int IndexOf(T item)
        {
            var bufferIndex = _head;
            var comparer = EqualityComparer<T>.Default;
            for (var i = 0; i < _size; i++, bufferIndex++)
            {
                if (bufferIndex == _capacity)
                    bufferIndex = 0;
                if (item == null && _buffer[bufferIndex] == null)
                    return bufferIndex;
                else if ((_buffer[bufferIndex] != null) &&
                    comparer.Equals(_buffer[bufferIndex], item))
                    return bufferIndex;
            }
            return -1;
        }

        public void Insert(int index, T item)
        {
            if (index > _size || index < 0)
                throw new ArgumentOutOfRangeException("index", "CircularBufferAA Insert at index > size");
            if (index == _size)
                Put(item);
            else
            {
                for (var i = 0; i < index; i++)
                    this[i] = this[i + 1];
                this[index] = item;
            }
        }

        public void RemoveAt(int index)
        {
            if (index >= _size || index < 0)
                throw new ArgumentOutOfRangeException("index", "CircularBufferAA RemoveAt index >= size");
            if (index != _size -1)
            {
                for (var i = index; i < _size; i++)
                    this[i] = this[i + 1];
            }
            _tail = (_tail - 1 + _capacity) % _capacity;
            _size--;
        }

        #endregion

        #region ICollection<T> Members

        int ICollection<T>.Count => Size;

        bool ICollection<T>.IsReadOnly => false;

        void ICollection<T>.Add(T item)
        {
            Put(item);
        }

        bool ICollection<T>.Remove(T item)
        {
            if (_size == 0)
                return false;

            Get();
            return true;
        }

        #endregion

        #region IEnumerable<T> Members

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return GetEnumerator();
        }

        #endregion

        #region ICollection Members

        int ICollection.Count => Size;

        bool ICollection.IsSynchronized => false;

        object ICollection.SyncRoot
        {
            get
            {
                if (_syncRoot == null)
                    Interlocked.CompareExchange(ref _syncRoot, new object(), null);
                return _syncRoot;
            }
        }

        void ICollection.CopyTo(Array array, int arrayIndex)
        {
            CopyTo((T[])array, arrayIndex);
        }

        #endregion

        #region IEnumerable Members

        IEnumerator IEnumerable.GetEnumerator()
        {
            return (IEnumerator)GetEnumerator();
        }

        #endregion
    }
}