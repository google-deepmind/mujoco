// Copyright 2019 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections;
using System.Collections.Generic;

namespace Mujoco {
  // An implementation of IList<T>, only missing the indexer and the count.
  public abstract class FixedSizeIListHelper<T> : IList<T> {
    public IEnumerator<T> GetEnumerator() {
      for (int i = 0; i < this.Count; ++i) {
        yield return this[i];
      }
    }

    IEnumerator IEnumerable.GetEnumerator() {
      return GetEnumerator();
    }

    public void Add(T item) {
      throw new NotSupportedException();
    }

    public void Clear() {
      throw new NotSupportedException();
    }

    public bool Contains(T item) {
      for (int i = 0; i < Count; ++i) {
        if (this[i].Equals(item)) {
          return true;
        }
      }
      return false;
    }

    public void CopyTo(T[] array, int arrayIndex) {
      for (int i = 0; i < Count; ++i) {
        array[arrayIndex + i] = this[i];
      }
    }

    public bool Remove(T item) {
      throw new NotSupportedException();
    }

    public bool IsReadOnly => false;

    public int IndexOf(T item) {
      for (int i = 0; i < Count; ++i) {
        if (this[i].Equals(item)) {
          return i;
        }
      }
      return -1;
    }

    public void Insert(int index, T item) {
      throw new NotSupportedException();
    }

    public void RemoveAt(int index) {
      throw new NotSupportedException();
    }

    public abstract int Count { get; }
    public abstract T this[int index] {
      get;
      set;
    }
  }
}
