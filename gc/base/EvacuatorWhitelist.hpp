/*******************************************************************************
 * Copyright (c) 2018, 2018 IBM Corp. and others
 *
 * This program and the accompanying materials are made available under
 * the terms of the Eclipse Public License 2.0 which accompanies this
 * distribution and is available at https://www.eclipse.org/legal/epl-2.0/
 * or the Apache License, Version 2.0 which accompanies this distribution and
 * is available at https://www.apache.org/licenses/LICENSE-2.0.
 *
 * This Source Code may also be made available under the following
 * Secondary Licenses when the conditions for such availability set
 * forth in the Eclipse Public License, v. 2.0 are satisfied: GNU
 * General Public License, version 2 with the GNU Classpath
 * Exception [1] and GNU General Public License, version 2 with the
 * OpenJDK Assembly Exception [2].
 *
 * [1] https://www.gnu.org/software/classpath/license.html
 * [2] http://openjdk.java.net/legal/assembly-exception.html
 *
 * SPDX-License-Identifier: EPL-2.0 OR Apache-2.0 OR GPL-2.0 WITH Classpath-exception-2.0 OR LicenseRef-GPL-2.0 WITH Assembly-exception
 *******************************************************************************/

#ifndef EVACUATORWHITELIST_HPP_
#define EVACUATORWHITELIST_HPP_

#if defined(EVACUATOR_DEBUG)
#include <string.h> // for memset
#endif /* defined(EVACUATOR_DEBUG) */

#include "AtomicSupport.hpp"
#include "Base.hpp"
#include "EnvironmentBase.hpp"
#include "EvacuatorBase.hpp"
#include "GCExtensionsBase.hpp"
#include "HeapLinkedFreeHeader.hpp"
#include "MemoryPoolAddressOrderedList.hpp"
#include "MemorySubSpace.hpp"
#include "ObjectModelBase.hpp"
#include "ScavengerStats.hpp"
/**
 * Free space reserved from survivor or tenure to receive matter copied from evacuation space is represented as a
 * heap linked free header to ensure the heap is walkable. For nontrivial (length > sizeof(MM_HeapLinkedFreeHeader))
 * whitespace a 64-bit word is installed to hold status flags and an eye catcher pattern as an aid to debugging. The
 * eye catcher pattern is inverted when/if the whitespace is reclaimed for reuse by the evacuator.
 */
class MM_EvacuatorWhitespace : public MM_HeapLinkedFreeHeader {
	/*
	 * Data members
	 */
private:
	/* eyecatcher/loa flag bits are included only for multislot whitespace (ie, when sizeof(MM_EvacuatorWhitespace) <= MM_HeapLinkedFreeHeader::_size) */
	static const uint32_t _eyeCatcher = 0x5353535C;

	typedef enum WhitespaceFlags {
		flagLOA = 1
		, flagDiscarded = 2
	} WhitespaceFlags;

	/* four low order bits are 0xCxy store discarded x=~a and LOA y=~b bits, the high order bits are all eyecatcher 535353(xy) or inverted ACACAC(ab) when reused */
	uint32_t _flags;
	uint32_t _pad64;
protected:
public:
	/*
	 * Function members
	 */
private:
protected:
public:
	static MM_EvacuatorWhitespace *
	whitespace(void *address, uintptr_t length, bool isLOA = false)
	{
		/* trivial whitespace (length < sizeof(MM_EvacuatorWhitespace)) is always discarded as a heap hole */
		MM_EvacuatorWhitespace *freespace = (MM_EvacuatorWhitespace *)fillWithHoles(address, length);
		if (sizeof(MM_EvacuatorWhitespace) <= length) {
			freespace->_flags = _eyeCatcher | (isLOA ? (uint32_t)flagLOA : 0);
		} else {
			freespace = NULL;
		}
		return freespace;
	}

	uintptr_t length() { return _size; }

	uint8_t *getBase() { return (uint8_t *)this; }

	uint8_t *getEnd() { return getBase() + length(); }

	bool isLOA() { return (sizeof(MM_EvacuatorWhitespace) <= length()) && ((uint32_t)flagLOA == ((uint32_t)flagLOA & _flags)); }

	bool isReused() { return (sizeof(MM_EvacuatorWhitespace) > length()) || (_eyeCatcher != (_eyeCatcher & _flags)); }

	bool isDiscarded() { return (sizeof(MM_EvacuatorWhitespace) > length()) || ((uint32_t)flagDiscarded == ((uint32_t)flagDiscarded & _flags)); }

	void
	clear()
	{
		/* this whitespace is being reused by the evacuator */
		if (sizeof(MM_EvacuatorWhitespace) <= length()) {
			/* flip the eyecatcher bits to ACACACACACACACACAx where (x & 3) preserves discarded and LOA flags */
			_flags ^= ~(uint32_t)0x3;
		}
		_next = 0;
	}

	void
	discard()
	{
		/* this whitespace is being discarded by the evacuator without reuse */
		if (sizeof(MM_EvacuatorWhitespace) <= length()) {
			/* set the discarded bit to indicate that whitespace was discarded */
			_flags |= (uint32_t)flagDiscarded;
		}
	}

#if defined(EVACUATOR_DEBUG)
	static void
	poison(MM_EvacuatorWhitespace *whitespace)
	{
		if (whitespace->length() > sizeof(MM_EvacuatorWhitespace)) {
			memset((uint8_t*)whitespace + sizeof(MM_EvacuatorWhitespace), 0x77, whitespace->length() - sizeof(MM_EvacuatorWhitespace));
		}
	}
#endif /* defined(EVACUATOR_DEBUG) */
};

/**
 *  Bounded priority queue of free space, pointer to largest on top at _whitelist[0]. Elements
 *  added at capacity will force a smaller element to be dropped, dropped elements are converted
 *  to holes in the runtime heap.
 */
class MM_EvacuatorWhitelist : public MM_Base
{
/*
 * Data members
 */
private:
	MM_EvacuatorWhitespace *_whitelist[MM_EvacuatorBase::max_whitelist];	/* array of pointers to free space */
	MM_EvacuatorWhitespace *_tail;											/* largest discarded whitespace, add() can discard smaller whitespace without sifting */
	uintptr_t _count;														/* number of active free space elements in array */
	uintptr_t _volume;														/* current sum of bytes available as whitespace */
	uintptr_t _discarded;													/* cumulative sum of bytes discarded as heap holes */
	uintptr_t _flushed;														/* cumulative sum of bytes flushed as heap holes */
	MM_EnvironmentBase * _env;												/* for port library access for discard trace */
	MM_MemorySubSpace *_subspace;											/* memory subspace receives discarded fragments */
	MM_MemoryPoolAddressOrderedList *_memoryPool;							/* HACK: this assumes AOL memory pool */
	MM_ScavengerStats *_stats;												/* pointer to _env->_scavengerStats */
	uintptr_t _index;														/* evacuator worker index for discard trace */
	bool _tenure;															/* true if managing tenure whitespace */
#if defined(EVACUATOR_DEBUG)
	MM_EvacuatorBase *_debugger;											/* from controller */
#endif /* defined(EVACUATOR_DEBUG) */

protected:
public:

/*
 * Function members
 */
private:
	bool odd(uintptr_t n) { return (1 == (n & (uintptr_t)1)); }

	/* left and right children of element n */
	uintptr_t left(uintptr_t n) { return (n << 1) + 1; }
	uintptr_t right(uintptr_t n) { return (n << 1) + 2; }

	/* parent of element n -- parent(0) is undefined */
	uintptr_t parent(uintptr_t n) { return (n - 1) >> 1; }

	/* comparators for free space pointers in array */
	bool lt(uintptr_t a, uintptr_t b) { return _whitelist[a]->length() < _whitelist[b]->length(); }
	bool le(uintptr_t a, uintptr_t b) { return lt(a, b) || (_whitelist[a]->length() == _whitelist[b]->length()); }

	/* swap free space pointers in array */
	void swap(uintptr_t a, uintptr_t b)
	{
		MM_EvacuatorWhitespace *temp = _whitelist[a];
		_whitelist[a] = _whitelist[b];
		_whitelist[b] = temp;
	}

#if defined(EVACUATOR_DEBUG)
	void
	clean()
	{
		for (uintptr_t i = 0; i < _count; i += 1) {
			Debug_MM_true(_env->getExtensions()->objectModel.isDeadObject((void *)_whitelist[i]));
			Debug_MM_true(MM_EvacuatorBase::max_scanspace_remainder < _whitelist[i]->length());
			Debug_MM_true(NULL == _whitelist[i]->getNext());
		}
		if (NULL != _tail) {
			Debug_MM_true(_env->getExtensions()->objectModel.isDeadObject((void *)_tail));
			Debug_MM_true(MM_EvacuatorBase::max_scanspace_remainder < _tail->length());
			Debug_MM_true(NULL == _tail->getNext());
		}
	}

	void
	debug(MM_EvacuatorWhitespace *whitespace, const char* op)
	{
		if (_debugger->isDebugWhitelists()) {
			OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
			char buf[512];
			uintptr_t sum = 0;
			uintptr_t len = omrstr_printf(buf, 512, "%5lu    %2llu:%7s[%c]; address:%llx; length:%llx; count:%llx; volume:%llx; top:%llx; tail:%llx; discarded:%llx; flushed:%llx;",
					_env->_scavengerStats._gcCount, (uint64_t)_index, op, (_tenure ? 'T' : 'S'), (uint64_t)whitespace, (uint64_t)((NULL !=  whitespace) ? whitespace->length() : 0), (uint64_t)_count, (uint64_t)_volume,
					(uint64_t)((NULL !=  _whitelist[0]) ? _whitelist[0]->length() : 0), (uint64_t)((NULL != _tail) ? _tail->length() : 0), (uint64_t)_discarded, (uint64_t)_flushed);
			for (uintptr_t i = 0; i < _count; i++) {
				len += omrstr_printf(buf + len, 512 - len, " %llx", (uint64_t)_whitelist[i]->length());
				sum += _whitelist[i]->length();
			}
			buf[len] = 0;
			omrtty_printf("%s\n", buf);
			Debug_MM_true(sum == _volume);
		}
	}
#endif /* defined(EVACUATOR_DEBUG) */

	void
	siftDown()
	{
		uintptr_t pos = 0;
		uintptr_t end = _count >> 1;
		while (pos < end) {
			uintptr_t l = left(pos);
			uintptr_t next = l;
			uintptr_t r = right(pos);
			if ((r < _count) && lt(l, r)) {
				next = r;
			}
			if (lt(pos, next)) {
				swap(pos, next);
				pos = next;
			} else {
				break;
			}
		}
	}

	void
	siftUp(uintptr_t bottom)
	{
		uintptr_t pos = bottom;
		while (0 < pos) {
			uintptr_t next = parent(pos);
			if (lt(pos, next)) {
				break;
			}
			swap(pos, next);
			pos = next;
		}
	}

	void
	discard(MM_EvacuatorWhitespace *discard, bool flushing = false)
	{
		uintptr_t discarded = (NULL != discard) ? discard->length() : 0;
		if (0 < discarded) {
			/* tail holds largest discard from the whitelist (ie, tail is not longer than any whitespace in the whitelist) */
			if (!flushing && (discard->length() > MM_EvacuatorBase::max_scanspace_remainder) &&
					((NULL == _tail) || (discard->length() > _tail->length()))
			) {
				/* swap hole into tail and set up previous tail for discard */
				MM_EvacuatorWhitespace *tail = _tail;
				_tail = discard;
				if (NULL != tail) {
					discarded = tail->length();
					discard = tail;
				} else {
					return;
				}
			}
			discard->discard();

			/* recycle or fill discards with holes to keep runtime heap walkable */
			void *top = (void *)((uintptr_t)discard + discarded);
			if (_tenure || !flushing || (MM_EvacuatorBase::min_recyclable_whitespace > discarded)) {
				_subspace->abandonHeapChunk(discard, top);
			} else if ((0 == discarded) || _memoryPool->recycleHeapChunk(discard, top)) {
				return;
			}

			if (flushing) {
				_flushed += discarded;
			} else {
				_discarded += discarded;
			}

#if defined(EVACUATOR_DEBUG)
			verify();
			debug(discard, flushing ? "flush" : "discard");
#endif /* defined(EVACUATOR_DEBUG) */
		}
	}

protected:
public:
	/**
	 * Basic array constructor obviates need for stdlibc++ linkage in gc component libraries. Array
	 * is allocated from forge as contiguous block sized to contain requested number of elements and
	 * must be freed using MM_Forge::free() when no longer needed. See MM_Evacuator::tearDown().
	 *
	 * @param count the number of aray elements to instantiate
	 * @return a pointer to instantiated array
	 */
	static MM_EvacuatorWhitelist *
	newInstanceArray(MM_Forge *forge, uintptr_t count)
	{
		MM_EvacuatorWhitelist *whitelist = (MM_EvacuatorWhitelist *)forge->allocate(sizeof(MM_EvacuatorWhitelist) * count, OMR::GC::AllocationCategory::FIXED, OMR_GET_CALLSITE());
		if (NULL != whitelist) {
			for (uintptr_t i = 0; i < count; i += 1) {
				MM_EvacuatorWhitelist *list = new(whitelist + i) MM_EvacuatorWhitelist();
				if (NULL == list) {
					return NULL;
				}
			}
		}
		return whitelist;
	}

	/**
	 * Returns the number of whitespace elements in the list
	 */
	uintptr_t getSize() { return _count; }

	/**
	 * Returns the number of whitespace bytes discarded (filled with holes)
	 */
	uintptr_t getDiscarded() { return VM_AtomicSupport::lockCompareExchange(&_discarded, _discarded, _discarded); }

	/**
	 * Returns the number of whitespace bytes discarded (filled with holes)
	 */
	uintptr_t getFlushed() { return VM_AtomicSupport::lockCompareExchange(&_flushed, _flushed, _flushed); }

	/**
	 * Get the length of largest whitespace at top of whitelist
	 */
	uintptr_t top() { return (0 < _count) ? _whitelist[0]->length() : 0; }

	/**
	 * Takes largest whitespace from top and sifts down a small one from end of list to restore largest on top
	 *
	 * @param length the minimum number of bytes of whitespace required
	 * @return whitespace with required capacity (length) or NULL if nothing available
	 */
	MM_EvacuatorWhitespace *
	top(uintptr_t length)
	{
		MM_EvacuatorWhitespace *freespace = NULL;
		if ((0 < _count) && (_whitelist[0]->length() >= length)) {
			MM_EvacuatorWhitespace *next = _tail;
			if (NULL == _tail) {
				_count -= 1;
				if (0 < _count) {
					next = _whitelist[_count];
					_whitelist[_count] = NULL;
				}
			} else {
				_volume += _tail->length();
				_tail = NULL;
			}
			freespace = _whitelist[0];
			_volume -= freespace->length();
			freespace->clear();
			_whitelist[0] = next;
			siftDown();
#if defined(EVACUATOR_DEBUG)
			verify();
			debug(freespace, "-white");
#endif /* defined(EVACUATOR_DEBUG) */
		}
		return freespace;
	}

	/**
	 * Tries to add a new free space element and sift it up the queue. It will be discarded
	 * if too small to include in current whitelist.
	 *
	 * @param whitespace points to head of free space to add
	 * @param length indicates size in bytes of free space
	 */
	void
	add(MM_EvacuatorWhitespace *freespace)
	{
		if (NULL != freespace) {
#if defined(EVACUATOR_DEBUG)
			Debug_MM_true(_tenure == _env->getExtensions()->isOld((omrobjectptr_t)freespace));
			MM_EvacuatorWhitespace *address = freespace;
#endif /* defined(EVACUATOR_DEBUG) */
			uintptr_t length = freespace->length();
			/* any dropped free space bytes will be linked into a heap free header to keep heap walkable */
			if (((NULL != _tail) ? _tail->length() : MM_EvacuatorBase::max_scanspace_remainder) < length) {
				/* assume the whitelist is not full and freespace will be appended at next leaf */
				uintptr_t pos = _count;
				if (_count == MM_EvacuatorBase::max_whitelist) {
					/* whitelist is full -- find smallest leaf */
					uintptr_t min = length;
					for (uintptr_t j = _count >> 1; j < _count; j += 1) {
						if (_whitelist[j]->length() < min) {
							min = _whitelist[j]->length();
							pos = j;
						}
					}
					/* swap freespace into smallest leaf if freespace is larger, displacing smallest leaf for discarding */
					if (min < length) {
						MM_EvacuatorWhitespace *leafspace = _whitelist[pos];
						_whitelist[pos] = freespace;
						_volume -= leafspace->length();
						freespace = leafspace;
					}
				} else {
					/* whitelist not full -- append freespace without displacement */
					_whitelist[pos] = freespace;
					freespace = NULL;
					_count += 1;
				}
				/* if new free space was not discarded, sift it up the heap */
				if (pos < _count) {
					siftUp(pos);
					_volume += length;
#if defined(EVACUATOR_DEBUG)
					verify();
					debug(address, "+white");
#endif /* defined(EVACUATOR_DEBUG) */
				}
			}
			if (NULL != freespace) {
				discard(freespace);
			}
		}
	}

	/**
	 * Discards (recycles or fills with holes) all whitespace in current whitelist.
	 */
	uintptr_t
	flush()
	{
		MM_EvacuatorWhitespace *whitespace = top(0);
		while (NULL != whitespace) {
			Debug_MM_true(MM_EvacuatorBase::max_scanspace_remainder < whitespace->length());
			discard(whitespace, true);
			whitespace = top(0);
		}
		if (NULL != _tail) {
			Debug_MM_true(MM_EvacuatorBase::max_scanspace_remainder < _tail->length());
			discard(_tail, true);
			_tail = NULL;
		}
		Debug_MM_true((0 == _count) && (0 == _volume));

		uintptr_t flushed = _flushed;
		if (_tenure) {
			_stats->_tenureDiscardBytes += _flushed;
			_flushed = 0;
		} else {
			_stats->_flipDiscardBytes += _flushed;
		}

		return flushed;
	}

	void
	bind(MM_EvacuatorBase *debugger, MM_EnvironmentBase *env, uintptr_t evacuatorIndex, MM_MemorySubSpace *subspace, bool isTenure)
	{
		_env = env;
		_index = evacuatorIndex;
		_subspace = subspace;
		_memoryPool = (MM_MemoryPoolAddressOrderedList *)_subspace->getMemoryPool();
		_stats = &_env->_scavengerStats;
		_discarded = 0;
		if (!isTenure) {
			_tail = NULL;
			_count = 0;
			_volume = 0;
			_flushed = 0;
		}
		_tenure = isTenure;
#if defined(EVACUATOR_DEBUG)
		_debugger = debugger;
		if (!_tenure) {
			for (uintptr_t i = 0; i < MM_EvacuatorBase::max_whitelist; i++) {
				Debug_MM_true(NULL == _whitelist[i]);
			}
		} else {
			clean();
		}
#endif /* defined(EVACUATOR_DEBUG) */
	}

#if defined(EVACUATOR_DEBUG)
	void
	verify()
	{
		uintptr_t volume = 0;
		if (_debugger->isDebugWhitelists()) {
			Debug_MM_true((MM_EvacuatorBase::max_whitelist == _count) || (NULL == _tail));
			Debug_MM_true(((0 == _count) && (NULL == _whitelist[0])) || (DEFAULT_SCAN_CACHE_MAXIMUM_SIZE >= _whitelist[0]->length()));
			for (uintptr_t i = 0; i < _count; i += 1) {
				Debug_MM_true(NULL != _whitelist[i]);
				Debug_MM_true(0  == ((sizeof(_whitelist[i]->length()) - 1) & _whitelist[i]->length()));
				Debug_MM_true(MM_EvacuatorBase::max_scanspace_remainder < _whitelist[i]->length());
				Debug_MM_true(_env->getExtensions()->objectModel.isDeadObject((void *)_whitelist[i]));
				Debug_MM_true(_env->getExtensions()->objectModel.getSizeInBytesDeadObject((omrobjectptr_t )_whitelist[i]) == _whitelist[i]->length());
				Debug_MM_true(_tenure == _env->getExtensions()->isOld((omrobjectptr_t)_whitelist[i]));
				Debug_MM_true(!_whitelist[i]->isReused());
				Debug_MM_true(!_whitelist[i]->isDiscarded());
				Debug_MM_true((0 == i) || le(i, parent(i)));
				volume += _whitelist[i]->length();
			}
			uintptr_t end = _count >> 1;
			for (uintptr_t j = 0; j < end; j += 1) {
				Debug_MM_true(le(left(j), j));
				Debug_MM_true((right(j) >=_count) || le(right(j), j));
				Debug_MM_true((NULL == _tail) || (_whitelist[j]->length() >= _tail->length()));
			}
			Debug_MM_true(volume == _volume);
		}
	}
#endif /* defined(EVACUATOR_DEBUG) */

	/**
	 * Constructor
	 */
	MM_EvacuatorWhitelist()
		: MM_Base()
		, _tail(NULL)
		, _count(0)
		, _volume(0)
		, _discarded(0)
		, _flushed(0)
		, _env(NULL)
		, _subspace(NULL)
		, _memoryPool(NULL)
		, _stats(NULL)
		, _index(0)
		, _tenure(false)
#if defined(EVACUATOR_DEBUG)
		, _debugger(NULL)
#endif /* defined(EVACUATOR_DEBUG) */
	{
		for (uintptr_t i = 0; i < MM_EvacuatorBase::max_whitelist; i++) {
			_whitelist[i] = NULL;
		}
	}
};

#endif /* EVACUATORWHITELIST_HPP_ */
