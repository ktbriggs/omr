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
		MM_EvacuatorWhitespace *space = (MM_EvacuatorWhitespace *)fillWithHoles(address, length);
		if (sizeof(MM_EvacuatorWhitespace) <= length) {
			space->_flags = _eyeCatcher | (isLOA ? (uint32_t)flagLOA : 0);
		} else {
			space = NULL;
		}
		return space;
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

	uintptr_t
	discard()
	{
		/* this whitespace is being discarded by the evacuator without reuse */
		if (sizeof(MM_EvacuatorWhitespace) <= length()) {
			/* set the discarded bit to indicate that whitespace was discarded */
			_flags |= (uint32_t)flagDiscarded;
		}

		return length();
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
	uintptr_t _count;														/* number of active free space elements in array */
	uintptr_t _volume;														/* current sum of bytes available as whitespace */
	uintptr_t _discarded;													/* cumulative sum of bytes discarded as heap holes */
	uintptr_t _flushed;														/* cumulative sum of bytes flushed as heap holes */
	uintptr_t _recycled;													/* cumulative sum of bytes recycled into memory pool */
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
	/* left and right inferiors and superior of element n -- up(0) is undefined */
	uintptr_t left(uintptr_t n) { return (n << 1) + 1; }
	uintptr_t right(uintptr_t n) { return (n << 1) + 2; }
	uintptr_t up(uintptr_t n) { return (n - 1) >> 1; }

	/* comparator for free space pointers in array */
	bool lt(uintptr_t a, uintptr_t b) { return _whitelist[a]->length() < _whitelist[b]->length(); }

	/* swap free space pointers in array */
	void swap(uintptr_t a, uintptr_t b)
	{
		MM_EvacuatorWhitespace *temp = _whitelist[a];
		_whitelist[a] = _whitelist[b];
		_whitelist[b] = temp;
	}

	void
	siftDown()
	{
		uintptr_t pos = 0;
		uintptr_t end = _count >> 1;
		while (pos < end) {
			uintptr_t l = left(pos);
			uintptr_t r = right(pos);
			uintptr_t next = ((r < _count) && lt(l, r)) ? r : l;
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
			uintptr_t next = up(pos);
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
		uintptr_t length = (NULL != discard) ? discard->discard() : 0;
		void *top = (void *)((uintptr_t)discard + length);

		/* recycle survivor discards whenever possible, always abandon tenure discards */
		if (!_tenure && (_memoryPool->getMinimumFreeEntrySize() <= length) && _memoryPool->recycleHeapChunk(discard, top)) {

			_recycled += length;
			length = 0;
		}

		/* abandon discards when not recycled and fill with holes to keep runtime heap walkable */
		if (0 < length) {

			_subspace->abandonHeapChunk(discard, top);

			/* local discard counts are aggregate for survivor + tenure regions */
			if (flushing) {
				/* intentional discard */
				_flushed += length;
			} else {
				/* incidental discard  */
				_discarded += length;
			}

			/* regional stats for all discards */
			if (_tenure) {
				_stats->_tenureDiscardBytes += length;
			} else {
				_stats->_flipDiscardBytes += length;
			}
		}

#if defined(EVACUATOR_DEBUG)
		verify();
		debug(discard, flushing ? "flush" : "discard");
#endif /* defined(EVACUATOR_DEBUG) */
	}

protected:
public:
	/**
	 * Get the length of largest whitespace at top of whitelist
	 */
	uintptr_t top() { return (0 < _count) ? _whitelist[0]->length() : 0; }

	/**
	 * Takes largest whitespace from top and sifts down a small one from end of list to restore largest on top
	 *
	 * @param length the minimum number of bytes of whitespace required
	 * @param swap optional pointer to whitespace with size < length to swap into whitelist
	 * @return whitespace with required capacity (length) or NULL if nothing available
	 */
	MM_EvacuatorWhitespace *
	top(uintptr_t length, MM_EvacuatorWhitespace *swap = NULL)
	{
		Debug_MM_true((NULL == swap) || (swap->length() < length));

		/* try to take whitespace from the top of the whitelist */
		MM_EvacuatorWhitespace * const whitespace = ((0 < _count) && (_whitelist[0]->length() >= length)) ? _whitelist[0] : NULL;
		if (NULL != whitespace) {

			/* discard unviable residual whitespace */
			if ((NULL != swap) && (MM_EvacuatorBase::min_reusable_whitespace >= swap->length())) {
				discard(swap);
				swap = NULL;
			}

			/* detach whitespace from whitelist */
			_volume -= whitespace->length();
			whitespace->clear();

			/* swap residual whitespace into top and sift it down */
			if (NULL != swap) {
				_volume += swap->length();
				_whitelist[0] = swap;
			} else {
				_count -= 1;
				_whitelist[0] = _whitelist[_count];
				_whitelist[_count] = NULL;
			}
			siftDown();

#if defined(EVACUATOR_DEBUG)
			verify();
			debug(whitespace, "-white");
#endif /* defined(EVACUATOR_DEBUG) */
		} else {

			/* add residual */
			add(swap);
		}

		return whitespace;
	}

	/**
	 * Tries to add a new free space element and sift it up the queue. It will be discarded
	 * if too small to include in current whitelist.
	 *
	 * @param whitespace points to head of free space to add
	 */
	void
	add(MM_EvacuatorWhitespace *whitespace)
	{
		/* presume whitespace too small to add */
		MM_EvacuatorWhitespace *displaced = whitespace;
		uintptr_t const length = (NULL != displaced) ? displaced->length() : 0;

		if (MM_EvacuatorBase::min_reusable_whitespace < length) {
			Debug_MM_true(_tenure == _env->getExtensions()->isOld((omrobjectptr_t)whitespace));

			/* append whitespace or displace a leaf from the bottom of the pile */
			uintptr_t leaf = _count;
			if (MM_EvacuatorBase::max_whitelist == leaf) {

				/* whitelist is full -- walk whitelist array from midpoint to end, displace first leaf smaller than whitespace */
				leaf = MM_EvacuatorBase::max_whitelist >> 1;
				while ((MM_EvacuatorBase::max_whitelist > leaf) && (_whitelist[leaf]->length() >= length)) {
					leaf += 1;
				}
			}

			/* if new whitespace was not too small to add, append and sift it up the heap */
			if (leaf < MM_EvacuatorBase::max_whitelist) {

				 if (leaf < _count) {
				 	displaced = _whitelist[leaf];
					_volume -= displaced->length();
				} else {
					displaced = NULL;
					_count += 1;
				}
				_whitelist[leaf] = whitespace;
				_volume += length;
				siftUp(leaf);

#if defined(EVACUATOR_DEBUG)
				debug(whitespace, "+white");
#endif /* defined(EVACUATOR_DEBUG) */
			}
		}

		/* discard any displacement */
		discard(displaced);

#if defined(EVACUATOR_DEBUG)
		verify();
#endif /* defined(EVACUATOR_DEBUG) */
	}

	/**
	 * Discards (recycles or fills with holes) all whitespace in current whitelist.
	 */
	void
	flush()
	{
		MM_EvacuatorWhitespace *whitespace = top(0);

		while (NULL != whitespace) {
			Debug_MM_true(MM_EvacuatorBase::min_reusable_whitespace < whitespace->length());
			discard(whitespace, true);
			whitespace = top(0);
		}

		Debug_MM_true((0 == _count) && (0 == _volume));
	}

	/**
	 * Returns the number of whitespace elements in the list
	 */
	uintptr_t getSize() { return _count; }

	/**
	 * Returns the number of whitespace bytes discarded (filled with holes)
	 */
	uintptr_t getDiscarded() { uintptr_t  discarded = _discarded; _discarded = 0; return discarded; }

	/**
	 * Returns the number of whitespace bytes discarded (filled with holes)
	 */
	uintptr_t getFlushed() { uintptr_t  flushed = _flushed; _flushed = 0; return flushed; }

	/**
	 * Returns the number of whitespace bytes discarded (filled with holes)
	 */
	uintptr_t getRecycled() { uintptr_t  recycled = _recycled; _recycled = 0; return recycled; }

	/**
	 * Basic array constructor obviates need for stdlibc++ linkage in gc component libraries. Array
	 * is allocated from forge as contiguous block sized to contain requested number of elements and
	 * must be freed using MM_Forge::free() when no longer needed. See MM_Evacuator::tearDown().
	 *
	 * @param forge the system memory allocator
	 * @param count the number of array elements to instantiate
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

	void
	bind(MM_EvacuatorBase *debugger, MM_EnvironmentBase *env, uintptr_t evacuatorIndex, MM_MemorySubSpace *subspace, bool isTenure)
	{
#if defined(EVACUATOR_DEBUG)
		_debugger = debugger;
		if (!_tenure) {
			Debug_MM_true(0 == (_discarded + _flushed + _recycled));
			for (uintptr_t i = 0; i < MM_EvacuatorBase::max_whitelist; i++) {
				Debug_MM_true(NULL == _whitelist[i]);
			}
		} else {
			/* tenure whitespace is retained between cycles, flushed before global gc and on collector shutdown */
			Debug_MM_true((0 < _count) || (0 == _flushed));
			for (uintptr_t i = 0; i < _count; i += 1) {
				Debug_MM_true(MM_EvacuatorBase::min_reusable_whitespace < _whitelist[i]->length());
				Debug_MM_true(NULL == _whitelist[i]->getNext());
			}
		}
#endif /* defined(EVACUATOR_DEBUG) */

		/* discarded/flushed/recycled counters are reset at end of cycle */
		_env = env;
		_index = evacuatorIndex;
		_subspace = subspace;
		_memoryPool = (MM_MemoryPoolAddressOrderedList *)_subspace->getMemoryPool();
		_stats = &_env->_scavengerStats;
		if (!isTenure) {
			_count = 0;
			_volume = 0;
		}
		_tenure = isTenure;
	}

	/**
	 * Constructor
	 */
	MM_EvacuatorWhitelist()
		: MM_Base()
		, _count(0)
		, _volume(0)
		, _discarded(0)
		, _flushed(0)
		, _recycled(0)
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

#if defined(EVACUATOR_DEBUG)
	void
	debug(MM_EvacuatorWhitespace *whitespace, const char* op)
	{
		if (_debugger->isDebugWhitelists()) {
			OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
			omrtty_printf("%5lu    %2lu:%7s[%c]; lx@%lx; volume:%lx; discarded:%lx; flushed:%lx; [%ld]{",
					_env->_scavengerStats._gcCount, _index, op, (_tenure ? 'T' : 'S'), ((NULL !=  whitespace) ? whitespace->length() : 0), (uintptr_t)whitespace,
					_volume, _discarded, _flushed, _count);
			uintptr_t sum = 0;
			for (uintptr_t i = 0; i < _count; i++) {
				omrtty_printf(" %lx@%lx", _whitelist[i]->length(), (uintptr_t)_whitelist[i]);
				sum += _whitelist[i]->length();
			}
			omrtty_printf(" }\n");
			Debug_MM_true(sum == _volume);
		}
	}

	bool le(uintptr_t a, uintptr_t b) { return _whitelist[a]->length() <= _whitelist[b]->length(); }

	void
	verify()
	{
		Debug_MM_true(MM_EvacuatorBase::max_whitelist >= _count);
		Debug_MM_true((0 == _count) == (0 == _volume));

		if (_debugger->isDebugWhitelists()) {
			uintptr_t volume = 0;
			Debug_MM_true(((0 == _count) && (NULL == _whitelist[0])) || (DEFAULT_SCAN_CACHE_MAXIMUM_SIZE >= _whitelist[0]->length()));
			for (uintptr_t i = 0; i < _count; i += 1) {
				Debug_MM_true(NULL != _whitelist[i]);
				Debug_MM_true(0  == ((sizeof(_whitelist[i]->length()) - 1) & _whitelist[i]->length()));
				Debug_MM_true(MM_EvacuatorBase::min_reusable_whitespace < _whitelist[i]->length());
				Debug_MM_true(_env->getExtensions()->objectModel.isDeadObject((void *)_whitelist[i]));
				Debug_MM_true(_env->getExtensions()->objectModel.getSizeInBytesDeadObject((omrobjectptr_t )_whitelist[i]) == _whitelist[i]->length());
				Debug_MM_true(_tenure == _env->getExtensions()->isOld((omrobjectptr_t)_whitelist[i]));
				Debug_MM_true(!_whitelist[i]->isReused());
				Debug_MM_true(!_whitelist[i]->isDiscarded());
				Debug_MM_true((0 == i) || le(i, up(i)));
				volume += _whitelist[i]->length();
			}
			uintptr_t end = _count >> 1;
			for (uintptr_t j = 0; j < end; j += 1) {
				Debug_MM_true(le(left(j), j));
				Debug_MM_true((right(j) >=_count) || le(right(j), j));
			}
			Debug_MM_true(volume == _volume);
		}
	}
#endif /* defined(EVACUATOR_DEBUG) */
};

#endif /* EVACUATORWHITELIST_HPP_ */
