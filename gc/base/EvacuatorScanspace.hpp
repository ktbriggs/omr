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

#ifndef EVACUATORSCANSPACE_HPP_
#define EVACUATORSCANSPACE_HPP_

#include "Base.hpp"
#include "EvacuatorCopyspace.hpp"
#include "EvacuatorWhitelist.hpp"
#include "IndexableObjectScanner.hpp"
#include "ObjectScanner.hpp"
#include "ObjectScannerState.hpp"

/**
 * Extends copyspace to allow scanning.
 */
class MM_EvacuatorScanspace : public MM_EvacuatorCopyspace
{
/*
 * Data members
 */
private:
	GC_ObjectScannerState _objectScannerState;	/* space reserved for instantiation of object scanner for current object */
	GC_ObjectScanner *_objectScanner;			/* points to _objectScannerState after scanner is instantiated, NULL if object scanner not instantiated */
	uint8_t *_scan;								/* scan head points to object being scanned */
#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	uint64_t _activations;						/* number of times this scanspace has been activated (pushed into) */
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

protected:
	/* enumeration of flag bits may be extended past scanEndFlag by subclasses */
	typedef enum scanspaceFlags {
		isRememberedFlag = copyEndFlag << 1		/* remembered state of object at scan head */
		, isSplitArrayFlag = copyEndFlag << 2	/* indicates that scanspace contains a slit array segment when set */
		, scanEndFlag = isSplitArrayFlag		/* marks end of scanspace flags */
	} scanspaceFlags;

public:
/*
 * Function members
 */
private:

protected:

public:
	/**
	 * Basic array constructor obviates need for stdlibc++ linkage in gc component libraries. Array
	 * is allocated from forge as contiguous block sized to contain requested number of elements and
	 * must be freed using MM_Forge::free() when no longer needed. See MM_Evacuator::tearDown().
	 *
	 * @param count the number of array elements to instantiate
	 * @return a pointer to instantiated array
	 */
	static MM_EvacuatorScanspace *
	newInstanceArray(MM_Forge *forge, uintptr_t count)
	{
		MM_EvacuatorScanspace *scanspace = (MM_EvacuatorScanspace *)forge->allocate(sizeof(MM_EvacuatorScanspace) * count, OMR::GC::AllocationCategory::FIXED, OMR_GET_CALLSITE());

		if (NULL != scanspace) {
			for (uintptr_t i = 0; i < count; i += 1) {
				MM_EvacuatorScanspace *space = new(scanspace + i) MM_EvacuatorScanspace();
				if (NULL == space) {
					return NULL;
				}
			}
		}

		return scanspace;
	}

	/*
	 * Current or latent object scanner is instantiated in space reserved within containing scanspace. This
	 * method should only be called when caller is committed to use the returned pointer for this purpose.
	 *
	 * To get current pointer to active object scanner, which may be NULL, use getObjectScanner().
	 *
	 * @see getActiveObjectScanner()
	 */
	GC_ObjectScannerState *getObjectScannerState() { _objectScanner = (GC_ObjectScanner *)&_objectScannerState; return &_objectScannerState; }

	/**
	 * Return pointer to active object scanner, or NULL if object scanner not instantiated.
	 */
	GC_ObjectScanner *getActiveObjectScanner() { return _objectScanner; }

	/**
	 * Advance the scan pointer to next unscanned object and drop active object scanner.
	 *
	 * @param scannedBytes number of bytes scanned (size of scanned object)
	 */
	GC_ObjectScanner *
	advanceScanHead(uintptr_t scannedBytes)
	{
		Debug_MM_true(_scan <= _copy);

		/* split array segment or current object, if any, has been completely scanned */
		if (_scan < _copy) {
			/* scan head for split array scanspaces is preset to copy head so this is necessary only for scalars and non-split arrays */
			_scan += scannedBytes;
		}

		/* done with current object scanner */
		_objectScanner = NULL;

		return _objectScanner;
	}

	/**
	 * Return the position of the scan head
	 */
	uint8_t *getScanHead() { return _scan; }

	/**
	 * Return the number of bytes remaining to be scanned
	 */
	uintptr_t getWorkSize() { return (uintptr_t)(_copy - _scan); }

	/**
	 * Return the number of bytes spanned between address of a slot within the
	 * object being scanned and current copy head.
	 *
	 * @param address the base address below copy head
	 */
	uintptr_t getCopySpan(void *address) { return (uintptr_t)(_copy - (uint8_t *)address); }

	/**
	 * Clear the remembered state (before starting to scan a new object)
	 */
	void clearRememberedState() { _flags &= ~(uintptr_t)isRememberedFlag; }

	/**
	 * Get the remembered state of the most recently scanned object
	 */
	bool getRememberedState() { return isRememberedFlag == (_flags & (uintptr_t)isRememberedFlag); }

	/**
	 * Set the remembered state of the current object if it is tenured and has a referent in new space
	 *
	 * @param referentInSurvivor set this to true if the referent is in new space
	 */
	void
	updateRememberedState(bool referentInSurvivor)
	{
		if (referentInSurvivor) {
			_flags |= (uintptr_t)isRememberedFlag;
		}
	}

	/**
	 * Load whitespace or unscanned work to scan into this scanspace. If there is work it
	 * may be followed by additional whitespace. In either case the copy limit will be set at the next
	 * page boundary or truncated at the copy head.
	 *
	 * @param base points to start of unscanned work
	 * @param copy points to whitespace at copy head
	 * @param length extent in bytes of unscanned work at base
	 * @param isLOA true if space is in large object area (LOA)
	 */
	void
	setScanspace(uint8_t *base, uint8_t *copy, uintptr_t length, bool isLOA = false)
	{
		Debug_MM_true(0 == getWorkSize());

		setCopyspace(base, copy, length, isLOA);

		_scan = _base;
		_objectScanner = NULL;

		clearRememberedState();

		Debug_MM_true(_copy <= _end);
	}

	/**
	 * Load a split array segment into this scanspace.
	 *
	 * @param base points to indexable object containing the segment
	 * @param end points after end of contiguous indexable object containing the segment
	 * @param scanner the object scanner preset to scan the array segment
	 */
	void
	setSplitArrayScanspace(uint8_t *base, uint8_t *end, GC_IndexableObjectScanner *scanner)
	{
		Debug_MM_true(0 == getWorkSize());

		setCopyspace(base, end, (uintptr_t)(end - base));
		_scan = _end;
		_flags |= isSplitArrayFlag;

		clearRememberedState();
		_objectScanner = scanner;

		Debug_MM_true((base == _base) && (_scan == _copy) &&(_copy == _end) && (_end == end));
	}

	/**
	 * Test whether scanspace contains a split array segment
	 */
	bool isSplitArraySegment() { return isSplitArrayFlag == (_flags & (uintptr_t)isSplitArrayFlag); }

	/**
	 * Pull work from an outside copyspace leaving it empty of work and retaining whitespace.
	 *
	 * @param fromspace the source copyspace
	 * @param maxObjectSize length in bytes of maximal small object
	 */
	void
	pullWork(MM_EvacuatorCopyspace *fromspace)
	{
		Debug_MM_true(0 == getWorkSize());
		Debug_MM_true(0 == getWhiteSize());

		uintptr_t length = 0;
		/* pull work from copyspace into this scanspace and rebase copyspace to copy head */
		_base = _scan = fromspace->rebase(&length);
		_copy = _end = _base + length;
		setLOA(fromspace->isLOA());

		/* reset remembered state and passivate active scanner */
		clearRememberedState();
		_objectScanner = NULL;
	}

	/**
	 * Pull work from a scanspace from point of last copy along with remaining whitespace.
	 *
	 * @param fromspace the source copyspace
	 * @param base points to head of tail to pull
	 */
	void
	pullTail(MM_EvacuatorScanspace *fromspace, uint8_t *base)
	{
		Debug_MM_true(0 == getWorkSize());
		Debug_MM_true(0 == getWhiteSize());

		/* pull work from copyspace into this scanspace and rebase copyspace to copy head */
		_base = _scan = base;
		_copy = fromspace->getCopyHead();
		_end = fromspace->getEnd();
		setLOA(fromspace->isLOA());

		/* truncate fromspace at point of last copy */
		fromspace->trim(base);

		/* reset remembered state and passivate active scanner */
		clearRememberedState();
		_objectScanner = NULL;
	}

	/**
	 * Pull whitespace from another scanspace
	 *
	 * @param fromspace the other scanspace
	 */
	void
	pullWhitespace(MM_EvacuatorScanspace *fromspace)
	{
		/* pull whitespace at end of fromspace into this scanspace */
		setScanspace(fromspace->_copy, fromspace->_copy, fromspace->getWhiteSize(), fromspace->isLOA());

		/* trim tail of fromspace and leave base, scan head and flags as they are */
		fromspace->_end = fromspace->_copy;
	}

	/**
	 * Reset the base and scan head to copy head and set new limit.
	 */
	void
	rebase()
	{
		Debug_MM_true(0 == getWorkSize());

		_base = _scan;
		_flags &= ~(isRememberedFlag | isSplitArrayFlag);
	}

	/**
	 * Reset this scanspace to an empty state
	 */
	void
	clear()
	{
		setScanspace(NULL, NULL, 0, false);
#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		_activations = 0;
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
	}

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	/**
	 * Bump activation count
	 */
	void activated() { _activations += 1; }

	/**
	 * Return the number of times this scanspace has been activated (pushed into)
	 */
	uint64_t getActivationCount() { return _activations; }
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

	/**
	 * Constructor
	 */
	MM_EvacuatorScanspace()
		: MM_EvacuatorCopyspace()
		, _objectScanner(NULL)
		, _scan(NULL)
#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		, _activations(0)
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
	{ }
};

#endif /* EVACUATORSCANSPACE_HPP_ */
