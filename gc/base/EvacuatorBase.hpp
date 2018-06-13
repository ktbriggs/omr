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

#ifndef EVACUATORBASE_HPP_
#define EVACUATORBASE_HPP_

#undef EVACUATOR_DEBUG
#define EVACUATOR_DEBUG_ALWAYS

#if defined(EVACUATOR_DEBUG) && defined(EVACUATOR_DEBUG_ALWAYS)
#error "EVACUATOR_DEBUG and EVACUATOR_DEBUG_ALWAYS are mutually exclusive"
#endif

#if defined(EVACUATOR_DEBUG)
#include "omrgcconsts.h"
#include "ModronAssertions.h"

#include "Math.hpp"

/* base debug flags */
#define EVACUATOR_DEBUG_END 1
#define EVACUATOR_DEBUG_CYCLE 2
#define EVACUATOR_DEBUG_EPOCH 4
#define EVACUATOR_DEBUG_WORK 8
#define EVACUATOR_DEBUG_STACK 16
#define EVACUATOR_DEBUG_COPY 32
#define EVACUATOR_DEBUG_REMEMBERED 64
#define EVACUATOR_DEBUG_ALLOCATE 128
#define EVACUATOR_DEBUG_WHITELISTS 256
#define EVACUATOR_DEBUG_POISON_DISCARD 512
#define EVACUATOR_DEBUG_BACKOUT 1024
#define EVACUATOR_DEBUG_DELEGATE 2048
#define EVACUATOR_DEBUG_HEAPCHECK 4096

/* default debug flags */
#if defined(EVACUATOR_DEBUG)
#define EVACUATOR_DEBUG_DEFAULT_FLAGS 1
#else
#define EVACUATOR_DEBUG_DEFAULT_FLAGS 0
#endif /* defined(EVACUATOR_DEBUG) */

/* delegate can define additional flags above 0x10000 */
#define EVACUATOR_DEBUG_DELEGATE_BASE 0x10000

#define Debug_MM_true(assertion) Assert_MM_true(assertion)
#define Debug_MM_true1(env, assertion, format, arg) Assert_GC_true_with_message1(env, assertion, format, arg)
#define Debug_MM_true2(env, assertion, format, arg1, arg2) Assert_GC_true_with_message2(env, assertion, format, arg1, arg2)
#define Debug_MM_true3(env, assertion, format, arg1, arg2, arg3) Assert_GC_true_with_message3(env, assertion, format, arg1, arg2, arg3)
#define Debug_MM_true4(env, assertion, format, arg1, arg2, arg3, arg4) Assert_GC_true_with_message4(env, assertion, format, arg1, arg2, arg3, arg4)
#else
#define Debug_MM_true(assertion)
#define Debug_MM_true1(env, assertion, format, arg)
#define Debug_MM_true2(env, assertion, format, arg1, arg2)
#define Debug_MM_true3(env, assertion, format, arg1, arg2, arg3)
#define Debug_MM_true4(env, assertion, format, arg1, arg2, arg3, arg4)
#endif /* defined(EVACUATOR_DEBUG) */

#include "GCExtensionsBase.hpp"

#define NOINLINE

class MM_EvacuatorBase
{
/**
 * Data members
 */
private:
#if defined(EVACUATOR_DEBUG)
	uintptr_t _gcCycle;
	uintptr_t _gcEpoch;
	uintptr_t _debugCycle;
	uintptr_t _debugEpoch;
	uintptr_t _debugTrace;
	uintptr_t _debugFlags;
#endif /* defined(EVACUATOR_DEBUG) */

protected:
public:
	/* largest amount of whitespace that can be discarded from the scan stack and outside copyspaces */
	static const uintptr_t max_scanspace_remainder = 32;
	static const uintptr_t max_copyspace_remainder = 768;

	/* Work packet size cannot be less than this */
	static const uintptr_t min_workspace_size = 256;
	/* multiplier for minimum work packet size determines threshold byte count for objects overflowing copyspace whitespace remainder */
	static const uintptr_t max_large_object_overflow_quanta = 3;

	/* minimum size of whitespace that is recyclable from whitelists */
	static const uintptr_t min_recyclable_whitespace = 8192;

	/* maximum number of array element slots to include in each split array segment */
	static const uintptr_t max_split_segment_elements = DEFAULT_ARRAY_SPLIT_MINIMUM_SIZE;
	/* minimum size in bytes of a splitable indexable object (header, element count, slots...) */
	static const uintptr_t min_split_indexable_size = (max_split_segment_elements * sizeof(fomrobject_t));

	/* Minimal size of scan stack -- a value of 1 forces breadth first scanning */
	static const uintptr_t min_scan_stack_depth = 1;
	/* Object size threshold for copying inside -cannot be set to a value lower than this */
	static const uintptr_t min_inside_object_size = OMR_MINIMUM_OBJECT_SIZE;

	/* number of elements in whitelist backing array must be (2^N)-1 for some N */
	static const uintptr_t max_whitelist = 15;

/**
 * Function members
 */
private:
protected:
public:
#if defined(EVACUATOR_DEBUG)
	static const char *callsite(const char *id);

	enum { always, until, at, from };

	void
	setDebugFlags(uintptr_t debugFlags, uintptr_t debugCycle, uintptr_t debugEpoch, uintptr_t debugTrace = always)
	{
		_debugFlags = debugFlags;
		_debugCycle = debugCycle;
		_debugEpoch = debugEpoch;
		_debugTrace = debugTrace;
	}
	void setDebugFlags(uint64_t debugFlags = EVACUATOR_DEBUG_DEFAULT_FLAGS);

	void setDebugCycleAndEpoch(uintptr_t gcCycle, uintptr_t cycleEpoch) { _gcCycle = gcCycle;  _gcEpoch = cycleEpoch; }

	bool
	isDebugCycleAndEpoch()
	{
		if (0 != _debugFlags) {
			if (always != _debugTrace) {
				switch (_debugTrace) {
				case until:
					return ((0 == _debugCycle) || (_gcCycle <= _debugCycle)) && ((0 == _debugEpoch) || (_gcEpoch <= _debugEpoch));
				case at:
					return ((0 == _debugCycle) || (_gcCycle == _debugCycle)) && ((0 == _debugEpoch) || (_gcEpoch == _debugEpoch));
				case from:
					return ((0 == _debugCycle) || (_gcCycle >= _debugCycle)) && ((0 == _debugEpoch) || (_gcEpoch >= _debugEpoch));
				default:
					break;
				}
			}
		}
		return true;
	}

	bool isDebugFlagSet(uintptr_t debugFlag) { return isDebugCycleAndEpoch() && (debugFlag == (debugFlag & _debugFlags)); }
	bool isAnyDebugFlagSet(uintptr_t debugFlags) { return isDebugCycleAndEpoch() && (0 != (debugFlags | _debugFlags)); }
	bool areAllDebugFlagsSet(uintptr_t debugFlags) { return isDebugFlagSet(debugFlags); }
	bool isDebugEnd() { return isDebugFlagSet(EVACUATOR_DEBUG_END); }
	bool isDebugCycle() { return isDebugFlagSet(EVACUATOR_DEBUG_CYCLE); }
	bool isDebugEpoch() { return isDebugFlagSet(EVACUATOR_DEBUG_EPOCH); }
	bool isDebugStack() { return isDebugFlagSet(EVACUATOR_DEBUG_STACK); }
	bool isDebugWork() { return isDebugFlagSet(EVACUATOR_DEBUG_WORK); }
	bool isDebugCopy() { return isDebugFlagSet(EVACUATOR_DEBUG_COPY); }
	bool isDebugRemembered() { return isDebugFlagSet(EVACUATOR_DEBUG_REMEMBERED); }
	bool isDebugWhitelists() { return isDebugFlagSet(EVACUATOR_DEBUG_WHITELISTS); }
	bool isDebugPoisonDiscard() { return isDebugFlagSet(EVACUATOR_DEBUG_POISON_DISCARD); }
	bool isDebugAllocate() { return isDebugFlagSet(EVACUATOR_DEBUG_ALLOCATE); }
	bool isDebugBackout() { return isDebugFlagSet(EVACUATOR_DEBUG_BACKOUT); }
	bool isDebugDelegate() { return isDebugFlagSet(EVACUATOR_DEBUG_DELEGATE); }
	bool isDebugHeapCheck() { return isDebugFlagSet(EVACUATOR_DEBUG_HEAPCHECK); }
#else
	void setDebugFlags(uintptr_t debugFlags, uintptr_t debugCycle, uintptr_t debugEpoch, uintptr_t debugTrace = 0) { }
	void setDebugFlags(uint64_t debugFlags = 0) { }
	void setDebugCycleAndEpoch(uintptr_t gcCycle, uintptr_t cycleEpoch) { }
	bool isDebugCycleAndEpoch() { return false; }
	bool isDebugFlagSet(uintptr_t debugFlag) { return false; }
	bool isAnyDebugFlagSet(uintptr_t debugFlags) { return false; }
	bool areAllDebugFlagsSet(uintptr_t debugFlags) { return false; }
	bool isDebugEnd() { return false; }
	bool isDebugCycle() { return false; }
	bool isDebugEpoch() { return false; }
	bool isDebugWork() { return false; }
	bool isDebugStack() { return false; }
	bool isDebugCopy() { return false; }
	bool isDebugRemembered() { return false; }
	bool isDebugWhitelists() { return false; }
	bool isDebugPoisonDiscard() { return false; }
	bool isDebugAllocate() { return false; }
	bool isDebugBackout() { return false; }
	bool isDebugDelegate() { return false; }
	bool isDebugHeapCheck() { return false; }
#endif /* defined(EVACUATOR_DEBUG) */

	MM_EvacuatorBase()
#if defined(EVACUATOR_DEBUG)
	: _gcCycle(0)
	, _gcEpoch(0)
	, _debugCycle(0)
	, _debugEpoch(0)
	, _debugTrace(0)
	, _debugFlags(0)
#endif /* defined(EVACUATOR_DEBUG) */
	{ }
};

#endif /* EVACUATORBASE_HPP_ */
