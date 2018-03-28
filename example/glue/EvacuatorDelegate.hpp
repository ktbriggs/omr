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

#ifndef EVACUATORDELEGATE_HPP_
#define EVACUATORDELEGATE_HPP_

#include "omr.h"
#include "omrcfg.h"
#include "omrExampleVM.hpp"
#include "omrhashtable.h"

#include "EvacuatorBase.hpp"
#include "EnvironmentStandard.hpp"
#include "ForwardedHeader.hpp"
#include "IndexableObjectScanner.hpp"
#include "MixedObjectScanner.hpp"

class MM_Evacuator;
class MM_EvacuatorController;
class MM_Scavenger;

class MM_EvacuatorDelegate
{
/*
 * Data members
 */
public:
	/**
	 * Controller maintains a volatile bitset of delegate flags, evacuators mutate and test
	 * these to control evacuation stages of each gc cycle. Evacuator delegate may define
	 * flags as required to share this information across evacuators during gc cycles.
	 *
	 * @see MM_EvacuatorController::max_evacuator_public_flag (public bit mask is 0xffffffff)
	 */
	enum {
		unused = 1
	};
private:
	MM_EnvironmentStandard *_env;
	MM_Scavenger *_controller;
	MM_Evacuator *_evacuator;
	bool _isCleared;

protected:
public:

/*
 * Function members
 */
private:
protected:
public:
	/**
	 * Evacuator calls this when it instantiates the delegate to bind controller-evacuator-delegate. This
	 * binding persists over the delegate's lifetime.
	 *
	 * @param evacuator the MM_Evacuator instance to bind delegate to
	 * @param forge points to system memory allocator
	 * @controller points to the evacuator controller
	 */
	bool
	initialize(MM_Evacuator *evacuator, MM_Forge *forge, MM_EvacuatorController *controller)
	{
		_evacuator = evacuator;
		_controller = (MM_Scavenger *)controller;
		return true;
	}


	/**
	 * This is called when the OMR vm is shut down, to release resources held by the delegate
	 */
	void tearDown() { }

	/** This is called from the controller before activating any evacuator instances to allow the
	 * delegate to set up evacuator flags for the evacuation cycle.
	 *
	 * @param extensions points to omr vm extensions
	 * @return preset evacuator flags for the cycle
	 */
	static uintptr_t prepareForEvacuation(MM_GCExtensionsBase *extensions) { return 0; }

	/**
	/**
	 * Evacuator calls this when it starts starts work in an evacuation cycle. This binds the evacuator
	 * gc thread (environment) to the evacuator-delegate for the duration of the cycle. This method must
	 * be implemented in EvacuatorDelegate.cpp, as MM_Evacutor is inaccessible here. The implementation
	 * must set MM_EvacuatorDelegate::_env to the environment bound to the evacuator at this time.
	 */
	void cycleStart(); /* { _env = evacuator->getEnvironment(); } */

	void cycleEnd() { _env = NULL; }

	/**
	 * Evacuator calls this to instantiate an object scanner within space provided by objectScannerState
	 *
	 * @param objectptr the object to be scanned
	 * @param objectScannerState points to space to instantiate the object scanner into
	 * @param flags to be set in the object scanner
	 * @return a pointer to the object scanner
	 */
	GC_ObjectScanner *
	getObjectScanner(omrobjectptr_t objectptr, void *objectScannerState, uintptr_t flags)
	{
		Debug_MM_true(GC_ObjectScanner::isHeapScan(flags) ^ GC_ObjectScanner::isRootScan(flags));
		return GC_MixedObjectScanner::newInstance(_env, objectptr, objectScannerState, flags);
	}

	/**
	 * Evacuator calls this to instantiate an object scanner for a splitable array object within space provided by
	 * objectScannerState. Splitable arrays are indexable objects with contiguous representation in the heap, which
	 * may be split into multiple segments for parallel scanning. The example framework does not support indexable
	 * objects, so this implementation does not split objects for parallel scanning.
	 *
	 * It is assumed that the array elements are contiguous with the object header. For the last segment the end
	 * slot will point just past the end of the image of the object in evacuation space, which may or may not
	 * include additional material, eg hash code, contiguous with the last array element. In that case the
	 * implementation must adjust the end pointer to point just past the end of the last array element.
	 *
	 * @param objectptr the object to be scanned
	 * @param objectScannerState points to space to instantiate the object scanner into
	 * @param splitIndex index (0-based) of first array element to scan in split array segment
	 * @param splitAmount number of array elements to scan in split array segment
	 * @param flags to be set in the object scanner
	 * @return a pointer to the object scanner
	 */
	GC_IndexableObjectScanner *
	getSplitPointerArrayObjectScanner(omrobjectptr_t objectptr, void *objectScannerState, uintptr_t splitIndex, uintptr_t splitAmount, uintptr_t flags)
	{
		Debug_MM_true(false);
		return NULL;
	}

	/**
	 * Return true if teh object is an indexable array of pointers to objects
	 */
	bool isIndexablePointerArray(MM_ForwardedHeader *forwardedHeader) { return false; }

	/**
	 * Locate the base of the contiguous range of indexed object reference slots contained in an indexable object and
	 * count the reference slots contained in the range.
	 *
	 * If the caller's runtime does not support indexable objects this method should return NULL without touching
	 * the caller's element count.
	 */
	fomrobject_t *getIndexableDataBounds(omrobjectptr_t indexableObject, uintptr_t *numberOfElements) { return NULL; }

	bool hasClearable() { return !_isCleared; }

	bool objectHasIndirectObjectsInNursery(omrobjectptr_t objectptr) { return false; }

	bool scanIndirectObjects(omrobjectptr_t objectptr) { return false; }

	void scanRoots();

	void scanClearable();

	void rescanThreadSlots() { }

	void flushForWaitState() { }

	void flushForEndCycle() { }

	MM_EvacuatorDelegate()
		: _env(NULL)
		, _controller(NULL)
		, _evacuator(NULL)
		, _isCleared(false)
	{ }

#if defined(EVACUATOR_DEBUG_DELEGATE)
	void debugValidateObject(omrobjectptr_t objectptr) { }
	void debugValidateObject(MM_ForwardedHeader *forwardedHeader) { }
	const char *
	debugGetClassname(omrobjectptr_t objectptr, char *buffer, uintptr_t bufferLength)
	{
		buffer[0] = 0;
		return buffer;
	}
#endif /* defined(EVACUATOR_DEBUG_DELEGATE) */
};
#endif /* EVACUATORDELEGATE_HPP_ */
