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

#ifndef EVACUATOR_HPP_
#define EVACUATOR_HPP_

#include "omr.h"
#include "omrcfg.h"
#include "omrmodroncore.h"
#include "omrthread.h"

#include "BaseNonVirtual.hpp"
#include "EnvironmentStandard.hpp"
#include "EvacuatorBase.hpp"
#include "EvacuatorCopyspace.hpp"
#include "EvacuatorDelegate.hpp"
#include "EvacuatorWorklist.hpp"
#include "EvacuatorScanspace.hpp"
#include "EvacuatorWhitelist.hpp"
#include "ForwardedHeader.hpp"
#include "GCExtensionsBase.hpp"
#include "ObjectModel.hpp"
#include "ParallelTask.hpp"
#include "SlotObject.hpp"

class GC_ObjectScanner;
class GC_SlotObject;
class MM_EvacuatorController;

class MM_Evacuator : public MM_BaseNonVirtual
{
/*
 * Data members
 */
public:
	/* Enumeration of memory spaces that are receiving evacuated material */
	typedef enum EvacuationRegion {
		survivor					/* survivor semispace for current gc */
		, tenure					/* tenure space */
		, evacuate					/* evacuate semispace for current gc */
		, unreachable				/* upper bound for evacuation regions */
	} EvacuationRegion;

private:
	/* Enumeration of conditions that relate to flushing */
	typedef enum FlushCondition {
		depth_first = 1				/* holding inside copy distance at 0 to force depth-first push/up flush/down until stack_overflow cleared */
		, breadth_first = 2			/* always flush all objects to outside copyspaces */
		, stack_overflow = 4		/* forcing flush while winding down stack after stack overflow */
		, stall = 8					/* forcing flush to service stalled evacuators */
		, survivor_overflow = 16	/* forcing flush to fill survivor outside copyspace remainder */
		, tenure_overflow = 32		/* forcing flush to fill tenure outside copyspace remainder */
	} FlushCondition;

	const uintptr_t _maxStackDepth;					/* number of frames to allocate for the scan stack */
	const uintptr_t _maxInsideCopySize;				/* limit on size of object that can be copied inside stack frames */
	const uintptr_t _maxInsideCopyDistance;			/* limit on distance from scan to copy head for copying inside stack frames */
	const uintptr_t _workerIndex;					/* controller's index of this evacuator */
	MM_EnvironmentStandard *_env;					/* collecting thread environment (this thread) */
	MM_EvacuatorController * const _controller;		/* controller provides collective services and instrumentation */
	MM_EvacuatorDelegate _delegate;					/* implements methods the evacuator delegates to the language/runtime */
	GC_ObjectModel * const _objectModel;			/* object model for language */
	MM_Forge * const _forge;						/* system memory allocator */
	omrthread_monitor_t	_mutex;						/* controls access to evacuator worklist */
	uintptr_t _splitArrayBytesToScan;				/* records length of split array segments while they are scanned on bottom of stack */
	uintptr_t _copiedBytesDelta[2];					/* cumulative number of bytes copied out of evacuation semispace since last report */
	uintptr_t _scannedBytesDelta;					/* cumulative number of bytes scanned in survivor semispace or tenure space since last report */
	uintptr_t _copiedBytesReportingDelta;			/* copied bytes increment for reporting copied/scanned byte counts to controller */
	uintptr_t _copyspaceRefreshThreshold;			/* maximum whitespace remainder that can be trimmed from copyspace before refreshing */
	uintptr_t _workReleaseThreshold;				/* number of bytes of unscanned bytes that should accumulate in outside copyspace before rebasing */
	uintptr_t _tenureMask;							/* used to determine age threshold for tenuring evacuated objects */
	MM_ScavengerStats *_stats;						/* pointer to MM_EnvironmentBase::_scavengerStats */
	uintptr_t _flushState;							/* bitmap of flush conditions */

	MM_EvacuatorScanspace * const _stackBottom;		/* bottom (location) of scan stack */
	MM_EvacuatorScanspace * const _stackCeiling;	/* physical limit of depth of scan stack */
	MM_EvacuatorScanspace * _stackLimit;			/* operational limit of depth of scan stack */
	MM_EvacuatorScanspace * _scanStackFrame;		/* active stack frame at current stack position, NULL if scan stack empty */
	MM_EvacuatorScanspace * _whiteStackFrame[2];	/* these point to stack or parked frame that holds the tail of the most recent allocation of inside whitespace per survivor/tenure */

	MM_EvacuatorCopyspace * const _copyspace;		/* points to array of outside copyspace to receive outside copy, one for each of survivor, tenure */
	MM_EvacuatorWhitelist * const _whiteList;		/* points to array of priority queue (largest on top) of whitespace, one for each of survivor, tenure */

	MM_EvacuatorCopyspace _largeCopyspace;			/* copyspace for receiving large objects (large objects are copied and distributed solo) */
	MM_EvacuatorWorklist _workList;					/* FIFO queue of distributable workspaces, in survivor or tenure space */
	MM_EvacuatorFreelist _freeList;					/* LIFO queue of empty workspaces */

	uintptr_t _copyspaceOverflow[2];				/* counts large objects overflowing outside copyspaces, reset when outside copyspace is refreshed */
	uint8_t *_heapBounds[3][2];						/* lower and upper bounds for nursery semispaces and tenure space */

	bool _completedScan;							/* set when heap scan is complete, cleared before heap scan starts */
	bool _abortedCycle;								/* set when work is aborted by any evacuator task */
protected:
public:

/*
 * Function members
 */
private:
	void scanRoots();
	void scanRemembered();
	void scanHeap();
	bool scanClearable();
	void scanComplete();

	MMINLINE MM_EvacuatorScanspace *reserveRootCopyspace(const EvacuationRegion evacuationRegion, uintptr_t slotObjectSizeAfterCopy);
	MMINLINE bool reserveInsideCopyspace(const EvacuationRegion evacuationRegion, const uintptr_t whiteSize, const uintptr_t slotObjectSizeAfterCopy);
	MMINLINE omrobjectptr_t copyForward(MM_ForwardedHeader *forwardedHeader, fomrobject_t *referringSlotAddress, MM_EvacuatorCopyspace * const copyspace, const uintptr_t originalLength, const uintptr_t forwardedLength);
	MMINLINE MM_EvacuatorScanspace *nextStackFrame(const EvacuationRegion evacuationRegion, MM_EvacuatorScanspace *frame);
	MMINLINE GC_ObjectScanner *nextObjectScanner(MM_EvacuatorScanspace * const scanspace, bool finalizeObjectScan = true);

	NOINLINE void scan();
	MMINLINE void copy();
	MMINLINE MM_EvacuatorScanspace *clear();
	MMINLINE void pull(MM_EvacuatorCopyspace *copyspace);
	MMINLINE void pull(MM_EvacuatorWorkspace *work);
	MMINLINE void push(MM_EvacuatorScanspace * const nextStackFrame);
	MMINLINE void pop();

	MMINLINE MM_EvacuatorCopyspace *reserveOutsideCopyspace(EvacuationRegion *evacuationRegion, const uintptr_t slotObjectSizeAfterCopy, bool useLargeCopyspace);
	MMINLINE omrobjectptr_t copyOutside(EvacuationRegion evacuationRegion, MM_ForwardedHeader *forwardedHeader, fomrobject_t *referringSlotAddress, const uintptr_t slotObjectSizeBeforeCopy, const uintptr_t slotObjectSizeAfterCopy);
	MMINLINE bool shouldRefreshCopyspace(const EvacuationRegion evacuationRegion, const uintptr_t slotObjectSizeAfterCopy, const uintptr_t copyspaceRemainder) const;
	MMINLINE bool shouldFlushOutside(const EvacuationRegion evacuationRegion);

	MMINLINE bool isSplitArrayWorkspace(MM_EvacuatorWorkspace *work) { return (0 < work->offset); }
	MMINLINE bool isSplitablePointerArray(MM_ForwardedHeader *forwardedHeader, uintptr_t objectSizeInBytes);
	MMINLINE void splitPointerArrayWork(omrobjectptr_t pointerArray);

	MMINLINE void addWork(MM_EvacuatorWorkspace *work);
	MMINLINE MM_EvacuatorWorkspace *findWork();
	MMINLINE bool getWork();

	MMINLINE bool rememberObject(omrobjectptr_t object);
	MMINLINE bool isNurseryAge(uintptr_t objectAge) { return (0 == (((uintptr_t)1 << objectAge) & _tenureMask)); }
	MMINLINE void flushForWaitState();
	MMINLINE void flushRememberedSet();

	MMINLINE void setAbortedCycle();
	MMINLINE bool isAbortedCycle();


	MMINLINE FlushCondition
	copyspaceOverflowCondition(EvacuationRegion evacuationRegion)
	{
		return (FlushCondition)((uintptr_t)survivor_overflow << (uintptr_t)evacuationRegion);
	}

	MMINLINE void
	setFlushCondition(FlushCondition condition, bool value)
	{
		if (value) {
			_flushState |= (uintptr_t)condition;
		} else {
			_flushState &= ~(uintptr_t)condition;
		}
	}

	MMINLINE bool
	isAnyFlushConditionSet(EvacuationRegion evacuationRegion)
	{
		return (0 != (_flushState & ~((uintptr_t)depth_first | (uintptr_t)copyspaceOverflowCondition(otherOutsideRegion(evacuationRegion)))));
	}

	MMINLINE bool testFlushCondition(FlushCondition condition) { return (0 != (_flushState & (uintptr_t)condition)); }

	MMINLINE void debugStack(const char *stackOp, bool treatAsWork = false);

#if defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	MMINLINE uint64_t startWaitTimer();
	MMINLINE void endWaitTimer(uint64_t waitStartTime, MM_EvacuatorWorkspace *work);
	MMINLINE uint64_t cycleMicros() { OMRPORT_ACCESS_FROM_ENVIRONMENT(_env); return omrtime_hires_delta(_env->getExtensions()->incrementScavengerStats._startTime, omrtime_hires_clock(), OMRPORT_TIME_DELTA_IN_MICROSECONDS); }
#endif /* defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

protected:
public:
	virtual UDATA getVMStateID() { return OMRVMSTATE_GC_EVACUATOR; }

	/**
	 * Instantiate evacuator.
	 *
	 * @param workerIndex the controller's index binding evacuator to controller
	 * @param controller the evacuation controller (collector)
	 * @param objectModel the runtime object model
	 * @param forge the system memory allocator
	 * @return an evacuator instance
	 */
	static MM_Evacuator *newInstance(uintptr_t workerIndex, MM_EvacuatorController *controller, GC_ObjectModel *objectModel, uintptr_t maxStackDepth, uintptr_t maxInsideCopySize, uintptr_t maxInsideCopyDistance, uintptr_t copyspaceRefreshThreshold, MM_Forge *forge);

	/**
	 * Terminate and deallocate evacuator instance
	 */
	void kill();

	/**
	 * Per instance evacuator initialization
	 */
	bool initialize();

	/**
	 * Per instance evacuator finalization
	 */
	void tearDown();

	MMINLINE uintptr_t getWorkerIndex() { return _workerIndex; }
	MMINLINE MM_EnvironmentStandard *getEnvironment() { return _env; }
	MMINLINE MM_EvacuatorDelegate *getDelegate() { return &_delegate; }

	MMINLINE bool isInEvacuate(void *address) { return (_heapBounds[evacuate][0] <= (uint8_t *)address) && ((uint8_t *)address < _heapBounds[evacuate][1]); }
	MMINLINE bool isInSurvivor(void *address) { return (_heapBounds[survivor][0] <= (uint8_t *)address) && ((uint8_t *)address < _heapBounds[survivor][1]); }
	MMINLINE bool isInTenure(void *address) { return _env->getExtensions()->isOld((omrobjectptr_t)address); }

	void flushWhitespace(EvacuationRegion region);

	/**
	 * Get the heap region (survivor|tenure|evacuate) containing an address
	 *
	 * @param address a putative heap address
	 * @return heap region or unreachable if address not in heap
	 */
	MMINLINE EvacuationRegion
	getEvacuationRegion(void *address)
	{
		if (isInSurvivor(address)) {
			return survivor;
		}
		if (isInTenure(address)) {
			return tenure;
		}
		if (isInEvacuate(address)) {
			return evacuate;
		}
		return unreachable;
	}

	/**
	 * Get the complementary outside region (survivor|tenure)
	 *
	 * @param thisOutsideRegion the outside region to complement
	 * @return complementary outside region
	 */
	MMINLINE EvacuationRegion
	otherOutsideRegion(EvacuationRegion thisOutsideRegion)
	{
		Debug_MM_true(tenure >= thisOutsideRegion);
		return (EvacuationRegion)(1 - (intptr_t)thisOutsideRegion);
	}

	/**
	 * Copy and forward root object given address of referring slot
	 *
	 * @param slotPtr address of referring slot
	 * @param breadthFirst copy object without recursing into dependent referents
	 * @return true if the root object was copied to new space (not tenured), false otherwise
	 */
	MMINLINE bool
	evacuateRootObject(volatile omrobjectptr_t *slotPtr, bool breadthFirst = false)
	{
		omrobjectptr_t object = *slotPtr;
		if (isInEvacuate(object)) {
			/* slot object must be evacuated -- determine before and after object size */
			MM_ForwardedHeader forwardedHeader(object);
			object = evacuateRootObject(&forwardedHeader, breadthFirst);
			Debug_MM_true(NULL != object);
			*slotPtr = object;
		}
		/* failure to evacuate must be reported as object in survivor space to maintain remembered set integrity */
		return isInSurvivor(object) || isInEvacuate(object);
	}

	/**
	 * Copy and forward root object given slot object encapsulating address of referring slot
	 *
	 * @param slotObject pointer to slot object encapsulating address of referring slot
	 * @param breadthFirst copy object without recursing into dependent referents
	 * @return true if the root object was copied to new space (not tenured), false otherwise
	 */
	MMINLINE bool
	evacuateRootObject(GC_SlotObject* slotObject, bool breadthFirst = false)
	{
		omrobjectptr_t object = slotObject->readReferenceFromSlot();
		if (isInEvacuate(object)) {
			/* slot object must be evacuated -- determine before and after object size */
			MM_ForwardedHeader forwardedHeader(object);
			object = evacuateRootObject(&forwardedHeader, breadthFirst);
			Debug_MM_true(NULL != object);
			slotObject->writeReferenceToSlot(object);
		}
		/* failure to evacuate must be reported as object in survivor space to maintain remembered set integrity */
		return isInSurvivor(object) || isInEvacuate(object);
	}

	/**
	 * Main evacuation method driven by all gc slave threads during a nursery collection.
	 *
	 * (scanRemembered scanRoots (scanHeap scanComplete)) (scanClearable (scanHeap scanComplete))*
	 *
	 * For j9 java, all clearing is performed in an evacuator delegate, using a deprecated (legacy)
	 * calling pattern dictated by MM_RootScanner protocol. In that context, the clearing term in
	 * the above expression becomes:
	 *
	 * (evacuateRootObject* (evacuateHeap scanComplete))
	 *
	 * The evacuateHeap method is provided for this context only and its use is deprecated. Evacuator
	 * is designed to deal with a stream of pointers to root objects presented via evacuateRootObject
	 * in scanClearable. When scanClearable completes, the evacuator will recursively scan the objects
	 * depending from the roots presented in scanClearable.
	 *
	 * @param[in] env worker thread environment
	 *
	 * @see MM_ParallelScavengeTask::run(MM_EnvironmentBase *)
	 */
	void workThreadGarbageCollect(MM_EnvironmentStandard *env);

	/**
	 * Evacuate all objects in evacuate space referenced by an object in the remembered set
	 *
	 * @param objectptr the remembered object, in tenure space
	 * @return true if the remembered object contained any evacuated referents
	 */
	bool evacuateRememberedObject(omrobjectptr_t objectptr);

	/**
	 * Test tenured object for containment of referents in survivor space. This method should not be
	 * called until after evacuation is complete.
	 */
	bool shouldRememberObject(omrobjectptr_t objectPtr);

	/**
	 * Copy and forward root object given a forwarding header obtained from the object
	 *
	 * @param forwardedHeader pointer to forwarding header obtained from the object
	 * @param breadthFirst copy object without recursing into dependent referents
	 * @return address in survivor or tenure space that object was forwarded to
	 */
	omrobjectptr_t evacuateRootObject(MM_ForwardedHeader *forwardedHeader, bool breadthFirst = false);

	/**
	 * Copy and forward root object from mutator stack slot given address of referring slot.
	 *
	 * NOTE: the object will be copied and forwarded here but the indirect pointer parameter
	 * update may be deferred if forwarded to tenure space. In that case the indirect pointer
	 * will be updated after recursive heap scanning is complete, when the delegate rescans
	 * thread slots.
	 *
	 * @param objectPtrIndirect address of referring slot
	 * @see MM_EvacuatorDelegate::rescanThreadSlots()
	 * @see rescanThreadSlot(omrobjectptr_t)
	 */
	void evacuateThreadSlot(volatile omrobjectptr_t *objectPtrIndirect);

	/**
	 * Update a thread slot holding a pointer to an object that was evacuated into tenure space
	 * in the current nursery collection. These updates are deferred from evacuateThreadSlot()
	 * to obviate the need for an internal write barrier.
	 *
	 * @param objectPtrIndirect address of referring slot
	 */
	void rescanThreadSlot(omrobjectptr_t *objectPtrIndirect);

	/**
	 * Copy and forward all objects in evacuation space depending from clearable objects copied
	 * during a clearing stage.
	 *
	 * @return true unless gc cycle is aborting
	 */
	bool evacuateHeap();

	/**
	 * Controller calls this to get the volume of work available on the evacator's work queue.
	 */
	MMINLINE uintptr_t getVolumeOfWork() { return *(_workList.volume()); }

	/**
	 * Controller calls this when it allocates a TLH from survivor or tenure region that is too small to hold
	 * the current object. The evacuator adds the unused TLH to the whitelist for the containing region.
	 */
	void receiveWhitespace(MM_EvacuatorWhitespace *whitespace);

	/**
	 * Per gc, bind evacuator instance to worker thread and set up evacuator environment, clear evacuator gc stats
	 *
	 * @param[in] env worker thread environment to bind to
	 * @param[in] tenureMask a copy of the controller's tenure mask for the cycle
	 * @param[in] heapBounds address bounds for heap regions survivor, tenure, evacuate
	 * @param[in] copiedBytesReportingDelta the number of scanned/copied bytes to process between reporting scan/copy counts to controller
	 */
	void bindWorkerThread(MM_EnvironmentStandard *env, uintptr_t tenureMask, uint8_t *heapBounds[][2], uintptr_t copiedBytesReportingDelta);

	/**
	 * Per gc, unbind evacuator instance from worker thread, merge evacuator gc stats
	 *
	 * @param[in] env worker thread environment to unbind from
	 */
	void unbindWorkerThread(MM_EnvironmentStandard *env);

	/**
	 * Get the number of allocated bytes discarded during the gc cycle (micro-fragmentation).
	 */
	uintptr_t getDiscarded() { return _whiteList[survivor].getDiscarded() + _whiteList[tenure].getDiscarded(); }

	/**
	 * Get the number of allocated bytes flushed at the end of the gc cycle (macro-fragmentation).
	 */
	uintptr_t getFlushed() { return _whiteList[survivor].getFlushed() + _whiteList[tenure].getFlushed(); }

	/**
	 * Get the number of allocated bytes flushed at the end of the gc cycle (macro-fragmentation).
	 */
	uintptr_t getRecycled() { return _whiteList[survivor].getRecycled() + _whiteList[tenure].getRecycled(); }

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	uintptr_t getStackActivationCount(uintptr_t depth) { return _stackBottom[depth].getActivationCount(); }
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

#if defined(EVACUATOR_DEBUG)
	void checkSurvivor();
	void checkTenure();
#endif /* defined(EVACUATOR_DEBUG) */

	/**
	 * Constructor. The minimum number of stack frames is three - two to hold whitespace and one or more
	 * for scanning over full stack range up to maxStackDepth. Set maxStackDepth=1 for breadth first scanning
	 * and maxInsideCopyDistance=0 for depth first scanning.
	 *
	 * @param workerIndex worker thread index assigned by controller
	 * @param controller the controller
	 * @param objectModel the object model
	 * @param maxStackDepth maximum stack depth
	 * @param maxInsideCopySize maximum size of objects that can be copied inside stack frame
	 * @param maxInsideCopyDistance maximum distance from scan frame base to copy head to allow inside copy
	 * @param copyspaceRefreshThreshold maximum whitespace remainder that can be trimmed from copyspace before refreshing
	 * @parma forge the system memory allocator
	 */
	MM_Evacuator(uintptr_t workerIndex, MM_EvacuatorController *controller, GC_ObjectModel *objectModel, uintptr_t maxStackDepth, uintptr_t maxInsideCopySize, uintptr_t maxInsideCopyDistance, uintptr_t copyspaceRefreshThreshold, MM_Forge *forge)
		: MM_BaseNonVirtual()
		, _maxStackDepth(maxStackDepth)
		, _maxInsideCopySize(maxInsideCopySize)
		, _maxInsideCopyDistance(maxInsideCopyDistance)
		, _workerIndex(workerIndex)
		, _env(NULL)
		, _controller(controller)
		, _delegate()
		, _objectModel(objectModel)
		, _forge(forge)
		, _mutex(NULL)
		, _splitArrayBytesToScan(0)
		, _scannedBytesDelta(0)
		, _copiedBytesReportingDelta(0)
		, _copyspaceRefreshThreshold(copyspaceRefreshThreshold)
		, _workReleaseThreshold(0)
		, _tenureMask(0)
		, _stats(NULL)
		, _flushState(0)
		, _stackBottom(MM_EvacuatorScanspace::newInstanceArray(_forge, OMR_MAX(unreachable, _maxStackDepth)))
		, _stackCeiling(_stackBottom + OMR_MAX(unreachable, _maxStackDepth))
		, _stackLimit(_stackCeiling)
		, _scanStackFrame(NULL)
		, _copyspace(MM_EvacuatorCopyspace::newInstanceArray(_forge, evacuate))
		, _whiteList(MM_EvacuatorWhitelist::newInstanceArray(_forge, evacuate))
		, _freeList(_forge)
		, _completedScan(true)
		, _abortedCycle(false)
	{
		_typeId = __FUNCTION__;

		_copiedBytesDelta[survivor] = _copiedBytesDelta[tenure] = 0;
		_copyspaceOverflow[survivor] = _copyspaceOverflow[tenure] = 0;
		_whiteStackFrame[survivor] = _whiteStackFrame[tenure] = NULL;

		Debug_MM_true(0 == (_objectModel->getObjectAlignmentInBytes() % sizeof(uintptr_t)));
		Assert_MM_true((NULL != _stackBottom) && (NULL != _copyspace) && (NULL != _whiteList));
	}

	friend class MM_EvacuatorController;
	friend class MM_ScavengerRootClearer;
};

#endif /* EVACUATOR_HPP_ */
