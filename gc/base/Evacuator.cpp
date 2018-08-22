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

#include "CollectorLanguageInterface.hpp"
#include "EnvironmentStandard.hpp"
#include "Evacuator.hpp"
#include "EvacuatorController.hpp"
#include "EvacuatorDelegate.hpp"
#include "ForwardedHeader.hpp"
#include "GCExtensionsBase.hpp"
#include "IndexableObjectScanner.hpp"
#include "Math.hpp"
#include "MemcheckWrapper.hpp"
#include "ObjectModel.hpp"
#include "ScavengerStats.hpp"
#include "SlotObject.hpp"
#include "SublistFragment.hpp"

extern "C" {
	uintptr_t allocateMemoryForSublistFragment(void *vmThreadRawPtr, J9VMGC_SublistFragment *fragmentPrimitive);
}

MM_Evacuator *
MM_Evacuator::newInstance(uintptr_t workerIndex, MM_EvacuatorController *controller, GC_ObjectModel *objectModel, uintptr_t maxInsideCopySize, MM_Forge *forge)
{
	MM_Evacuator *evacuator = (MM_Evacuator *)forge->allocate(sizeof(MM_Evacuator), OMR::GC::AllocationCategory::FIXED, OMR_GET_CALLSITE());
	if(NULL != evacuator) {
		new(evacuator) MM_Evacuator(workerIndex, controller, objectModel, maxInsideCopySize, forge);
		if (!evacuator->initialize()) {
			evacuator->kill();
			evacuator = NULL;
		}
	}
	return evacuator;
}

void
MM_Evacuator::kill()
{
	tearDown();
	_forge->free(this);
}

bool
MM_Evacuator::initialize()
{
	/* initialize the evacuator mutex */
	if (0 != omrthread_monitor_init_with_name(&_mutex, 0, "MM_Evacuator::_mutex")) {
		return false;
	}

	/* initialize the delegate */
	if (!_delegate.initialize(this, _forge, _controller)) {
		return false;
	}

	return true;
}

void
MM_Evacuator::tearDown()
{
	/* tear down delegate */
	_delegate.tearDown();

	/* tear down mutex */
	if (NULL != _mutex) {
		omrthread_monitor_destroy(_mutex);
		_mutex = NULL;
	}

	/* free forge memory bound to arrays instantiated in constructor */
	_forge->free(_stackBottom);
	_forge->free(_copyspace);
	_forge->free(_whiteList);

	/* free the freelist */
	_freeList.reset();
}

/**
 * Per gc cycle setup. This binds the evacuator instance to a gc worker thread for the duration of the cycle.
 *
 * @param env the environment for the gc worker thread that is bound to this evacuator instance
 */
void
MM_Evacuator::bindWorkerThread(MM_EnvironmentStandard *env)
{
	omrthread_monitor_enter(_mutex);

	/* bind evacuator and delegate to executing gc thread */
	_env = env;
	_env->setEvacuator(this);
	_delegate.cycleStart();

	/* clear worker gc stats */
	_stats = &_env->_scavengerStats;
	_stats->clear(true);
	_stats->_gcCount = _env->getExtensions()->scavengerStats._gcCount;

	/* Reset the local remembered set fragment */
	_env->_scavengerRememberedSet.count = 0;
	_env->_scavengerRememberedSet.fragmentCurrent = NULL;
	_env->_scavengerRememberedSet.fragmentTop = NULL;
	_env->_scavengerRememberedSet.fragmentSize = (uintptr_t)OMR_SCV_REMSET_FRAGMENT_SIZE;
	_env->_scavengerRememberedSet.parentList = &_env->getExtensions()->rememberedSet;

	/* clear cycle stats */
	_splitArrayBytesToScan = 0;
	_completedScan = _abortedCycle = false;
	_copiedBytesDelta[survivor] = _copiedBytesDelta[tenure] = 0;
	_scannedBytesDelta = 0;

	/* set up whitespaces for the cycle */
#if !defined(EVACUATOR_DEBUG)
	_whiteList[survivor].bind(NULL, _env, _workerIndex, _controller->getMemorySubspace(survivor), false);
	_whiteList[tenure].bind(NULL, _env, _workerIndex, _controller->getMemorySubspace(tenure), true);
#else
	_whiteList[survivor].bind(&_controller->_debugger, _env, _workerIndex, _controller->getMemorySubspace(survivor), false);
	_whiteList[tenure].bind(&_controller->_debugger, _env, _workerIndex, _controller->getMemorySubspace(tenure), true);
#endif /* defined(EVACUATOR_DEBUG) */

	/* load some empty work packets into the freelist -- each evacuator retains forge memory between gc cycles to back this up */
	_freeList.reload();

	/* signal controller that this evacuator is ready to start work -- the controller will bind the evacuator to the gc cycle */
	_controller->startWorker(this, &_tenureMask, _heapBounds, &_copiedBytesReportingDelta);

	_workReleaseThreshold = _controller->calculateOptimalWorkPacketSize(getVolumeOfWork());

	Debug_MM_true(0 == *(_workList.volume()));

	omrthread_monitor_exit(_mutex);
}

/**
 * Per gc cycle cleanup. This unbinds the evacuator instance from its gc worker thread.
 *
 * @param env the environment for the gc worker thread that is bound to this evacuator instance
 */
void
MM_Evacuator::unbindWorkerThread(MM_EnvironmentStandard *env)
{
	omrthread_monitor_enter(_mutex);

	/* flush to freelist any abandoned work from the worklist  */
	_workList.flush(&_freeList);

	/* reset the freelist to free any underflow chunks allocated from system memory */
	_freeList.reset();

	/* merge GC stats */
	_controller->mergeThreadGCStats(_env);

	/* unbind delegate from gc thread */
	_delegate.cycleEnd();

	/* unbind evacuator from gc thread */
	_env->setEvacuator(NULL);
	_env = NULL;

	omrthread_monitor_exit(_mutex);
}

omrobjectptr_t
MM_Evacuator::evacuateRootObject(MM_ForwardedHeader *forwardedHeader)
{
	omrobjectptr_t forwardedAddress = forwardedHeader->getForwardedObject();

	if (!isAbortedCycle() && (NULL == forwardedAddress)) {
		/* slot object must be evacuated -- determine before and after object size */
		uintptr_t slotObjectSizeAfterCopy = 0, slotObjectSizeBeforeCopy = 0, hotFieldAlignmentDescriptor = 0;
		_objectModel->calculateObjectDetailsForCopy(_env, forwardedHeader, &slotObjectSizeBeforeCopy, &slotObjectSizeAfterCopy, &hotFieldAlignmentDescriptor);

		/* reserve space for object in outside copyspace or abort */
		uintptr_t objectAge = _objectModel->getPreservedAge(forwardedHeader);
		EvacuationRegion evacuationRegion = isNurseryAge(objectAge) ? survivor : tenure;
		MM_EvacuatorCopyspace *effectiveCopyspace = reserveOutsideCopyspace(&evacuationRegion, slotObjectSizeAfterCopy);

		/* copy object to outside copyspace */
		if (NULL != effectiveCopyspace) {
			omrobjectptr_t copyHead = (omrobjectptr_t)effectiveCopyspace->getCopyHead();
			forwardedAddress = copyForward(forwardedHeader, (fomrobject_t *)NULL, effectiveCopyspace, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);
			if (copyHead == forwardedAddress) {
				Debug_MM_true((effectiveCopyspace != &_largeCopyspace) || ((slotObjectSizeAfterCopy == _largeCopyspace.getWorkSize()) && (0 == _largeCopyspace.getWhiteSize())));
				/* object copied to effective copyspace -- see if we can release a work packet */
				if ((effectiveCopyspace == &_largeCopyspace) || (effectiveCopyspace->getWorkSize() >= _workReleaseThreshold)) {
					/* always release work from large object copyspace immediately as solo object work packet */
					MM_EvacuatorWorkPacket *work = _freeList.next();
					/* latch the space between base and copy head into a work packet and rebase copyspace at current copy head */
					work->base = (omrobjectptr_t)effectiveCopyspace->rebase(&work->length);
					/* add work packet to worklist */
					addWork(work);
				}
				/* update evacuator progress for epoch reporting */
				if ((_copiedBytesDelta[survivor] + _copiedBytesDelta[tenure]) >= _copiedBytesReportingDelta) {
					_controller->reportProgress(this, _copiedBytesDelta, &_scannedBytesDelta);
				}

			} else {
				/* object copied by other thread */
				if (effectiveCopyspace == &_largeCopyspace) {
					_whiteList[evacuationRegion].add(_largeCopyspace.trim());
				}
			}
		}
	}

	return forwardedAddress;
}

bool
MM_Evacuator::evacuateRootObject(volatile omrobjectptr_t *slotPtr)
{
	omrobjectptr_t object = *slotPtr;
	if (isInEvacuate(object)) {
		/* slot object must be evacuated -- determine before and after object size */
		MM_ForwardedHeader forwardedHeader(object);
		object = evacuateRootObject(&forwardedHeader);
		if (NULL != object) {
			*slotPtr = object;
		}
	}
	/* failure to evacuate must be reported as object in survivor space */
	return isInSurvivor(*slotPtr) || isInEvacuate(*slotPtr);
}

bool
MM_Evacuator::evacuateRootObject(GC_SlotObject* slotObject)
{
	omrobjectptr_t object = slotObject->readReferenceFromSlot();
	bool copiedToNewSpace = evacuateRootObject((volatile omrobjectptr_t *)&object);
	slotObject->writeReferenceToSlot(object);
	return copiedToNewSpace;
}

void
MM_Evacuator::evacuateThreadSlot(volatile omrobjectptr_t *objectPtrIndirect)
{
	omrobjectptr_t objectPtr = *objectPtrIndirect;
	if (NULL != objectPtr) {
		/* auto-remember stack- and thread-referenced objects */
		if (isInEvacuate(objectPtr)) {
			if (!evacuateRootObject(objectPtrIndirect) && !_env->getExtensions()->isConcurrentScavengerEnabled()) {
				Trc_MM_ParallelScavenger_copyAndForwardThreadSlot_deferRememberObject(_env->getLanguageVMThread(), *objectPtrIndirect);
				/* the object was tenured while it was referenced from the stack. Undo the forward, and process it in the rescan pass. */
				_controller->setEvacuatorFlag(MM_EvacuatorController::rescanThreadSlots, true);
				*objectPtrIndirect = objectPtr;
			}
		} else if (!_env->getExtensions()->isConcurrentScavengerEnabled() && isInTenure(objectPtr)) {
			if (_env->getExtensions()->objectModel.atomicSwitchReferencedState(objectPtr, OMR_TENURED_STACK_OBJECT_RECENTLY_REFERENCED, OMR_TENURED_STACK_OBJECT_CURRENTLY_REFERENCED)) {
				Trc_MM_ParallelScavenger_copyAndForwardThreadSlot_renewingRememberedObject(_env->getLanguageVMThread(), objectPtr, OMR_TENURED_STACK_OBJECT_RECENTLY_REFERENCED);
			}
		}
	}
}

void
MM_Evacuator::rescanThreadSlot(omrobjectptr_t *objectPtrIndirect)
{
	omrobjectptr_t objectPtr = *objectPtrIndirect;
	if (isInEvacuate(objectPtr)) {
		/* the slot is still pointing at evacuate memory. This means that it must have been left unforwarded
		 * in the first pass so that we would process it here.
		 */
		MM_ForwardedHeader forwardedHeader(objectPtr);
		omrobjectptr_t tenuredObjectPtr = forwardedHeader.getForwardedObject();
		*objectPtrIndirect = tenuredObjectPtr;

		Debug_MM_true(NULL != tenuredObjectPtr);
		Debug_MM_true(isInTenure(tenuredObjectPtr));

		rememberObject(tenuredObjectPtr, true);
		_objectModel->setRememberedBits(tenuredObjectPtr, OMR_TENURED_STACK_OBJECT_CURRENTLY_REFERENCED);
	}
}

bool
MM_Evacuator::evacuateHeap()
{
	/* TODO: deprecate this calling pattern, forced by MM_RootScanner; root scanners should not call scanHeap() directly  */
	scanHeap();

	return !isAbortedCycle();
}

bool
MM_Evacuator::isAbortedCycle()
{
	if (!_abortedCycle) {
		_abortedCycle = _controller->isAborting();
	}
	return _abortedCycle;
}

bool
MM_Evacuator::isBreadthFirst()
{
	return _controller->isEvacuatorFlagSet(MM_EvacuatorController::breadthFirstScan);
}

void
MM_Evacuator::flushRememberedSet()
{
	if (0 != _env->_scavengerRememberedSet.count) {
		_env->flushRememberedSet();
	}
}

void
MM_Evacuator::flushForWaitState()
{
	flushRememberedSet();
	_delegate.flushForWaitState();
}

uintptr_t
MM_Evacuator::flushWhitespace(MM_Evacuator::EvacuationRegion region)
{
	/* large whitespace fragements may be recycled back into the memory pool */
	uintptr_t flushed = _whiteList[region].flush(true);

	/* whitelist should be empty, top(0) should be NULL */
	Debug_MM_true(NULL == _whiteList[region].top(0));

	return flushed;
}

void
MM_Evacuator::workThreadGarbageCollect(MM_EnvironmentStandard *env)
{
	Debug_MM_true(_env == env);
	Debug_MM_true(_env->getEvacuator() == this);
	Debug_MM_true(_controller->isBoundEvacuator(_workerIndex));

	/* reserve space for scan stack and set pointers to stack bounds  */
	_stackLimit = isBreadthFirst() ? (_stackBottom + 1) : _stackCeiling;
	for (_scanStackFrame = _stackBottom; _scanStackFrame < _stackCeiling; _scanStackFrame += 1) {
		_scanStackFrame->resetScanspace();
	}
	_scanStackFrame = _whiteStackFrame = NULL;

	/* scan roots and remembered set and objects depending from these */
	scanRemembered();
	scanRoots();
	scanHeap();

	/* trick to obviate a read barrier -- objects tenured this cycle are always remebwered */
	if (!isAbortedCycle() && _controller->isEvacuatorFlagSet(MM_EvacuatorController::rescanThreadSlots)) {
		_delegate.rescanThreadSlots();
		flushRememberedSet();
	}

	/* scan clearable objects -- this may involve 0 or more phases, with evacuator threads joining in scanHeap() at end of each phase */
	while (!isAbortedCycle() && _delegate.hasClearable()) {
		/* java root clearer has repeated phases involving *deprecated* evacuateHeap() and its delegated scanClearable() leaves no unscanned work */
		if (scanClearable()) {
			/* other language runtimes should use evacuateObject() only in delegated scanClearble() and allow scanHeap() to complete each delegated phase */
			 scanHeap();
		}
	}
	_delegate.flushForEndCycle();

	/* clear whitespace from the last active frame */
	if (NULL != _whiteStackFrame) {
		_whiteList[survivor].add(_whiteStackFrame->clip());
		_whiteStackFrame->resetScanspace();
		_whiteStackFrame = NULL;
	}
	/* large survivor whitespace fragments on the whitelist may be recycled back into the memory pool */
	flushWhitespace(survivor);

	/* release unused whitespace from outside copyspaces */
	for (intptr_t space = (intptr_t)survivor; space <= (intptr_t)tenure; space += 1) {
		Debug_MM_true(isAbortedCycle() || (0 == _copyspace[space].getWorkSize()));
		MM_EvacuatorWhitespace *whitespace = _copyspace[space].trim();
		_copyspace[space].resetCopyspace();
		_whiteList[space].add(whitespace);
	}

	/* reset large copyspace (it is void of work and whitespace at this point) */
	Debug_MM_true(0 == _largeCopyspace.getWorkSize());
	Debug_MM_true(0 == _largeCopyspace.getWhiteSize());
	_largeCopyspace.resetCopyspace();

	if (!isAbortedCycle()) {
		/* prune remembered set */
		_controller->pruneRememberedSet(_env);
		Debug_MM_true(0 == getVolumeOfWork());
	} else {
		/* flush tenure whitelist before backout */
		_whiteList[tenure].flush();
		/* back out evacuation */
		_controller->setBackOutFlag(_env, backOutFlagRaised);
		_controller->completeBackOut(_env);
	}
}

bool
MM_Evacuator::evacuateRememberedObjectReferents(omrobjectptr_t objectptr)
{
	Debug_MM_true((NULL != objectptr) && isInTenure(objectptr));

	/* TODO: allow array splitting when implemented for evacuator */
	bool rememberObject = false;
	GC_ObjectScannerState objectScannerState;
	uintptr_t scannerFlags = GC_ObjectScanner::scanRoots | GC_ObjectScanner::indexableObjectNoSplit;
	GC_ObjectScanner *objectScanner = getObjectScanner(objectptr, &objectScannerState, scannerFlags);
	if (NULL != objectScanner) {
		GC_SlotObject *slotPtr;
		while (NULL != (slotPtr = objectScanner->getNextSlot())) {
			if (evacuateRootObject(slotPtr)) {
				rememberObject = true;
			}
		}
	}

	/* The remembered state of a class object also depends on the class statics */
	if (_env->getExtensions()->objectModel.hasIndirectObjectReferents((CLI_THREAD_TYPE*)_env->getLanguageVMThread(), objectptr)) {
		if (_delegate.scanIndirectObjects(objectptr)) {
			rememberObject = true;
		}
	}

	return rememberObject;
}

void
MM_Evacuator::scanRoots()
{
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2llu %2llu:     roots; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%llx; vow:%llx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */

	_delegate.scanRoots();
}

void
MM_Evacuator::scanRemembered()
{
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2llu %2llu:remembered; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%llx; vow:%llx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */

	_env->flushRememberedSet();

	_controller->scavengeRememberedSet(_env);
}

bool
MM_Evacuator::scanClearable()
{
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2llu %2llu: clearable; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%llx; vow:%llx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */
	/* if there are more root or other unreachable objects to be evacuated they can be copied and forwarded here */
	_delegate.scanClearable();

	/* run scanHeap() if scanClearable() produced more work to be scanned */
	return !_controller->hasCompletedScan();
}

void
MM_Evacuator::scanHeap()
{
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2llu %2llu:      heap; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%llx; vow:%llx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */

	/* mark start of scan cycle */
	_completedScan = false;

	/* try to find some some work to prime the scan stack */
	MM_EvacuatorWorkPacket *work = loadWork();

	while (NULL != work) {
		/* push found work to prime the stack */
		push(work);
		debugStack(" load");

		/* burn stack down until empty or gc cycle is aborted */
		while (NULL != _scanStackFrame) {
			/* scan and copy inside top stack frame until scan head == copy head or copy head >= limit at next page boundary */
			EvacuationRegion evacuationRegion = unreachable;
			uintptr_t slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy;
			GC_SlotObject *slotObject = copyInside(&slotObjectSizeBeforeCopy, &slotObjectSizeAfterCopy, &evacuationRegion);
			if (NULL != slotObject) {
				/* referents that can't be copied inside current frame are pushed up the stack or flushed to outside copyspace */
				if (reserveInsideCopyspace(slotObjectSizeAfterCopy)) {
					/* push slot object up to next stack frame */
					push(slotObject, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);
					debugStack(" push");
				} else {
					/* no stack whitespace so flush current stack frame to outside copyspace and pop */
					debugStack("flush", true);
					flush(slotObject);
				}
			} else {
				/* current stack frame has been completely scanned so pop */
				pop();
				debugStack("pop");
			}
		}

		/* load more work until scanning is complete or cycle aborted */
		work = loadWork();
	}

	/* drive stack to empty without scanning frames if aborting */
	if (isAbortedCycle()) {
		/* this leaves remaining whitespace bound to _whiteStackFrame */
		while (_scanStackFrame >= _stackBottom) {
			pop();
		}
	}

	Debug_MM_true(_completedScan);
}

void
MM_Evacuator::scanComplete()
{
	if (!isAbortedCycle()) {
		for (MM_EvacuatorScanspace *stackFrame = _stackBottom; stackFrame < _stackCeiling; stackFrame += 1) {
			Debug_MM_true(0 == stackFrame->getUnscannedSize());
			Debug_MM_true(0 == stackFrame->getWhiteSize());
		}
		Debug_MM_true(NULL == _scanStackFrame);
		Debug_MM_true(0 == _copyspace[survivor].getWorkSize());
		Debug_MM_true(0 == _copyspace[tenure].getWorkSize());
	}

	/* reset stack  */
	_stackLimit = isBreadthFirst() ? (_stackBottom + 1) : _stackCeiling;
	_scanStackFrame = NULL;
	_whiteStackFrame = NULL;

	/* all done heap scan */
	Debug_MM_true(!_completedScan);
	_completedScan = true;
}

bool
MM_Evacuator::isSplitablePointerArray(GC_SlotObject *slotObject, uintptr_t objectSizeInBytes)
{
	/* large pointer arrays are split by default but scanners for these objects may inhibit splitting eg when pruning remembered set */
	bool isSplitable = (MM_EvacuatorBase::min_split_indexable_size < objectSizeInBytes);

	/* TODO: split remembered set pointer arrays */
	if (isSplitable){
		MM_ForwardedHeader forwardedHeader(slotObject->readReferenceFromSlot());
		isSplitable &= _delegate.isIndexablePointerArray(&forwardedHeader);
	}

	/* scan work is split into work packets when the array is copied and determines how many evacuators may participate in scanning */
	return isSplitable;
}

 GC_ObjectScanner *
MM_Evacuator::getObjectScanner(omrobjectptr_t objectPtr, GC_ObjectScannerState *scannerSpace, uintptr_t flags)
{
	return _delegate.getObjectScanner(objectPtr, scannerSpace, flags);
}

GC_ObjectScanner *
MM_Evacuator::nextObjectScanner(MM_EvacuatorScanspace *scanspace, bool finalizeObjectScan)
{
	uintptr_t scannedBytes = 0;

	/* finalize object scan for current object scanner, if any */
	GC_ObjectScanner *objectScanner = scanspace->getObjectScanner();
	if (finalizeObjectScan && (NULL != objectScanner)) {
		/* advance scan head past parent object or split array segment */
		scannedBytes = _objectModel->getConsumedSizeInBytesWithHeader(objectScanner->getParentObject());
		if (scanspace->isSplitArraySegment()) {
			/* indexable object header and trailing remainder scanned bytes are counted with first array segment scanned */
			if (objectScanner->isHeadObjectScanner()) {
				scannedBytes = _splitArrayBytesToScan + (scannedBytes - ((GC_IndexableObjectScanner *)objectScanner)->getDataSizeInBytes());
			} else {
				scannedBytes = _splitArrayBytesToScan;
			}
			_stats->countWorkPacketSize(scannedBytes, _controller->_maximumWorkspaceSize);
			_splitArrayBytesToScan = 0;
		}

		/* update remembered state for parent object */
		if ((tenure == _scanStackRegion) && scanspace->getRememberedState()) {
			rememberObject(objectScanner->getParentObject());
		}
		scanspace->clearRememberedState();

		/* move scan head over scanned object and drop its object scanner */
		objectScanner = scanspace->advanceScanHead(scannedBytes);
		_scannedBytesDelta += scannedBytes;
	}

	/* advance scan head over leaf objects and objects with no scanner to set up next object scanner */
	while ((NULL == objectScanner) && (scanspace->getScanHead() < scanspace->getCopyHead())) {
		omrobjectptr_t objectPtr = (omrobjectptr_t)scanspace->getScanHead();
		objectScanner = getObjectScanner(objectPtr, scanspace->getObjectScannerState(), GC_ObjectScanner::scanHeap);
		if ((NULL == objectScanner) || objectScanner->isLeafObject()) {
			/* nothing to scan for object at scan head, drop object scanner and advance scan head to next object in scanspace */
			scannedBytes = _objectModel->getConsumedSizeInBytesWithHeader(objectPtr);
			objectScanner = scanspace->advanceScanHead(scannedBytes);
			_scannedBytesDelta += scannedBytes;
		}
	}

	/* update evacuator progress for epoch reporting */
	if ((_scannedBytesDelta >= _copiedBytesReportingDelta) || ((_copiedBytesDelta[survivor] + _copiedBytesDelta[tenure]) >= _copiedBytesReportingDelta)) {
		_controller->reportProgress(this, _copiedBytesDelta, &_scannedBytesDelta);
	}

	return objectScanner;
}

void
MM_Evacuator::push(MM_EvacuatorWorkPacket *work)
{
	Debug_MM_true(NULL != work);
	Debug_MM_true(0 < work->length);
	Debug_MM_true(NULL != work->base);
	Debug_MM_true(NULL == _whiteStackFrame);
	Debug_MM_true(NULL == _scanStackFrame);

	/* reset and adjust stack bounds and prepare stack to receive work packet in bottom frame */
	_stackLimit = (!_controller->areAnyEvacuatorsStalled() && !isBreadthFirst()) ? _stackCeiling : (_stackBottom + 1);
	_scanStackRegion = isInSurvivor(work->base) ? survivor : tenure;
	_scanStackFrame = _whiteStackFrame = _stackBottom;

	if (isSplitArrayPacket(work)) {
		GC_IndexableObjectScanner *scanner = (GC_IndexableObjectScanner *)_scanStackFrame->getObjectScannerState();
		/* the object scanner must be set in the scanspace at this point for split arrays -- work offset/length are array indices in this case */
		_delegate.getSplitPointerArrayObjectScanner(work->base, scanner, work->offset - 1, work->length, GC_ObjectScanner::scanHeap);
		/* the work packet contains a split array segment -- set scanspace base = array, scan = copy = limit = end = base + object-size */
		_scanStackFrame->setSplitArrayScanspace((uint8_t *)work->base, ((uint8_t *)work->base + _objectModel->getConsumedSizeInBytesWithHeader(work->base)), scanner);
		/* split array segments are scanned only on the bottom frame of the stack -- save the length of segment for updating _scannedBytes when this segment has been scanned */
		_splitArrayBytesToScan = sizeof(fomrobject_t) * work->length;
	} else {
		_stats->countWorkPacketSize(work->length, _controller->_maximumWorkspaceSize);
		/* the work packet contains a contiguous series of objects -- set scanspace base = scan, copy = limit = end = scan + length */
		_scanStackFrame->setScanspace((uint8_t *)work->base, (uint8_t *)work->base + work->length, work->length, _maxInsideCopySize);
		/* object scanners will be instantiated and _scannedBytes updated as objects in scanspace are scanned */
		_splitArrayBytesToScan = 0;
	}

#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2llu %2llu: push work; base:%llx; length:%llx; vow:%llx; sow:%llx; tow:%llx ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex,
				(uintptr_t)work->base, work->length, getVolumeOfWork(), _copyspace[survivor].getWorkSize(), _copyspace[tenure].getWorkSize());
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("\n");
	}
#endif /* defined(EVACUATOR_DEBUG) */

	_freeList.add(work);
}

void
MM_Evacuator::push(GC_SlotObject *slotObject, uintptr_t slotObjectSizeBeforeCopy, uintptr_t slotObjectSizeAfterCopy)
{
	/* copy and forward slot object inside peak frame -- this cannot change remembered state of parent so ignore returned pointer */
	copyForward(slotObject, _whiteStackFrame, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);

	if (_whiteStackFrame->getScanHead() < _whiteStackFrame->getCopyHead()) {
		/* object was evacuated in this copy/forward so consider it pushed */
		_scanStackFrame = _whiteStackFrame;
	}

	Debug_MM_true(_scanStackFrame < _stackLimit);
}

void
MM_Evacuator::pop()
{
	/* pop the stack */
	if (_stackBottom < _scanStackFrame) {
		if (isAbortedCycle()) {
			/* clear stack frame if aborting cycle */
			_whiteList[_scanStackRegion].add(_scanStackFrame->clip());
			if (NULL != _whiteStackFrame) {
				_scanStackFrame->resetScanspace();
				_whiteStackFrame = NULL;
			}
		}
		_scanStackFrame -= 1;
	} else {
		Debug_MM_true(_stackBottom == _scanStackFrame);
		/* stack empty -- recycle any whitespace remaining in peak or bottom stack frame */
		if (NULL != _whiteStackFrame) {
			_whiteList[_scanStackRegion].add(_whiteStackFrame->clip());
			_whiteStackFrame->resetScanspace();
		} else {
			_whiteList[_scanStackRegion].add(_scanStackFrame->clip());
		}
		/* clear the stack */
		_scanStackFrame = _whiteStackFrame = NULL;
	}
}

void
MM_Evacuator::flush(GC_SlotObject *slotObject)
{
	GC_ObjectScanner *objectScanner = nextObjectScanner(_scanStackFrame, false);
	do {
		if (NULL != slotObject) {
			/* check whether slot is in evacuate space */
			omrobjectptr_t object = slotObject->readReferenceFromSlot();
			if (isInEvacuate(object)) {
				/* slot object must be evacuated */
				uintptr_t slotObjectSizeBeforeCopy = 0;
				uintptr_t slotObjectSizeAfterCopy = 0;
				uintptr_t hotFieldAlignmentDescriptor = 0;
				MM_ForwardedHeader forwardedHeader(object);
				if (!forwardedHeader.isForwardedPointer()) {
					uintptr_t objectAge = _objectModel->getPreservedAge(&forwardedHeader);
					EvacuationRegion evacuationRegion = isNurseryAge(objectAge) ? survivor : tenure;
					_objectModel->calculateObjectDetailsForCopy(_env, &forwardedHeader, &slotObjectSizeBeforeCopy, &slotObjectSizeAfterCopy, &hotFieldAlignmentDescriptor);

					/* copy slot object to outside copyspace */
					copyOutside(slotObject, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy, evacuationRegion);
				} else {
					/* slot object already evacuated -- just update slot */
					slotObject->writeReferenceToSlot(forwardedHeader.getForwardedObject());
				}
			}
			if (tenure == _scanStackRegion) {
				object = slotObject->readReferenceFromSlot();
				_scanStackFrame->updateRememberedState(isInSurvivor(object));
				Debug_MM_true(!isInEvacuate(object) || isAbortedCycle());
			}
		}
		
		if (setStackLimit()) {
			return;
		}

		/* get scanner for next object in frame and find first slot */
		slotObject = objectScanner->getNextSlot();
		while ((NULL == slotObject) && (NULL != objectScanner)) {
			/* get scanner for next object at scan head, if any, and try to fetch next slot */
			objectScanner = nextObjectScanner(_scanStackFrame);
			if (NULL != objectScanner) {
				slotObject = objectScanner->getNextSlot();
			}
		}
	} while (NULL != objectScanner);

	/* current stack frame has been completely scanned so pop */
	pop();
	debugStack("pop");
}

bool
MM_Evacuator::setStackLimit()
{
	if (!isBreadthFirst()) {
		/* when there >0 stalled evacuators force bread-first flushing to outside copyspaces */
		bool areAnyEvacuatorsStalled = _controller->areAnyEvacuatorsStalled();
		
		if (areAnyEvacuatorsStalled && (_stackLimit == _stackCeiling)) {
			/* recalculate work release threshold to reflect stalling context and worklist volume */
			_workReleaseThreshold = _controller->calculateOptimalWorkPacketSize(getVolumeOfWork());
		
			/* limit stack to force flushing until stalled evacuator condition clears */
			_stackLimit = _stackBottom + 1;
		} else if (!areAnyEvacuatorsStalled && (_stackLimit < _stackCeiling)) {
			/* recalculate work release threshold to reflect worklist volume after stall condition clears */
			_workReleaseThreshold = _controller->calculateOptimalWorkPacketSize(getVolumeOfWork());
		
			/* restore the stack limit */
			_stackLimit = _stackCeiling;
			
			/* return true to signal flush() to stop scanning frame */
			return true;
		}
	}
	
	return false;
}

GC_SlotObject *
MM_Evacuator::copyInside(uintptr_t *slotObjectSizeBeforeCopy, uintptr_t *slotObjectSizeAfterCopy, EvacuationRegion *evacuationRegion)
{
	MM_EvacuatorScanspace * const stackFrame = _scanStackFrame;
	GC_ObjectScanner *objectScanner = nextObjectScanner(_scanStackFrame, false);
	while (!isAbortedCycle() && (NULL != objectScanner)) {
		Debug_MM_true((objectScanner->getParentObject() == (omrobjectptr_t)stackFrame->getScanHead()) || (stackFrame->isSplitArraySegment() && (objectScanner->getParentObject() == (omrobjectptr_t)stackFrame->getBase())));

		/* loop through reference slots in current object at scan head */
		GC_SlotObject *slotObject = objectScanner->getNextSlot();
		while (NULL != slotObject) {
			omrobjectptr_t object = slotObject->readReferenceFromSlot();
			if (isInEvacuate(object)) {
				MM_ForwardedHeader forwardedHeader(object);
				if (!forwardedHeader.isForwardedPointer()) {
					uintptr_t hotFieldAlignmentDescriptor = 0;
					/* slot object must be evacuated -- determine before and after object size */
					_objectModel->calculateObjectDetailsForCopy(_env, &forwardedHeader, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy, &hotFieldAlignmentDescriptor);

					/* copy and forward the slot object */
					uintptr_t objectAge = _objectModel->getPreservedAge(&forwardedHeader);
					*evacuationRegion = isNurseryAge(objectAge) ? survivor : tenure;
					if ((_scanStackRegion == *evacuationRegion) && (_maxInsideCopySize >= *slotObjectSizeAfterCopy)) {
						/* copy inside this stack frame copy head has not passed frame limit, otherwise push */
						uintptr_t stackRemainder = stackFrame->getWhiteSize();
						if ((stackFrame->getCopyHead() < stackFrame->getCopyLimit()) && (stackRemainder >= *slotObjectSizeAfterCopy)) {
							/* copy small slot object inside this stack frame and update the reference slot with forwarding address */
							object = copyForward(&forwardedHeader, slotObject->readAddressFromSlot(), stackFrame, *slotObjectSizeBeforeCopy, *slotObjectSizeAfterCopy);
							slotObject->writeReferenceToSlot(object);
						} else if ((stackRemainder >= *slotObjectSizeAfterCopy) || (stackRemainder < MM_EvacuatorBase::max_scanspace_remainder) || objectScanner->isIndexableObject()) {
							/* push small slot object up the scan stack if there is sufficient stack whitespace remaining or trailing whitespace can be trimmed */
							return slotObject;
						} else {
							/* redirect small objects outside until stack whitespace has depleted below max_scanspace_remainder */
							goto outside;
						}
					} else {
outside:				Debug_MM_true((_scanStackRegion != *evacuationRegion) || (_maxInsideCopySize < *slotObjectSizeAfterCopy) || ((stackFrame->getWhiteSize() >= MM_EvacuatorBase::max_scanspace_remainder) && !objectScanner->isIndexableObject()));
						/* copy to outside copyspace if not possible to copy inside stack */
						*evacuationRegion = copyOutside(slotObject, *slotObjectSizeBeforeCopy, *slotObjectSizeAfterCopy, *evacuationRegion);
					}
				} else {
					/* slot object already evacuated -- just update slot */
					object = forwardedHeader.getForwardedObject();
					slotObject->writeReferenceToSlot(object);
					*evacuationRegion = getEvacuationRegion(object);
				}
			}

			/* if scanning a tenured object update its remembered state */
			if (tenure == _scanStackRegion) {
				object = slotObject->readReferenceFromSlot();
				stackFrame->updateRememberedState(isInSurvivor(object));
				Debug_MM_true(!isInEvacuate(object) || isAbortedCycle());
			}

			/* if there are stalled evacuators limit stack to force flushing all copy to outside copyspaces */
			setStackLimit();

			/* continue with next slot object */
			slotObject = objectScanner->getNextSlot();
		}

		/* continue with next non-leaf object in frame, if any */
		objectScanner = nextObjectScanner(stackFrame);
	}

	/* this stack frame has been completely scanned */
	Debug_MM_true(isAbortedCycle() || (stackFrame->getScanHead() == stackFrame->getCopyHead()));
	*evacuationRegion = unreachable;
	*slotObjectSizeBeforeCopy = 0;
	*slotObjectSizeAfterCopy = 0;

	/* return NULL to pop scan stack */
	return NULL;
}

bool
MM_Evacuator::reserveInsideCopyspace(uintptr_t slotObjectSizeAfterCopy)
{
	Debug_MM_true(_whiteStackFrame >= _scanStackFrame);

	/* this is called before every push() to prepare next frame to receive an object */
	MM_EvacuatorScanspace *nextStackFrame = _scanStackFrame + 1;
	if (nextStackFrame < _stackLimit) {
		Debug_MM_true(nextStackFrame->getBase() <= nextStackFrame->getScanHead());
		Debug_MM_true(nextStackFrame->getScanHead() <= nextStackFrame->getCopyHead());
		Debug_MM_true(nextStackFrame->getCopyLimit() <= nextStackFrame->getEnd());

		/* the next stack frame is often a frame that was just popped while holding stack whitespace and is set as _whiteStackFrame */
		if (slotObjectSizeAfterCopy > nextStackFrame->getWhiteSize()) {
			Debug_MM_true((nextStackFrame == _whiteStackFrame) || (0 == nextStackFrame->getWhiteSize()));

			/* object won't fit in remaining whitespace in next frame -- ensure that whitespace in _whiteStackFrame is sufficient to hold object */
			if (slotObjectSizeAfterCopy > _whiteStackFrame->getWhiteSize()) {
				_whiteList[_scanStackRegion].add(_whiteStackFrame->clip());
				/* not enough room in _whiteStackFrame so try to get whitespace from top of evacuation region whitelist */
				MM_EvacuatorWhitespace *whitespace = _whiteList[_scanStackRegion].top(slotObjectSizeAfterCopy);
				if (NULL == whitespace) {
					/* get a new chunk of whitespace from the heap to burn down */
					whitespace = _controller->getInsideFreespace(this, _scanStackRegion, slotObjectSizeAfterCopy);
					if (NULL == whitespace) {
						/* force outside copy */
#if defined(EVACUATOR_DEBUG)
						if (_controller->_debugger.isDebugAllocate()) {
							OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
							omrtty_printf("%5lu %2llu %2llu:stack fail; %s stack; required:%llx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch,
									getWorkerIndex(), ((MM_Evacuator::survivor == _scanStackRegion) ? "survivor" : "tenure"), slotObjectSizeAfterCopy);
						}
#endif /* defined(EVACUATOR_DEBUG) */
						return false;
					}
				}
				/* set up next stack frame to receive object into allocated whitespace */
				nextStackFrame->setScanspace((uint8_t *)whitespace, (uint8_t *)whitespace, whitespace->length(), _maxInsideCopySize, whitespace->isLOA());
				_whiteStackFrame = nextStackFrame;
			} else if (nextStackFrame != _whiteStackFrame) {
				/* set up next stack frame to receive object into whitespace pulled from _whiteStackFrame */
				nextStackFrame->pullWhitespace(_whiteStackFrame, _maxInsideCopySize);
				_whiteStackFrame = nextStackFrame;
			}
		}

		/* next stack frame is holding stack whitespace */
		Debug_MM_true(_whiteStackFrame == nextStackFrame);
		return true;
	}

	return false;
}

MM_Evacuator::EvacuationRegion
MM_Evacuator::copyOutside(GC_SlotObject *slotObject, uintptr_t slotObjectSizeBeforeCopy, uintptr_t slotObjectSizeAfterCopy, EvacuationRegion evacuationRegion)
{
	/* reserve copy space */
	bool isSplitable = isSplitablePointerArray(slotObject, slotObjectSizeAfterCopy);
	MM_EvacuatorCopyspace *effectiveCopyspace = reserveOutsideCopyspace(&evacuationRegion, slotObjectSizeAfterCopy, isSplitable);
	if (NULL != effectiveCopyspace) {
		Debug_MM_true(slotObjectSizeAfterCopy <= effectiveCopyspace->getWhiteSize());
		/* copy slot object to effective outside copyspace */
		omrobjectptr_t copyHead = (omrobjectptr_t)effectiveCopyspace->getCopyHead();
		if (copyHead == copyForward(slotObject, effectiveCopyspace, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy)) {
			/* object copied into effective copyspace -- check for sharable work to distribute */
			uintptr_t worksize = effectiveCopyspace->getWorkSize();
			if (effectiveCopyspace == &_largeCopyspace) {
				MM_EvacuatorWorkPacket *work = _freeList.next();
				if (isSplitable) {
					/* record 1-based array offsets to mark split array work packets */
					uintptr_t offset = 1;
					uintptr_t elements = 0;

					/* distribute elements to segments as evenly as possible and take largest segment first */
					_delegate.getIndexableDataBounds(copyHead, &elements);
					uintptr_t segments = (elements / MM_EvacuatorBase::max_split_segment_elements) + ((0 != (elements % MM_EvacuatorBase::max_split_segment_elements)) ? 1 : 0);
					uintptr_t elementsPerSegment = elements / segments;
					uintptr_t elementsThisSegment = elementsPerSegment + (elements % segments);

					/* lock worklist and add each array segment as a work packet */
					omrthread_monitor_enter(_mutex);
					while (0 < segments) {
						MM_EvacuatorWorkPacket *work = _freeList.next();
						work->base = copyHead;
						work->offset = offset;
						work->length = elementsThisSegment;
						_workList.add(work, &_freeList);
						offset += elementsThisSegment;
						elementsThisSegment = elementsPerSegment;
						segments -= 1;
					}
					omrthread_monitor_exit(_mutex);

					/* tell the world about it */
					_controller->notifyOfWork();
				} else {
					/* set up work packet contained a single large scalar or non-splitable array object */
					Debug_MM_true((slotObjectSizeAfterCopy == worksize) && (0 == effectiveCopyspace->getWhiteSize()));
					work->base = (omrobjectptr_t)effectiveCopyspace->rebase(&work->length);
					addWork(work);
				}
				/* prepare the large object copyspace for next use */
				_largeCopyspace.resetCopyspace();
			} else if ((effectiveCopyspace != _scanStackFrame) && (worksize >= _workReleaseThreshold)) {
				/* strip work from head of effective copyspace into a work packet, leaving only trailing whitespace in copyspace */
				MM_EvacuatorWorkPacket *work = _freeList.next();
				work->base = (omrobjectptr_t)effectiveCopyspace->rebase(&work->length);
				addWork(work);
			}
		} else if (effectiveCopyspace == &_largeCopyspace) {
			/* object copied by other thread */
			_whiteList[evacuationRegion].add(_largeCopyspace.trim());
			/* prepare the large object copyspace for next use */
			_largeCopyspace.resetCopyspace();
		}
	} else {
		evacuationRegion = unreachable;
	}

	return evacuationRegion;
}

MM_EvacuatorCopyspace *
MM_Evacuator::reserveOutsideCopyspace(EvacuationRegion *evacuationRegion, uintptr_t slotObjectSizeAfterCopy, bool useLargeCopyspace)
{
	EvacuationRegion preferredRegion = *evacuationRegion;
	MM_EvacuatorWhitespace *whitespace = NULL;
	MM_EvacuatorCopyspace *copyspace = NULL;

	/* incoming useLargeCopyspace request indicates a large pointer array is to be copied into a solo object work packet or series of split segment work packets */
	if (!useLargeCopyspace) {
		/* use an outside copyspace if possible -- use large object space only if object will not fit in copyspace remainder whitespace */
		copyspace = &_copyspace[preferredRegion];
		uintptr_t copyspaceRemainder = copyspace->getWhiteSize();
		if (slotObjectSizeAfterCopy > copyspaceRemainder) {
			copyspace = NULL;
			/* try the preferred region whitelist first -- but only if it is presenting a large chunk on top */
			if ((slotObjectSizeAfterCopy <= _whiteList[preferredRegion].top()) && (_whiteList[preferredRegion].top() >= MM_EvacuatorBase::max_copyspace_remainder)) {
				whitespace = _whiteList[preferredRegion].top(slotObjectSizeAfterCopy);
			} else if ((copyspaceRemainder >= MM_EvacuatorBase::max_copyspace_remainder) && (_scanStackRegion == preferredRegion) && (NULL != _scanStackFrame) &&
				(MM_EvacuatorBase::max_split_segment_size >= slotObjectSizeAfterCopy) && (slotObjectSizeAfterCopy <= _scanStackFrame->getWhiteSize())) {
				/* if object size < split array segment size it can be written into the stack if currently scanning frame in preferred region */
				copyspace = _scanStackFrame;
			} else {
				/* try to allocate whitespace from the preferred region */
				whitespace = _controller->getOutsideFreespace(this, preferredRegion, copyspaceRemainder, slotObjectSizeAfterCopy);
				if (NULL == whitespace) {
					/* try the other evacuation region (survivor/tenure) */
					*evacuationRegion = otherOutsideRegion(*evacuationRegion);
					copyspaceRemainder = _copyspace[*evacuationRegion].getWhiteSize();
					if (slotObjectSizeAfterCopy > copyspaceRemainder) {
						/* try to allocate from other region whitelist -- but only if it is presenting a large chunk on top */
						if ((slotObjectSizeAfterCopy <= _whiteList[*evacuationRegion].top()) && (_whiteList[*evacuationRegion].top() >= MM_EvacuatorBase::max_copyspace_remainder)) {
							whitespace = _whiteList[*evacuationRegion].top(slotObjectSizeAfterCopy);
						}
						if (NULL == whitespace) {
							/* try to allocate from other region */
							whitespace = _controller->getOutsideFreespace(this, *evacuationRegion, copyspaceRemainder, slotObjectSizeAfterCopy);
							if (NULL == whitespace) {
								/* last chance -- outside regions and whitelists exhausted, try to steal the stack's whitespace */
								MM_EvacuatorScanspace *whiteStackFrame = (NULL != _whiteStackFrame) ? _whiteStackFrame : _scanStackFrame;
								if ((NULL != whiteStackFrame) && (slotObjectSizeAfterCopy <= whiteStackFrame->getWhiteSize())) {
									whitespace = whiteStackFrame->clip();
									*evacuationRegion = _scanStackRegion;
								}
							}
						}
					} else {
						/* the other outside region has enough whitespace to receive the object */
						copyspace = &_copyspace[*evacuationRegion];
					}
				}
			}
		}
	} else {
		/* allocate exactly enough heap memory to contain the large object */
		whitespace = _controller->getOutsideFreespace(this, *evacuationRegion, MM_EvacuatorBase::max_copyspace_remainder, slotObjectSizeAfterCopy);
		if (NULL == whitespace) {
			*evacuationRegion = otherOutsideRegion(*evacuationRegion);
			whitespace = _controller->getOutsideFreespace(this, *evacuationRegion, MM_EvacuatorBase::max_copyspace_remainder, slotObjectSizeAfterCopy);
		}
		Debug_MM_true((NULL == whitespace) || (slotObjectSizeAfterCopy == whitespace->length()));
	}

	if (NULL == copyspace) {
		if (NULL != whitespace) {
			/* load whitespace into outside or large copyspace for evacuation region */
			if (!useLargeCopyspace && (slotObjectSizeAfterCopy < whitespace->length())) {
				/* object will leave remainder whitespace after copy end so load whitespace into outside copyspace */
				copyspace = &_copyspace[*evacuationRegion];
				/* distribute any existing work in copyspace */
				if (0 < copyspace->getWorkSize()) {
					MM_EvacuatorWorkPacket *work = _freeList.next();
					work->base = (omrobjectptr_t)copyspace->rebase(&work->length);
					addWork(work);
				}
				/* trim whitespace from copyspace to whitelist */
				_whiteList[*evacuationRegion].add(copyspace->trim());
			} else {
				/* object will be exactly contained in whitespace so load whitespace into large copyspace */
				copyspace = &_largeCopyspace;
				Debug_MM_true4(_env, (slotObjectSizeAfterCopy > MM_EvacuatorBase::max_copyspace_remainder), "object should fit in outside %s copyspace tail: object=%llx; survivor-tail=%llx; tenure-tail=%llx\n",
						((survivor == *evacuationRegion) ? "survivor" : "tenure"), slotObjectSizeAfterCopy, _copyspace[survivor].getWhiteSize(), _copyspace[tenure].getWhiteSize());
				Debug_MM_true(slotObjectSizeAfterCopy == whitespace->length());
				Debug_MM_true(0 == copyspace->getWorkSize());
				Debug_MM_true(0 == copyspace->getWhiteSize());
			}
			/* load the reserved whitespace into the selected copyspace */
			copyspace->setCopyspace((uint8_t*)whitespace, (uint8_t*)whitespace, whitespace->length(), whitespace->isLOA());
		} else {
			if (survivor == preferredRegion) {
				_stats->_failedFlipCount += 1;
				_stats->_failedFlipBytes += slotObjectSizeAfterCopy;
			} else {
				_stats->_failedTenureCount += 1;
				_stats->_failedTenureBytes += slotObjectSizeAfterCopy;
				_stats->_failedTenureLargest = OMR_MAX(slotObjectSizeAfterCopy, _stats->_failedTenureLargest);
			}
			/* abort the evacuation and broadcast this to other evacuators through controller */
			_abortedCycle = _controller->setAborting(this);
		}
	}

	return copyspace;
}

MMINLINE omrobjectptr_t
MM_Evacuator::copyForward(GC_SlotObject *slotObject, MM_EvacuatorCopyspace *copyspace, uintptr_t originalLength, uintptr_t forwardedLength)
{
	MM_ForwardedHeader forwardedHeader(slotObject->readReferenceFromSlot());
	omrobjectptr_t copiedObject = copyForward(&forwardedHeader, slotObject->readAddressFromSlot(), copyspace, originalLength, forwardedLength);
	slotObject->writeReferenceToSlot(copiedObject);
	return copiedObject;
}

MMINLINE omrobjectptr_t
MM_Evacuator::copyForward(MM_ForwardedHeader *forwardedHeader, fomrobject_t *referringSlotAddress, MM_EvacuatorCopyspace *copyspace, uintptr_t originalLength, uintptr_t forwardedLength)
{
	/* if object not already forwarded try to set forwarding address to the copy head in copyspace */
	omrobjectptr_t forwardingAddress = (omrobjectptr_t)copyspace->getCopyHead();
	omrobjectptr_t forwardedAddress = forwardedHeader->isForwardedPointer() ? forwardedHeader->getForwardedObject() : forwardedHeader->setForwardedObject(forwardingAddress);
	if (forwardedAddress == forwardingAddress) {
#if defined(OMR_VALGRIND_MEMCHECK)
		valgrindMempoolAlloc(_env->getExtensions(), (uintptr_t)forwardedAddress, forwardedLength);
#endif /* defined(OMR_VALGRIND_MEMCHECK) */
#if defined(EVACUATOR_DEBUG)
		_delegate.debugValidateObject(forwardedHeader);
#endif /* defined(EVACUATOR_DEBUG) */

		/* forwarding address set by this thread -- object will be evacuated to the copy head in copyspace */
		memcpy(forwardedAddress, forwardedHeader->getObject(), originalLength);

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		/* update proximity and size metrics */
		_stats->countCopyDistance((uintptr_t)referringSlotAddress, (uintptr_t)forwardedAddress);
		_stats->countObjectSize(forwardedLength);
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

		/* update scavenger stats */
		uintptr_t objectAge = _objectModel->getPreservedAge(forwardedHeader);
		if (isInTenure(forwardedAddress)) {
			_stats->_tenureAggregateCount += 1;
			_stats->_tenureAggregateBytes += originalLength;
			_stats->getFlipHistory(0)->_tenureBytes[objectAge + 1] += forwardedLength;
#if defined(OMR_GC_LARGE_OBJECT_AREA)
			if (copyspace->isLOA()) {
				_stats->_tenureLOACount += 1;
				_stats->_tenureLOABytes += originalLength;
			}
#endif /* OMR_GC_LARGE_OBJECT_AREA */
			_copiedBytesDelta[tenure] += forwardedLength;
		} else {
			Debug_MM_true(isInSurvivor(forwardedAddress));
			_stats->_flipCount += 1;
			_stats->_flipBytes += originalLength;
			_stats->getFlipHistory(0)->_flipBytes[objectAge + 1] += forwardedLength;
			_copiedBytesDelta[survivor] += forwardedLength;
		}

		/* update object age and finalize copied object header */
		if (tenure == getEvacuationRegion(forwardedAddress)) {
			objectAge = STATE_NOT_REMEMBERED;
		} else if (objectAge < OBJECT_HEADER_AGE_MAX) {
			objectAge += 1;
		}

		/* copy the preserved fields from the forwarded header into the destination object */
		forwardedHeader->fixupForwardedObject(forwardedAddress);
		/* fix the flags in the destination object */
		_objectModel->fixupForwardedObject(forwardedHeader, forwardedAddress, objectAge);

#if defined(EVACUATOR_DEBUG)
		_delegate.debugValidateObject(forwardedAddress);
		if (_controller->_debugger.isDebugCopy()) {
			OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
			char className[32];
			omrobjectptr_t parent = (NULL != _scanStackFrame) ? _scanStackFrame->getObjectScanner()->getParentObject() : NULL;
			omrtty_printf("%5lu %2llu %2llu:%c copy %3s; base:%llx; copy:%llx; end:%llx; free:%llx; %llx %s %llx -> %llx %llx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex,
					(NULL != _scanStackFrame) && ((uint8_t *)forwardedAddress >= _scanStackFrame->getBase()) && ((uint8_t *)forwardedAddress < _scanStackFrame->getEnd()) ? 'I' : 'O',
					isInSurvivor(forwardedAddress) ? "new" : "old",	(uintptr_t)copyspace->getBase(), (uintptr_t)copyspace->getCopyHead(), (uintptr_t)copyspace->getEnd(), copyspace->getWhiteSize(),
					(uintptr_t)parent, _delegate.debugGetClassname(forwardedAddress, className, 32), (uintptr_t)forwardedHeader->getObject(), (uintptr_t)forwardedAddress, forwardedLength);
		}
#endif /* defined(EVACUATOR_DEBUG) */

		/* object was copied by this thread -- advance the copy head in the receiving copyspace */
		copyspace->advanceCopyHead(forwardedLength);
	}

	return forwardedAddress;
}

/* TODO: Move back to MM_Scavenger as virtual method accessible through MM_EvacuatorController */
bool
MM_Evacuator::shouldRememberObject(omrobjectptr_t objectPtr)
{
	Debug_MM_true((NULL != objectPtr) && isInTenure(objectPtr));

	/* This method should be only called for RS pruning scan (whether in backout or not),
	 * which is either single threaded (overflow or backout), or if multi-threaded it does no work sharing.
	 * So we must not split, if it's indexable
	 */
	GC_ObjectScannerState objectScannerState;
	uintptr_t scannerFlags = GC_ObjectScanner::scanRoots | GC_ObjectScanner::indexableObjectNoSplit;
	GC_ObjectScanner *objectScanner = getObjectScanner(objectPtr, &objectScannerState, scannerFlags);
	if (NULL != objectScanner) {
		GC_SlotObject *slotPtr;
		while (NULL != (slotPtr = objectScanner->getNextSlot())) {
			omrobjectptr_t slotObjectPtr = slotPtr->readReferenceFromSlot();
			if (NULL != slotObjectPtr) {
				if (isInSurvivor(slotObjectPtr)) {
					return true;
				} else {
					Debug_MM_true(isInTenure(slotObjectPtr) || (isInEvacuate(slotObjectPtr) && objectScanner->isIndexableObject()));
				}
			}
		}
	}

	/* The remembered state of a class object also depends on the class statics */
	if (_env->getExtensions()->objectModel.hasIndirectObjectReferents((CLI_THREAD_TYPE*)_env->getLanguageVMThread(), objectPtr)) {
		return _delegate.objectHasIndirectObjectsInNursery(objectPtr);
	}

	return false;
}

void
MM_Evacuator::rememberObject(omrobjectptr_t object, bool isThreadReference)
{
	Debug_MM_true(isInTenure(object));
	/* try to set the REMEMBERED bit in the flags field (if it hasn't already been set) */
	if (_objectModel->atomicSetRememberedState(object, STATE_REMEMBERED)) {
		/* the object has been successfully marked as REMEMBERED - allocate an entry in the remembered set */
		Debug_MM_true(_objectModel->isRemembered(object));
		Debug_MM_true(!_controller->_debugger.isDebugRemembered() || isThreadReference || shouldRememberObject(object));
		if (_env->_scavengerRememberedSet.fragmentCurrent >= _env->_scavengerRememberedSet.fragmentTop) {
			/* there isn't enough room in the current fragment - allocate a new one */
			if (allocateMemoryForSublistFragment(_env->getOmrVMThread(), (J9VMGC_SublistFragment*)&_env->_scavengerRememberedSet)) {
				/* Failed to allocate a fragment - set the remembered set overflow state and exit */
				if (!_env->getExtensions()->isRememberedSetInOverflowState()) {
					_stats->_causedRememberedSetOverflow = 1;
				}
				_env->getExtensions()->setRememberedSetOverflowState();
				return;
			}
		}
		/* there is at least 1 free entry in the fragment - use it */
		_env->_scavengerRememberedSet.count++;
		uintptr_t *rememberedSetEntry = _env->_scavengerRememberedSet.fragmentCurrent++;
		*rememberedSetEntry = (uintptr_t)object;
	}
}

void
MM_Evacuator::receiveWhitespace(MM_EvacuatorWhitespace *whitespace)
{
	EvacuationRegion whiteRegion = getEvacuationRegion(whitespace);
	Debug_MM_true(whiteRegion < evacuate);
	_whiteList[whiteRegion].add(whitespace);
}

void
MM_Evacuator::addWork(MM_EvacuatorWorkPacket *work)
{
	omrthread_monitor_enter(_mutex);
	_workList.add(work, &_freeList);
	omrthread_monitor_exit(_mutex);

	_controller->notifyOfWork();

	_workReleaseThreshold = _controller->calculateOptimalWorkPacketSize(getVolumeOfWork());
}

MM_EvacuatorWorkPacket *
MM_Evacuator::findWork()
{
	MM_EvacuatorWorkPacket *work = NULL;

	if (!isAbortedCycle()) {
		/* select prospective donor as the evacuator with greatest volume of work */
		MM_Evacuator *max = this;
		for (MM_Evacuator *evacuator = _controller->getNextEvacuator(max); max != evacuator; evacuator = _controller->getNextEvacuator(evacuator)) {
			if (_controller->isBoundEvacuator(evacuator->getWorkerIndex()) && (evacuator->getVolumeOfWork() > max->getVolumeOfWork())) {
				max = evacuator;
			}
		}

		if (0 < max->getVolumeOfWork()) {
			MM_Evacuator *donor = max;
			/* no work obtained from selected donor, poll for the next evacuator that can donate work or bail if poll wraps to max */
			do {
				Debug_MM_true(this != donor);
				/* skip involved evacuators */
				if (0 == omrthread_monitor_try_enter(donor->_mutex)) {
					if (0 < donor->getVolumeOfWork()) {
						/* this evacuator holds the controller mutex so does not take its worklist mutex here */
						_workList.add(donor->_workList.next(), &_freeList);

						/* level this evacuator's volume of work up with donor */
						MM_EvacuatorWorkPacket *next = donor->_workList.peek();
						while (NULL != next) {
							if ((getVolumeOfWork() + next->length) < (donor->getVolumeOfWork() - next->length)) {
								_workList.add(donor->_workList.next(), &_freeList);
								next = donor->_workList.peek();
							} else {
								break;
							}
						}

						/* this evacuator's worklist should not be empty here */
						work = _workList.next();
					}
					omrthread_monitor_exit(donor->_mutex);
				}

				if (NULL == work) {
					/* no work from selected donor so continue donor enumeration, skipping evacuators with no work */
					do {
						/* bail with no work if donor wraps around to max */
						donor = _controller->getNextEvacuator(donor);
					} while ((donor != max) && (0 == donor->getVolumeOfWork()));
				} else {
#if defined(EVACUATOR_DEBUG)
					if (_controller->_debugger.isDebugWork()) {
						OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
						omrtty_printf("%5lu %2llu %2llu: pull work; base:%llx; length:%llx; vow:%llx; donor:%llx; donor-vow:%llx; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch,
								_workerIndex, (uintptr_t)work->base, work->length, getVolumeOfWork(),donor->_workerIndex, donor->getVolumeOfWork());
						_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
						_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
						omrtty_printf("\n");
					}
#endif /* defined(EVACUATOR_DEBUG) */
					break;
				}

			} while (donor != max);
		}
	}

	return work;
}

MM_EvacuatorWorkPacket *
MM_Evacuator::loadWork()
{
	Debug_MM_true((NULL == _scanStackFrame) || isAbortedCycle());

	MM_EvacuatorWorkPacket *work = NULL;
	if (!isAbortedCycle()) {
		/* try to take work from worklist */
		omrthread_monitor_enter(_mutex);
		work = _workList.next();
		omrthread_monitor_exit(_mutex);
	}

	if (NULL == work) {
		/* check for nonempty outside copyspace to provide work in case we can't pull from other evacuators */
		EvacuationRegion largestOutsideCopyspaceRegion = (_copyspace[survivor].getWorkSize() > _copyspace[tenure].getWorkSize()) ? survivor : tenure;
		bool hasOutsideWork = (0 < _copyspace[largestOutsideCopyspaceRegion].getWorkSize());

		/* worklist is empty, or aborting; if nothing in outside copyspaces or no other evacuators are stalled try to pull work from other evacuators */
		if (!_controller->areAnyEvacuatorsStalled() || !hasOutsideWork || isAbortedCycle()) {
			flushForWaitState();
			_controller->reportProgress(this, _copiedBytesDelta, &_scannedBytesDelta);

			_controller->acquireController();
			work = findWork();
			if ((NULL == work) && (!hasOutsideWork || isAbortedCycle())) {
#if defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS)
				uint64_t waitStartTime = startWaitTimer();
#endif /* defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS) */

				/* controller will release evacuator from stall when work is received or all other evacuators have stalled to complete or abort scan */
				while (_controller->isWaitingToCompleteStall(this, work)) {
					/* wait for work or until controller signals all evacuators to complete or abort scan */
					_controller->waitForWork();
					work = findWork();
				}
				/* continue scanning received work or complete or abort scan */
				_controller->continueAfterStall(this, work);

#if defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS)
				endWaitTimer(waitStartTime, work);
#endif /* defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS) */
			}
			_controller->releaseController();

			hasOutsideWork &= !isAbortedCycle();
		}

		/* no work except in outside copyspaces is more likely to occur at tail end of heap scan */
		if ((NULL == work) && hasOutsideWork) {
			/* evacuator worklists are empty */
			EvacuationRegion smallOutsideCopyspaceRegion = otherOutsideRegion(largestOutsideCopyspaceRegion);
			if (0 == _copyspace[smallOutsideCopyspaceRegion].getWorkSize()) {
				/* just take work from the nonempty outside copyspace */
				work = _freeList.next();
				work->base = (omrobjectptr_t)_copyspace[largestOutsideCopyspaceRegion].rebase(&work->length);
			} else {
				/* if other outside copyspace holds a substantial amount of work share it */
				if (MM_EvacuatorBase::max_copyspace_remainder <= _copyspace[largestOutsideCopyspaceRegion].getWorkSize()) {
					MM_EvacuatorWorkPacket *packet = _freeList.next();
					packet->base = (omrobjectptr_t)_copyspace[largestOutsideCopyspaceRegion].rebase(&packet->length);
					addWork(packet);
				}
				/* take work from the smallest outside copyspace */
				work = _freeList.next();
				work->base = (omrobjectptr_t)_copyspace[smallOutsideCopyspaceRegion].rebase(&work->length);
			}
		}
	}

	_workReleaseThreshold = _controller->calculateOptimalWorkPacketSize(getVolumeOfWork());

	/* if no work at this point heap scan completes (or aborts) */
	return work;
}

#if defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS)
uint64_t
MM_Evacuator::startWaitTimer()
{
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		omrtty_printf("%5lu %2llu %2llu:     stall; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%llx; vow:%llx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */
	return omrtime_hires_clock();
}

void
MM_Evacuator::endWaitTimer(uint64_t waitStartTime, MM_EvacuatorWorkPacket *work)
{
#if defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS)
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
	uint64_t waitEndTime = omrtime_hires_clock();
#endif /* defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS) */

#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		uint64_t waitMicros = omrtime_hires_delta(waitStartTime, waitEndTime, OMRPORT_TIME_DELTA_IN_MICROSECONDS);
		omrtty_printf("%5lu %2llu %2llu:    resume; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%llx; vow:%llx; work:0x%llx; length:%llx; micros:%llu\n", _controller->sampleEvacuatorFlags(),
				getVolumeOfWork(), (uintptr_t)work, ((NULL != work) ? work->length : 0), waitMicros);
	}
#endif /* defined(EVACUATOR_DEBUG) */

#if defined(J9MODRON_TGC_PARALLEL_STATISTICS)
	if (NULL == work) {
		_stats->addToCompleteStallTime(waitStartTime, waitEndTime);
	} else {
		_stats->addToWorkStallTime(waitStartTime, waitEndTime);
	}
#endif /* J9MODRON_TGC_PARALLEL_STATISTICS */
}
#endif /* defined(EVACUATOR_DEBUG) || defined(J9MODRON_TGC_PARALLEL_STATISTICS) */

void
MM_Evacuator::debugStack(const char *stackOp, bool treatAsWork)
{
#if defined(EVACUATOR_DEBUG)
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
	MM_EvacuatorScanspace *scanspace = (NULL != _scanStackFrame) ? _scanStackFrame : _stackBottom;
	if (_controller->_debugger.isDebugStack() || (treatAsWork && (_controller->_debugger.isDebugWork() || _controller->_debugger.isDebugBackout()))) {
		omrtty_printf("%5lu %2llu %2llu:%6s[%2d]; base:%llx; copy:%llx; end:%llx; free:%llx; limit:%llx; scan:%llx; unscanned:%llx; sow:%llx; tow:%llx\n",
				_controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, stackOp,
				scanspace - _stackBottom, (uintptr_t)scanspace->getBase(), (uintptr_t)scanspace->getCopyHead(), (uintptr_t)scanspace->getEnd(),
				scanspace->getWhiteSize(), (uintptr_t)scanspace->getCopyLimit(), (uintptr_t)scanspace->getScanHead(), scanspace->getUnscannedSize(),
				_copyspace[survivor].getWorkSize(), _copyspace[tenure].getWorkSize());
	}
#endif /* defined(EVACUATOR_DEBUG) */
}
