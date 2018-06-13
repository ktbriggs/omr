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

#include "thrtypes.h"

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
MM_Evacuator::newInstance(uintptr_t workerIndex, MM_EvacuatorController *controller, GC_ObjectModel *objectModel, uintptr_t maxStackDepth, uintptr_t maxInsideCopySize, uintptr_t maxInsideCopyDistance, uintptr_t copyspaceRefreshThreshold, MM_Forge *forge)
{
	MM_Evacuator *evacuator = (MM_Evacuator *)forge->allocate(sizeof(MM_Evacuator), OMR::GC::AllocationCategory::FIXED, OMR_GET_CALLSITE());
	if(NULL != evacuator) {
		new(evacuator) MM_Evacuator(workerIndex, controller, objectModel, maxStackDepth, maxInsideCopySize, maxInsideCopyDistance, copyspaceRefreshThreshold, forge);
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
	/* enable spinning for monitor-enter own evacuator worklist to put/take work */
	if (0 != omrthread_monitor_init_with_name(&_mutex, 0, "MM_Evacuator::_mutex")) {
		return false;
	}
	/* disable spinning for monitor-try-enter other evacuator worklist mutex to pull work */
	_mutex->flags &= ~J9THREAD_MONITOR_TRY_ENTER_SPIN;

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
MM_Evacuator::bindWorkerThread(MM_EnvironmentStandard *env, uintptr_t tenureMask, uint8_t *heapBounds[][2], uintptr_t copiedBytesReportingDelta)
{
	Debug_MM_true(0 == getVolumeOfWork());

	omrthread_monitor_enter(_mutex);

	/* bind evacuator and delegate to executing gc thread */
	_env = env;
	_env->setEvacuator(this);
	_tenureMask = tenureMask;
	_copiedBytesReportingDelta = copiedBytesReportingDelta;

	for (uintptr_t regionIndex = (uintptr_t)survivor; regionIndex <= ((uintptr_t)evacuate); regionIndex += 1) {
		_heapBounds[regionIndex][0] = heapBounds[regionIndex][0]; /* lower bound for heap region address range */
		_heapBounds[regionIndex][1] = heapBounds[regionIndex][1]; /* upper bound for heap region address range */
	}

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
	_copiedBytesDelta[survivor] = _copiedBytesDelta[tenure] = 0;
	_copyspaceOverflow[survivor] = _copyspaceOverflow[tenure] = 0;
	_scannedBytesDelta = 0;
	_completedScan = true;
	_abortedCycle = false;

	/* set up whitespaces for the cycle */
#if !defined(EVACUATOR_DEBUG)
	_whiteList[survivor].bind(NULL, _env, _workerIndex, _controller->getMemorySubspace(survivor), false);
	_whiteList[tenure].bind(NULL, _env, _workerIndex, _controller->getMemorySubspace(tenure), true);
#else
	_whiteList[survivor].bind(&_controller->_debugger, _env, _workerIndex, _controller->getMemorySubspace(survivor), false);
	_whiteList[tenure].bind(&_controller->_debugger, _env, _workerIndex, _controller->getMemorySubspace(tenure), true);
#endif /* defined(EVACUATOR_DEBUG) */

	/* load some empty workspaces into the freelist -- each evacuator retains forge memory between gc cycles to back this up */
	_freeList.reload();

	/* this evacuator has no work at this point so will set a low work release threshold */
	_workReleaseThreshold = _controller->calculateOptimalWorkspaceSize(getVolumeOfWork());

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

	Debug_MM_true((0 == getVolumeOfWork()) || isAbortedCycle());

	/* flush to freelist any abandoned work from the worklist  */
	_workList.flush(&_freeList);

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
MM_Evacuator::evacuateRootObject(MM_ForwardedHeader *forwardedHeader, bool breadthFirst)
{
	Debug_MM_true(NULL == _scanStackFrame);
	Debug_MM_true(0 == _stackBottom->getWorkSize());

	/* failure to evacuate will return original pointer to evacuate region */
	omrobjectptr_t forwardedAddress = forwardedHeader->getObject();

	if (!forwardedHeader->isForwardedPointer() && !isAbortedCycle()) {

		/* clear stack overflow condition */
		setFlushCondition(stack_overflow, false);
		setFlushCondition(depth_first, false);

		/* slot object must be evacuated -- determine before and after object size and which evacuation region should receive this object */
		uintptr_t slotObjectSizeAfterCopy = 0, slotObjectSizeBeforeCopy = 0, hotFieldAlignmentDescriptor = 0;
		_objectModel->calculateObjectDetailsForCopy(_env, forwardedHeader, &slotObjectSizeBeforeCopy, &slotObjectSizeAfterCopy, &hotFieldAlignmentDescriptor);
		EvacuationRegion const evacuationRegion = isNurseryAge(_objectModel->getPreservedAge(forwardedHeader)) ? survivor : tenure;

		/* can this object be spun out on the stack? */
		_scanStackFrame = (!breadthFirst && !shouldFlushOutside(evacuationRegion)) ? reserveRootCopyspace(evacuationRegion, slotObjectSizeAfterCopy) : NULL;
		if (NULL != _scanStackFrame) {
			Debug_MM_true(_scanStackFrame == _whiteStackFrame[evacuationRegion]);

			omrobjectptr_t const copyHead = (omrobjectptr_t)_scanStackFrame->getCopyHead();

			/* copy and forward object to bottom frame (we don't know the referring address so just say NULL) */
			forwardedAddress = copyForward(forwardedHeader, NULL, _scanStackFrame, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);

			/* if object was evacuated to bottom frame consider it pushed */
			if (forwardedAddress == copyHead) {
#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
				_scanStackFrame->activated();
				debugStack("R push");
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

				/* try to pull remaining whitespace up to next stack frame to force push for referents scanned in root object */
				MM_EvacuatorScanspace * const nextFrame = nextStackFrame(evacuationRegion, _scanStackFrame + 1);
				if (NULL != nextFrame) {
					nextFrame->pullWhitespace(_scanStackFrame);
					_whiteStackFrame[evacuationRegion] = nextFrame;
				}

			} else {

				/* object not copied by this evacuator */
				_scanStackFrame = NULL;
			}

		} else {

			/* copy object to outside copyspace -- this may redirect copy back into stack */
			forwardedAddress = copyOutside(evacuationRegion, forwardedHeader, NULL, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);
		}

		/* scan stack until empty */
		scan();

		Debug_MM_true(NULL == _scanStackFrame);
		Debug_MM_true(0 == _stackBottom->getWorkSize());

	} else if (!isAbortedCycle()) {

		Debug_MM_true(forwardedHeader->isForwardedPointer());

		forwardedAddress = forwardedHeader->getForwardedObject();
	}

	Debug_MM_true(NULL != forwardedAddress);

	return forwardedAddress;
}

bool
MM_Evacuator::evacuateRememberedObject(omrobjectptr_t objectptr)
{
	Debug_MM_true((NULL != objectptr) && isInTenure(objectptr));
	Debug_MM_true(_objectModel->getRememberedBits(objectptr) < (uintptr_t)0xc0);

	bool rememberObject = false;

	GC_ObjectScannerState objectScannerState;
	GC_ObjectScanner *objectScanner = _delegate.getObjectScanner(objectptr, &objectScannerState, GC_ObjectScanner::scanRoots);
	if (NULL != objectScanner) {
		GC_SlotObject *slotPtr;

		/* scan remembered object for referents in evacuate space */
		while (NULL != (slotPtr = objectScanner->getNextSlot())) {
			if (evacuateRootObject(slotPtr)) {
				rememberObject = true;
			}
		}
	}

	/* the remembered state of the object may also depends on its indirectly related objects */
	if (_objectModel->hasIndirectObjectReferents((CLI_THREAD_TYPE*)_env->getLanguageVMThread(), objectptr)) {
		if (_delegate.scanIndirectObjects(objectptr)) {
			rememberObject = true;
		}
	}

	return rememberObject;
}

void
MM_Evacuator::evacuateThreadSlot(volatile omrobjectptr_t *objectPtrIndirect)
{
	if (!_env->getExtensions()->isConcurrentScavengerEnabled()) {
		omrobjectptr_t objectPtr = *objectPtrIndirect;
		if (NULL != objectPtr) {
			/* auto-remember stack- and thread-referenced objects */
			if (isInEvacuate(objectPtr)) {

				if (!evacuateRootObject(objectPtrIndirect)) {
					Debug_MM_true(isInTenure(*objectPtrIndirect));
					Trc_MM_ParallelScavenger_copyAndForwardThreadSlot_deferRememberObject(_env->getLanguageVMThread(), *objectPtrIndirect);
					/* the object was tenured while it was referenced from the stack. Undo the forward, and process it in the rescan pass. */
					_controller->setEvacuatorFlag(MM_EvacuatorController::rescanThreadSlots, true);
					*objectPtrIndirect = objectPtr;
				}

			} else if (isInTenure(objectPtr)) {

				Debug_MM_true(_objectModel->getRememberedBits(objectPtr) < (uintptr_t)0xc0);
				if (_objectModel->atomicSwitchReferencedState(objectPtr, OMR_TENURED_STACK_OBJECT_RECENTLY_REFERENCED, OMR_TENURED_STACK_OBJECT_CURRENTLY_REFERENCED)) {
					Trc_MM_ParallelScavenger_copyAndForwardThreadSlot_renewingRememberedObject(_env->getLanguageVMThread(), objectPtr, OMR_TENURED_STACK_OBJECT_RECENTLY_REFERENCED);
				}
				Debug_MM_true(_objectModel->getRememberedBits(objectPtr) < (uintptr_t)0xc0);

			} else {

				Debug_MM_true(isInSurvivor(objectPtr));
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

		rememberObject(tenuredObjectPtr);
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

void
MM_Evacuator::setAbortedCycle()
{
	/* test and set controller abort condition for all evacuators to see */
	if (!_controller->setAborting()) {

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		/* report only if this evacuator is first to set the abort condition */
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2lu %2lu:     abort; flags:%lx", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, _controller->sampleEvacuatorFlags());
		if (NULL != _scanStackFrame) {
			omrtty_printf("; scanning:0x%lx", (uintptr_t)_scanStackFrame->getScanHead());
		}
		omrtty_printf("\n");
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

	}

	_abortedCycle = true;
}

bool
MM_Evacuator::isAbortedCycle()
{
	if (!_abortedCycle) {
		_abortedCycle = _controller->isAborting();
	}
	return _abortedCycle;
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

void
MM_Evacuator::flushWhitespace(MM_Evacuator::EvacuationRegion region)
{
	/* large whitespace fragements may be recycled back into the memory pool */
	_whiteList[region].flush();

	/* whitelist should be empty, top(0) should be NULL */
	Debug_MM_true(NULL == _whiteList[region].top(0));
}

void
MM_Evacuator::workThreadGarbageCollect(MM_EnvironmentStandard *env)
{
	Debug_MM_true(_env == env);
	Debug_MM_true(_env->getEvacuator() == this);
	Debug_MM_true(_controller->isBoundEvacuator(_workerIndex));
	Debug_MM_true(NULL == _whiteStackFrame[survivor]);
	Debug_MM_true(NULL == _whiteStackFrame[tenure]);

	/* initialize whitespace frame pointers for stack */
	for (EvacuationRegion region = survivor; region <= tenure; region = (EvacuationRegion)((intptr_t)region + 1)) {
		/* set empty whitespace into bottom stack frame offset by region */
		_stackBottom[region].setScanspace(_heapBounds[region][0], _heapBounds[region][0], 0);
		/* set pointer to whitespace frame -- this will follow the active scan stack frame */
		_whiteStackFrame[region] = &_stackBottom[region];
	}

	/* set flush state and reset remaining stack frames and scan/limit frame pointers */
	_flushState = 0;
	setFlushCondition(breadth_first, _controller->isEvacuatorFlagSet(MM_EvacuatorController::breadthFirstScan));
	for (_scanStackFrame = _stackBottom + evacuate; _scanStackFrame < _stackCeiling; _scanStackFrame += 1) {
		_scanStackFrame->clear();
	}
	_stackLimit = _stackCeiling;
	_scanStackFrame = NULL;

	/* scan roots and remembered set and objects depending from these */
	scanRemembered();
	scanRoots();
	scanHeap();

	/* trick to obviate a write barrier -- objects tenured this cycle are always remembered */
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

	/* clear whitespace from outside copyspaces and whitespace frames */
	for (intptr_t space = (intptr_t)survivor; space <= (intptr_t)tenure; space += 1) {

		/* clear whitespace from the last active whitespace frame */
		Debug_MM_true(NULL != _whiteStackFrame[space]);
		_whiteList[space].add(_whiteStackFrame[space]->trim());
		_whiteStackFrame[space] = NULL;

		/* release unused whitespace from outside copyspaces */
		Debug_MM_true(isAbortedCycle() || (0 == _copyspace[space].getCopySize()));
		_whiteList[space].add(_copyspace[space].trim());
		_copyspace[space].reset();
	}

#if defined(EVACUATOR_DEBUG)
	_whiteList[survivor].verify();
	_whiteList[tenure].verify();
#endif /* defined(EVACUATOR_DEBUG) */

	/* reset large copyspace (it is void of work and whitespace at this point) */
	Debug_MM_true(0 == _largeCopyspace.getCopySize());
	Debug_MM_true(0 == _largeCopyspace.getWhiteSize());
	_largeCopyspace.reset();

	/* large survivor whitespace fragments on the whitelist may be recycled back into the memory pool */
	flushWhitespace(survivor);

	/* tenure whitespace is retained between completed nursery collections but must be flushed for backout */
	Debug_MM_true(0 == _env->_scavengerRememberedSet.count);
	if (!isAbortedCycle()) {
		/* prune remembered set to complete cycle */
		_controller->pruneRememberedSet(_env);
		Debug_MM_true(0 == getVolumeOfWork());
	} else {
		/* flush tenure whitelist before backout */
		flushWhitespace(tenure);
		/* set backout flag to abort cycle */
		_controller->setBackOutFlag(_env, backOutFlagRaised);
		_controller->completeBackOut(_env);
	}
}

void
MM_Evacuator::scanRoots()
{
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2lu %2lu:     roots; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%lx; vow:%lx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
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
		omrtty_printf("%5lu %2lu %2lu:remembered; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%lx; vow:%lx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */

	/* flush locally buffered remembered objects to aggregated list */
	_env->flushRememberedSet();

	/* scan aggregated remembered objects concurrently with other evacuators */
	_controller->scavengeRememberedSet(_env);
}

bool
MM_Evacuator::scanClearable()
{
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2lu %2lu: clearable; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%lx; vow:%lx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
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
	Debug_MM_true(NULL == _scanStackFrame);
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2lu %2lu:      heap; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%lx; vow:%lx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */

	/* mark start of scan cycle -- this will be set after this evacuator receives controller's end of scan or aborts */
	_completedScan = false;

	/* try to find some work from this or other evacuator's worklist to pull into the scan stack */
	while (getWork()) {

		/* scan workspace, pull and scan work from outside copyspaces until both are empty */
		scan();
	}

	Debug_MM_true(_completedScan);
}

void
MM_Evacuator::scanComplete()
{
#if defined(EVACUATOR_DEBUG)
	Debug_MM_true(NULL == _scanStackFrame);
	for (MM_EvacuatorScanspace *stackFrame = _stackBottom; stackFrame < _stackCeiling; stackFrame += 1) {
		Debug_MM_true(0 == stackFrame->getWorkSize());
		Debug_MM_true(0 == stackFrame->getWhiteSize() || (stackFrame == _whiteStackFrame[getEvacuationRegion(stackFrame->getBase())]));
	}
	Debug_MM_true(isAbortedCycle() || (0 == _copyspace[survivor].getCopySize()));
	Debug_MM_true(isAbortedCycle() || (0 == _copyspace[tenure].getCopySize()));
#endif /* defined(EVACUATOR_DEBUG) */

	/* reset stack  */
	_stackLimit = _stackCeiling;

	/* all done heap scan */
	Debug_MM_true(!_completedScan);
	_completedScan = true;
}

void
MM_Evacuator::addWork(MM_EvacuatorWorkspace *work)
{
	omrthread_monitor_enter(_mutex);
	work = _workList.add(work);
	if (NULL != work) {
		_freeList.add(work);
	}
	omrthread_monitor_exit(_mutex);

	_controller->notifyOfWork(this);

	_workReleaseThreshold = _controller->calculateOptimalWorkspaceSize(getVolumeOfWork());
}

MM_EvacuatorWorkspace *
MM_Evacuator::findWork()
{
	MM_EvacuatorWorkspace *work = NULL;

	if (!isAbortedCycle()) {
		/* select prospective donor as the evacuator with greatest volume of work */
		MM_Evacuator *donor = this;
		for (MM_Evacuator *evacuator = _controller->getNextEvacuator(donor); this != evacuator; evacuator = _controller->getNextEvacuator(evacuator)) {
			if (_controller->isBoundEvacuator(evacuator->getWorkerIndex()) && (evacuator->getVolumeOfWork() > donor->getVolumeOfWork())) {
				donor = evacuator;
			}
		}

		if (0 < donor->getVolumeOfWork()) {

			/* try selected donor, if donor has no work poll for the next evacuator that can donate work or bail if poll wraps to max */
			MM_Evacuator * const max = donor;
			do {
				Debug_MM_true(this != donor);

				/* skip self and do not wait for donor if it is involved with its worklist */
				if (0 == omrthread_monitor_try_enter(donor->_mutex)) {

					if (0 < donor->getVolumeOfWork()) {

						/* take first available workspace as current work */
						work = donor->_workList.next();
						uintptr_t volume = _workList.volume(work);

						/* level this evacuator's volume of work up with donor */
						const MM_EvacuatorWorkspace *next = donor->_workList.peek();
						while (NULL != next) {

							/* receiver must leave at least as much work in donor worklist as received */
							uintptr_t nextVolume = _workList.volume(next);
							if ((volume + nextVolume) < (donor->getVolumeOfWork() - nextVolume)) {

								/* this evacuator holds the controller mutex so does not take its own worklist mutex here */
								MM_EvacuatorWorkspace *free = _workList.add(donor->_workList.next());
								if (NULL != free) {
									/* donated workspace merged into tail workspace and can be recycled */
									_freeList.add(free);
								}
								volume += nextVolume;
								next = donor->_workList.peek();

							} else {

								/* done leveling up */
								break;
							}
						}

						/* receiver has one workspace (work) and less volume on worklist than donor unless both worklists are empty */
						Debug_MM_true(getVolumeOfWork() <= donor->getVolumeOfWork());
					}

					omrthread_monitor_exit(donor->_mutex);

					/* stop and return if any work received from donor */
					if (NULL != work) {

						/* tell the world if donor has any distributable work left */
						_controller->notifyOfWork(donor);

#if defined(EVACUATOR_DEBUG)
						if (_controller->_debugger.isDebugWork()) {
							OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
							omrtty_printf("%5lu %2lu %2lu: pull work; base:%lx; length:%lx; vow:%lx; donor:%lx; donor-vow:%lx; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch,
									_workerIndex, (uintptr_t)work->base, work->length, getVolumeOfWork(), donor->_workerIndex, donor->getVolumeOfWork());
							_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
							_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
							omrtty_printf("\n");
						}
#endif /* defined(EVACUATOR_DEBUG) */

						return work;
					}
				}

				/* no work received so continue donor enumeration, skipping evacuators with no work */
				do {

					/* bail with no work if donor wraps around to max */
					donor = _controller->getNextEvacuator(donor);

				} while ((max != donor) && (0 == donor->getVolumeOfWork()));

			} while (max != donor);
		}
	}

	return NULL;
}

bool
MM_Evacuator::getWork()
{
	Debug_MM_true(NULL == _scanStackFrame);

	MM_EvacuatorWorkspace *work = NULL;
	if (!isAbortedCycle()) {

		/* outside copyspaces must be cleared before engaging the controller to find other work */
		uintptr_t copy[2] = { _copyspace[survivor].getCopySize(), _copyspace[tenure].getCopySize() };
		EvacuationRegion take = (copy[survivor] < copy[tenure]) ? survivor : tenure;
		if (0 == copy[take]) {
			take = otherOutsideRegion(take);
		}

		/* take shortest nonempty outside copyspace and return, if both empty try worklist */
		if (0 < copy[take]) {

			/* pull work from outside copyspace */
			pull(&_copyspace[take]);

			/* return without notifying other threads */
			return true;

		} else {

			/* try to take work from worklist */
			omrthread_monitor_enter(_mutex);
			work = _workList.next();
			omrthread_monitor_exit(_mutex);
		}
	}

	if (NULL == work) {

		/* worklist is empty, or aborting; try to pull work from other evacuators */
		if (0 != (_scannedBytesDelta | _copiedBytesDelta[survivor] | _copiedBytesDelta[tenure])) {
			/* flush scanned/copied byte counters if any are >0 before stalling */
			_controller->reportProgress(this, _copiedBytesDelta, &_scannedBytesDelta);
		}
		/* flush local buffers before stalling */
		flushForWaitState();

#if defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		uint64_t waitStartTime = startWaitTimer();
#endif /* defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

		_controller->acquireController();
		work = findWork();
		if (NULL == work) {

			/* controller will release evacuator from stall when work is received or all other evacuators have stalled to complete or abort scan */
			while (_controller->isWaitingToCompleteStall(this, work)) {
				/* wait for work or until controller signals all evacuators to complete or abort scan */
				_controller->waitForWork();
				work = findWork();
			}

			/* continue scanning received work or complete or abort scan */
			_controller->continueAfterStall(this, work);

		}
		_controller->releaseController();

#if defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		endWaitTimer(waitStartTime, work);
#endif /* defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
	}

	if (NULL != work) {
#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		uintptr_t headerSize = (work->offset == 1) ? _objectModel->getHeaderSize(work->base) : 0;
		_stats->countWorkPacketSize(headerSize + _workList.volume(work), _controller->_maximumWorkspaceSize);
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

		/* reset workspace release threshold */
		_workReleaseThreshold = _controller->calculateOptimalWorkspaceSize(getVolumeOfWork());

		/* pull work from workspace */
		pull(work);

		/* in case of other stalled evacuators */
		_controller->notifyOfWork(this);

		/* return workspace to freelist */
		_freeList.add(work);

		return true;
	}

	/* if no work at this point heap scan completes (or aborts) */
	return false;
}

MM_EvacuatorScanspace *
MM_Evacuator::clear()
{
	Debug_MM_true(NULL == _scanStackFrame);
	Debug_MM_true(0 == _stackBottom->getWorkSize());
	Debug_MM_true(0 == _stackBottom[evacuate].getWorkSize());

	/* clear stack overflow condition */
	setFlushCondition(stack_overflow, false);
	setFlushCondition(depth_first, false);

	/* lower work release threshold while thrashing on residue in outside copyspaces and not producing distributable work */
	if (testFlushCondition(stall)) {
		/* cut work release threshold in half -- this will be restored to free-running operating value when stall is cleared */
		_workReleaseThreshold = OMR_MAX(_copyspaceRefreshThreshold, (_workReleaseThreshold >> 1));
	}

	/* select scan frame as bottom frame unless breadth first, else reserve bottom two frames and select next */
	const bool isBreadthFirst = testFlushCondition(breadth_first);
	MM_EvacuatorScanspace * const scan = isBreadthFirst ? (_stackBottom + evacuate) : _stackBottom;

	/* unconditionally clear whitespace from scan frame -- find an empty frame to receive it */
	if (0 < scan->getWhiteSize()) {

		const EvacuationRegion scanRegion = getEvacuationRegion(scan->getBase());
		Debug_MM_true(scan == _whiteStackFrame[scanRegion]);

		/* find next empty frame that can receive whitespace from the scan frame */
		_whiteStackFrame[scanRegion] = nextStackFrame(scanRegion, (isBreadthFirst ? _stackBottom : (scan + 1)));
		Debug_MM_true((NULL != _whiteStackFrame[scanRegion]) && _whiteStackFrame[scanRegion]->isClear());

		/* pull whitespace into empty frame from scan frame */
		_whiteStackFrame[scanRegion]->pullWhitespace(scan);
	}

	/* scanspace is clear and ready to pull work */
	Debug_MM_true(scan->isClear());

	return scan;
}

void
MM_Evacuator::pull(MM_EvacuatorCopyspace *copyspace)
{
	Debug_MM_true(0 < copyspace->getCopySize());

	/* select stack frame to pull work into */
	_scanStackFrame = clear();
	/* set scanspace base=scan=copyspace.base, copy=limit=end=base+copyspace.end and rebase copyspace */
	_scanStackFrame->pullWork(copyspace);
	/* object scanners will be instantiated and _scannedBytes updated as objects in scanspace are scanned */
	_splitArrayBytesToScan = 0;

	#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	_scanStackFrame->activated();
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2lu %2lu: push work; base:%lx; length:%lx; vow:%lx; sow:%lx; tow:%lx ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex,
				(uintptr_t)_scanStackFrame->getBase(), _scanStackFrame->getWorkSize(), getVolumeOfWork(), _copyspace[survivor].getCopySize(), _copyspace[tenure].getCopySize());
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("\n");
	}
	debugStack("W pull");
#endif /* defined(EVACUATOR_DEBUG) */
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
}

void
MM_Evacuator::pull(MM_EvacuatorWorkspace *work)
{
	Debug_MM_true(NULL != work);
	Debug_MM_true(0 < work->length);
	Debug_MM_true(NULL != work->base);

	/* select stack frame to pull work into */
	_scanStackFrame = clear();

	/* load workspace into scan stack frame */
	if (isSplitArrayWorkspace(work)) {

		GC_IndexableObjectScanner *scanner = (GC_IndexableObjectScanner *)_scanStackFrame->getObjectScannerState();
		/* the object scanner must be set in the scanspace at this point for split arrays -- work offset/length are array indices in this case */
		_delegate.getSplitPointerArrayObjectScanner(work->base, scanner, work->offset - 1, work->length, GC_ObjectScanner::scanHeap);
		/* the workspace contains a split array segment -- set scanspace base = array, scan = copy = limit = end = base + object-size */
		_scanStackFrame->setSplitArrayScanspace((uint8_t *)work->base, ((uint8_t *)work->base + _objectModel->getConsumedSizeInBytesWithHeader(work->base)), scanner);
		/* split array segments are scanned only on the bottom frame of the stack -- save the length of segment for updating _scannedBytes when this segment has been scanned */
		_splitArrayBytesToScan = sizeof(fomrobject_t) * work->length;

	} else {

		/* the workspace contains a series of contiguous objects -- set scanspace base = scan, copy = limit = end = scan + length */
		_scanStackFrame->setScanspace((uint8_t *)work->base, (uint8_t *)work->base + work->length, work->length);
		/* object scanners will be instantiated and _scannedBytes updated as objects in scanspace are scanned */
		_splitArrayBytesToScan = 0;
	}

	#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	_scanStackFrame->activated();
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		omrtty_printf("%5lu %2lu %2lu: push work; base:%lx; length:%lx; vow:%lx; sow:%lx; tow:%lx ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex,
				(uintptr_t)work->base, work->length, getVolumeOfWork(), _copyspace[survivor].getCopySize(), _copyspace[tenure].getCopySize());
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("\n");
	}
	debugStack("W pull");
#endif /* defined(EVACUATOR_DEBUG) */
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
}

void
MM_Evacuator::scan()
{
	/* scan stack until empty */
	while (NULL != _scanStackFrame) {
		Debug_MM_true((_stackLimit == _stackCeiling) || (_stackLimit == _stackBottom));
		Debug_MM_true((_scanStackFrame < _stackLimit) || (_stackLimit == _stackBottom));
		Debug_MM_true(_scanStackFrame >= _stackBottom);

		/* copy inside current frame until push or pop */
		copy();
	}

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	for (MM_EvacuatorScanspace *frame = _stackBottom; frame < _stackCeiling; ++frame) {
		Debug_MM_true(0 == frame->getWorkSize());
	}
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
}

void
MM_Evacuator::push(MM_EvacuatorScanspace * const nextStackFrame)
{
	if (_scanStackFrame != nextStackFrame) {

		/* push to next frame */
		_scanStackFrame = nextStackFrame;

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
		_scanStackFrame->activated();
		debugStack("push");
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
	}
}

void
MM_Evacuator::pop()
{
	debugStack("pop");
	Debug_MM_true((_stackBottom <= _scanStackFrame) && (_scanStackFrame < _stackCeiling));
	Debug_MM_true((0 == _scanStackFrame->getWhiteSize()) || (_scanStackFrame == _whiteStackFrame[getEvacuationRegion(_scanStackFrame->getBase())]));
	Debug_MM_true((0 == _scanStackFrame->getWorkSize()) || isAbortedCycle());

	/* clear scanned work and work-related flags from scanspace */
	_scanStackFrame->rebase();

	/* pop stack frame leaving trailing whitespace where it is (in _whiteStackFrame[getEvacuationRegion(_scanStackFrame->getBase())]) */
	if (_stackBottom < _scanStackFrame) {

		/* pop to previous frame, will continue with whitespace in popped frame if next pushed object does not cross region boundary */
		_scanStackFrame -= 1;

		/* reset stack overflow state after setting or clearing depth first state */
		if (_scanStackFrame == _stackBottom) {

			/* force depth first scanning if stack overflowed and flushed as it unwound */
			setFlushCondition(depth_first, testFlushCondition(stack_overflow));

			/* clear stack overflow condition for next slot scanned in bottom frame */
			setFlushCondition(stack_overflow, false);
		}

	} else {

		/* pop to empty stack */
		_scanStackFrame = NULL;
	}

}

void
MM_Evacuator::copy()
{
	const uintptr_t sizeLimit = _maxInsideCopySize;
	MM_EvacuatorScanspace * const stackFrame = _scanStackFrame;
	const bool isTenureFrame = (tenure == getEvacuationRegion(stackFrame->getBase()));
	uint8_t * const frameLimit = stackFrame->getBase() + (testFlushCondition(depth_first) ? 0: _maxInsideCopyDistance);

	/* get the active object scanner without advancing slot */
	GC_ObjectScanner *objectScanner = nextObjectScanner(stackFrame, false);

	/* copy small stack region objects inside frame, large and other region objects outside, until push() or frame completed */
	while (NULL != objectScanner) {
		Debug_MM_true((objectScanner->getParentObject() == (omrobjectptr_t)stackFrame->getScanHead()) || (stackFrame->isSplitArraySegment() && (objectScanner->getParentObject() == (omrobjectptr_t)stackFrame->getBase())));

		/* loop through reference slots in current object at scan head */
		GC_SlotObject *slotObject = objectScanner->getNextSlot();
		while (NULL != slotObject) {

			const omrobjectptr_t object = slotObject->readReferenceFromSlot();
			omrobjectptr_t forwardedAddress = object;
			if (isInEvacuate(object)) {

				/* copy and forward the slot object */
				MM_ForwardedHeader forwardedHeader(object);
				if (!forwardedHeader.isForwardedPointer()) {

					/* slot object must be evacuated -- determine receiving region and before and after object size */
					uintptr_t slotObjectSizeBeforeCopy = 0, slotObjectSizeAfterCopy = 0, hotFieldAlignmentDescriptor = 0;
					_objectModel->calculateObjectDetailsForCopy(_env, &forwardedHeader, &slotObjectSizeBeforeCopy, &slotObjectSizeAfterCopy, &hotFieldAlignmentDescriptor);
					const EvacuationRegion evacuationRegion = isNurseryAge(_objectModel->getPreservedAge(&forwardedHeader)) ? survivor : tenure;
					const uintptr_t whiteStackFrameRemainder = _whiteStackFrame[evacuationRegion]->getWhiteSize();

					/* copy flush overflow and large objects outside,  copy small objects inside the stack ... */
					if (!shouldFlushOutside(evacuationRegion) && (sizeLimit >= slotObjectSizeAfterCopy) &&
							/* ... if sufficient whitespace remaining ... */
							((whiteStackFrameRemainder >= slotObjectSizeAfterCopy) ||
									/* ... or can be refreshed in a superior (next) stack frame */
									reserveInsideCopyspace(evacuationRegion, whiteStackFrameRemainder, slotObjectSizeAfterCopy))
					) {
						/* space for copy reserved in white stack frame -- this may or may not be the current scan frame */
						MM_EvacuatorScanspace * const whiteFrame = _whiteStackFrame[evacuationRegion];
						uint8_t * const copyHead = whiteFrame->getCopyHead();

						/* copy inside stack frame holding evacuation region whitespace */
						forwardedAddress = copyForward(&forwardedHeader, slotObject->readAddressFromSlot(), whiteFrame, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);

						/* if object was copied into scan frame past copy limit or into a superior frame a push may be necessary */
						if (((uint8_t *)forwardedAddress == copyHead) && (((whiteFrame == stackFrame) && (frameLimit < copyHead)) || (whiteFrame > stackFrame))) {

							/* object copied into scan frame -- find nearest superior frame that can receive whitespace for evacuation region */
							MM_EvacuatorScanspace * const nextFrame =  nextStackFrame(evacuationRegion, stackFrame + 1);

							/* if no next frame just obviate push and scan it in current frame */
							if (NULL != nextFrame) {

								/* pull copy and remaining whitespace up into next frame and set next frame as white frame for evacuation region */
								if (nextFrame != whiteFrame) {
									nextFrame->pullTail(whiteFrame, copyHead);
									_whiteStackFrame[evacuationRegion] = nextFrame;
								}

								/* push to next frame */
								push(nextFrame);
							}
						}

					} else {

						/* copy to outside copyspace if not possible to copy inside stack -- this may push work into the stack */
						forwardedAddress = copyOutside(evacuationRegion, &forwardedHeader, slotObject->readAddressFromSlot(), slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);
					}

				} else {

					/* just get the forwarding address */
					forwardedAddress = forwardedHeader.getForwardedObject();
				}

				/* update the referring slot with the forwarding address */
				slotObject->writeReferenceToSlot(forwardedAddress);
			}

			/* if scanning a tenured object update its remembered state */
			if (isTenureFrame && (isInSurvivor(forwardedAddress) || isInEvacuate(forwardedAddress))) {
				Debug_MM_true(!isInEvacuate(forwardedAddress) || isAbortedCycle());
				stackFrame->updateRememberedState(true);
			}

			/* check for push() */
			if (_scanStackFrame != stackFrame) {
				/* return to continue in new frame */
				return;
			}

			/* continue with next scanned slot object */
			slotObject = objectScanner->getNextSlot();
		}

		/* scan next non-leaf object in frame, if any */
		objectScanner = nextObjectScanner(stackFrame);
	}

	Debug_MM_true(stackFrame->getScanHead() == stackFrame->getCopyHead());

	/* pop scan stack */
	pop();
}

bool
MM_Evacuator::shouldFlushOutside(const EvacuationRegion evacuationRegion)
{
	Debug_MM_true(_stackLimit <= _stackCeiling);

	/* check for stalled evacuators */
	setFlushCondition(stall, _controller->shouldDistributeWork());

	/* toggle stack limit between bottom+1 and ceiling to force/end flushing all copy to outside copyspaces */
	if (isAnyFlushConditionSet(evacuationRegion)) {

		if (_stackLimit == _stackCeiling) {

			/* reduce work release threshold to lower bound of operational range */
			_workReleaseThreshold = _controller->_minimumWorkspaceSize;
			/* lower stack limit to mark start of flushing lower work release threshold if thrashing on residual outside copy to produce distributable work while flushing */
			_stackLimit = _stackBottom;
		}

		return true;

	} else if (_stackLimit < _stackCeiling) {

		/* restore work release threshold to lower bound of operational range */
		_workReleaseThreshold = OMR_MAX(_workReleaseThreshold, _controller->_minimumWorkspaceSize);
		/* raise stack limit to mark end of flushing */
		_stackLimit = _stackCeiling;
	}

	return false;
}

MM_EvacuatorScanspace *
MM_Evacuator::nextStackFrame(const EvacuationRegion evacuationRegion, MM_EvacuatorScanspace *frame)
{
	/* find the nearest frame not inferior to frame that holds or can receive whitespace for evacuation region */
	if (frame == _whiteStackFrame[otherOutsideRegion(evacuationRegion)]) {
		/* frame holds whitespace for other evacuation region and must be skipped */
		frame += 1;
	}

	/* frame is empty or holds whitespace for evacuation region or stack is blown */
	if (frame >= _stackCeiling) {
		setFlushCondition(stack_overflow, true);
		frame = NULL;
	}

	Debug_MM_true((NULL == frame) || (frame == _whiteStackFrame[evacuationRegion]) || (frame->getScanHead() == frame->getEnd()));

	return frame;
}

GC_ObjectScanner *
MM_Evacuator::nextObjectScanner(MM_EvacuatorScanspace * const scanspace, bool finalizeObjectScan)
{
	uintptr_t scannedBytes = 0;

	/* get current object scanner, if any */
	GC_ObjectScanner *objectScanner = scanspace->getActiveObjectScanner();

	/* object scanner is finalized after it returns its last slot */
	if (finalizeObjectScan && (NULL != objectScanner)) {

		/* advance scan head past parent object or split array segment */
		if (scanspace->isSplitArraySegment()) {

			scannedBytes = _splitArrayBytesToScan;
			if (objectScanner->isHeadObjectScanner()) {
				/* indexable object header and trailing remainder scanned bytes are counted with first array segment scanned */
				scannedBytes += (_objectModel->getConsumedSizeInBytesWithHeader(objectScanner->getParentObject()) - ((GC_IndexableObjectScanner *)objectScanner)->getDataSizeInBytes());
			}
			_splitArrayBytesToScan = 0;

		} else {

			scannedBytes = _objectModel->getConsumedSizeInBytesWithHeader(objectScanner->getParentObject());
		}

		/* update remembered state for parent object */
		if (isInTenure(scanspace->getScanHead()) && scanspace->getRememberedState()) {
			rememberObject(objectScanner->getParentObject());
		}
		scanspace->clearRememberedState();

		/* move scan head over scanned object (if not split array where scan is preset to end) and drop its object scanner */
		objectScanner = scanspace->advanceScanHead(scannedBytes);

		/* active object scanner was NULLed in advanceScanHead() */
		Debug_MM_true((NULL == objectScanner) && (objectScanner == scanspace->getActiveObjectScanner()));
	}

	/* advance scan head over leaf objects and objects with no scanner to set up next object scanner */
	while ((NULL == objectScanner) && (scanspace->getScanHead() < scanspace->getCopyHead())) {

		omrobjectptr_t objectPtr = (omrobjectptr_t)scanspace->getScanHead();
		objectScanner = _delegate.getObjectScanner(objectPtr, scanspace->getObjectScannerState(), GC_ObjectScanner::scanHeap);
		if ((NULL == objectScanner) || objectScanner->isLeafObject()) {

			/* nothing to scan for object at scan head, drop object scanner and advance scan head to next object in scanspace */
			uintptr_t bytes = _objectModel->getConsumedSizeInBytesWithHeader(objectPtr);
			objectScanner = scanspace->advanceScanHead(bytes);
			scannedBytes += bytes;

			/* active object scanner was NULLed in advanceScanHead() */
			Debug_MM_true((NULL == objectScanner) && (objectScanner == scanspace->getActiveObjectScanner()));
		}
	}

	/* update evacuator progress for epoch reporting */
	if (0 < scannedBytes) {
		_scannedBytesDelta += scannedBytes;
		if ((_scannedBytesDelta >= _copiedBytesReportingDelta) || ((_copiedBytesDelta[survivor] + _copiedBytesDelta[tenure]) >= _copiedBytesReportingDelta)) {
			_controller->reportProgress(this, _copiedBytesDelta, &_scannedBytesDelta);
		}
	}

	return objectScanner;
}

bool
MM_Evacuator::reserveInsideCopyspace(const EvacuationRegion evacuationRegion, const uintptr_t whiteSize, const uintptr_t slotObjectSizeAfterCopy)
{
	Debug_MM_true(evacuate > evacuationRegion);
	Debug_MM_true((0 == whiteSize) || (whiteSize == _whiteStackFrame[evacuationRegion]->getWhiteSize()));
	Debug_MM_true(slotObjectSizeAfterCopy > whiteSize);

	/* refresh stack whitespace only if remainder is discardable */
	if (MM_EvacuatorBase::max_scanspace_remainder >= whiteSize) {

		/* find next frame that can receive whitespace for evacuation region */
		MM_EvacuatorScanspace *nextFrame = nextStackFrame(evacuationRegion, _scanStackFrame + 1);
		if (NULL != nextFrame) {

			/* trim white frame remainder to whitelist (it will be discarded) */
			_whiteList[evacuationRegion].add(_whiteStackFrame[evacuationRegion]->trim());

			MM_EvacuatorWhitespace *whitespace = NULL;

			if (_whiteList[evacuationRegion].top() > OMR_MAX(slotObjectSizeAfterCopy, _copyspaceRefreshThreshold)) {

				/* trim remainder whitespace to whitelist and try to get whitespace from top of the whitelist */
				whitespace = _whiteList[evacuationRegion].top(slotObjectSizeAfterCopy);
				Debug_MM_true(NULL != whitespace);

			} else  {

				/* try to allocate whitespace from the heap */
				whitespace = _controller->getWhitespace(this, evacuationRegion, slotObjectSizeAfterCopy);
				if (NULL == whitespace) {
					/* failover to outside copy */
					return false;
				}
			}

			/* set whitespace into next stack frame */
			nextFrame->setScanspace((uint8_t *)whitespace, (uint8_t *)whitespace, whitespace->length(), whitespace->isLOA());

			/* receiving frame is holding enough stack whitespace to copy object to evacuation region */
			Debug_MM_true(evacuationRegion == getEvacuationRegion(nextFrame->getBase()));
			Debug_MM_true(slotObjectSizeAfterCopy <= nextFrame->getWhiteSize());

			/* stack whitespace for evacuation region is installed in root stack frame */
			_whiteStackFrame[evacuationRegion] = nextFrame;

			return true;
		}
	}

	/* current whitespace not discardable or no stack frame available to receive new whitespace */
	return false;
}

MM_EvacuatorScanspace *
MM_Evacuator::reserveRootCopyspace(const EvacuationRegion evacuationRegion, uintptr_t slotObjectSizeAfterCopy)
{
	MM_EvacuatorScanspace *rootFrame = testFlushCondition(breadth_first) ? _whiteStackFrame[evacuationRegion] : nextStackFrame(evacuationRegion, _stackBottom);

	if (NULL != rootFrame) {

		/* ensure capacity in evacuation region stack whitespace */
		MM_EvacuatorScanspace *whiteFrame = _whiteStackFrame[evacuationRegion];
		uintptr_t whiteSize = (NULL != whiteFrame) ? whiteFrame->getWhiteSize() : 0;
		if (slotObjectSizeAfterCopy <= whiteSize) {

			/* fits in remaining stack whitespace for evacuation region so pull whitespace into root stack frame */
			if (rootFrame != whiteFrame) {
				rootFrame->pullWhitespace(whiteFrame);
			}

		} else if (MM_EvacuatorBase::max_scanspace_remainder >= whiteSize) {

			/* trim white frame remainder to whitelist (it will be discarded) */
			_whiteList[evacuationRegion].add(whiteFrame->trim());

			/* does not fit in but stack whitespace for evacuation region can be refreshed */
			MM_EvacuatorWhitespace *whitespace = NULL;

			/* get fresh whitespace */
			if (_whiteList[evacuationRegion].top() >= OMR_MAX(slotObjectSizeAfterCopy, _copyspaceRefreshThreshold)) {
				/* try to refresh from top of the whitelist */
				whitespace = _whiteList[evacuationRegion].top(slotObjectSizeAfterCopy);
			} else {
				/* try to allocate whitespace from the heap */
				whitespace = _controller->getWhitespace(this, evacuationRegion, slotObjectSizeAfterCopy);
			}

			/* set whitespace into root stack frame, or failover to outside copy */
			if (NULL != whitespace) {
				/* set up to evacuate root object to next stack frame */
				rootFrame->setScanspace((uint8_t*) (whitespace), (uint8_t*) (whitespace), whitespace->length(), whitespace->isLOA());
			} else {
				/* failover to outside copy */
				return NULL;
			}

		} else {

			/* force outside copy */
			return NULL;
		}

		/* receiving frame is holding enough stack whitespace to copy object to evacuation region */
		Debug_MM_true(evacuationRegion == getEvacuationRegion(rootFrame->getBase()));
		Debug_MM_true(slotObjectSizeAfterCopy <= rootFrame->getWhiteSize());

		/* stack whitespace for evacuation region is installed in root stack frame */
		_whiteStackFrame[evacuationRegion] = rootFrame;
	}

	return rootFrame;
}

omrobjectptr_t
MM_Evacuator::copyOutside(EvacuationRegion evacuationRegion, MM_ForwardedHeader *forwardedHeader, fomrobject_t *referringSlotAddress, const uintptr_t slotObjectSizeBeforeCopy, const uintptr_t slotObjectSizeAfterCopy)
{
	Debug_MM_true(!forwardedHeader->isForwardedPointer());
	Debug_MM_true(isInEvacuate(forwardedHeader->getObject()));

	/* failure to evacuate must return original address in evacuate space */
	omrobjectptr_t forwardingAddress = forwardedHeader->getObject();

	/* use the preferred region copyspace if it can contain the object */
	MM_EvacuatorCopyspace *effectiveCopyspace = &_copyspace[evacuationRegion];
	if (slotObjectSizeAfterCopy > effectiveCopyspace->getWhiteSize()) {
		/* otherwise reserve copy space -- this may be obtained from inside whitespace if outside copyspace can't be refreshed */
		const bool useLargeCopyspace = isSplitablePointerArray(forwardedHeader, slotObjectSizeAfterCopy);
		effectiveCopyspace = reserveOutsideCopyspace(&evacuationRegion, slotObjectSizeAfterCopy, useLargeCopyspace);
	}

	if (NULL != effectiveCopyspace) {
		Debug_MM_true(slotObjectSizeAfterCopy <= effectiveCopyspace->getWhiteSize());

		/* copy slot object to effective outside copyspace */
		omrobjectptr_t copyHead = (omrobjectptr_t)effectiveCopyspace->getCopyHead();
		forwardingAddress = copyForward(forwardedHeader, referringSlotAddress, effectiveCopyspace, slotObjectSizeBeforeCopy, slotObjectSizeAfterCopy);
		if (copyHead == forwardingAddress) {

			/* object copied into effective copyspace -- check for sharable work to distribute */
			if (effectiveCopyspace == &_largeCopyspace) {

				/* add large object to the worklist to free large copyspace for reuse */
				if (isSplitablePointerArray(forwardedHeader, slotObjectSizeAfterCopy)) {

					/* record 1-based array offsets to mark split pointer array workspaces */
					splitPointerArrayWork(copyHead);

					/* advance base of copyspace to copy head as array scanning work has been distributed to worklist */
					_largeCopyspace.erase();

				} else {

					/* set up a workspace containing a single large scalar or primitive/non-splitable pointer array object */
					MM_EvacuatorWorkspace *work = _freeList.next();
					work->base = (omrobjectptr_t)_largeCopyspace.rebase(&work->length);
					Debug_MM_true(slotObjectSizeAfterCopy == work->length);

					/* add it to the worklist */
					addWork(work);
				}

				/* clear the large object copyspace for next use */
				_whiteList[evacuationRegion].add(_largeCopyspace.trim());

			} else if (effectiveCopyspace != static_cast<MM_EvacuatorCopyspace *>(_whiteStackFrame[evacuationRegion])) {
				Debug_MM_true(MM_EvacuatorBase::min_reusable_whitespace <= _workReleaseThreshold);

				/* release work from outside copyspace if it has accumulated a viable workspace */
				if (effectiveCopyspace->getCopySize() >= _workReleaseThreshold) {

					/* pull work from effective copyspace into a workspace, leaving only trailing whitespace in copyspace */
					MM_EvacuatorWorkspace *work = _freeList.next();
					work->base = (omrobjectptr_t)effectiveCopyspace->rebase(&work->length);
					addWork(work);

					/* trim to whitelist rump whitespace left over after repeatedly rebasing copyspace */
					if (effectiveCopyspace->getWhiteSize() < _copyspaceRefreshThreshold) {
						_whiteList[evacuationRegion].add(effectiveCopyspace->trim());
					}
				}

			} else if (_scanStackFrame < _whiteStackFrame[evacuationRegion]) {

				/* force push to receiving frame if it is superior to current frame */
				push(_whiteStackFrame[evacuationRegion]);
			}

		} else if (effectiveCopyspace == &_largeCopyspace) {
			Debug_MM_true(0 == _largeCopyspace.getCopySize());

			/* object copied by other thread so clip reserved whitespace from large object copyspace onto the whitelist */
			_whiteList[evacuationRegion].add(_largeCopyspace.trim());
		}
	}

	return forwardingAddress;
}

bool
MM_Evacuator::shouldRefreshCopyspace(const EvacuationRegion evacuationRegion, const uintptr_t slotObjectSizeAfterCopy, const uintptr_t copyspaceRemainder) const
{
	if (MM_EvacuatorBase::min_reusable_whitespace >= copyspaceRemainder) {
		/* copyspace has a small whitespace remainder that can be trimmed */
		return true;
	} else if ((MM_EvacuatorBase::max_copyspace_overflow_quanta * _controller->_minimumWorkspaceSize) < _copyspaceOverflow[evacuationRegion]) {
		/* a large volume of copy has overflowed the copyspace its remaining whitespace must now be trimmed */
		return true;
	} else if (_controller->_minimumCopyspaceSize > slotObjectSizeAfterCopy) {
		/* if an array of objects is being presented refresh if a reasonable number of elements could be contained in new whitespace allocation */
		return (NULL != _scanStackFrame) && _scanStackFrame->getActiveObjectScanner()->isIndexableObject();
	}
	return false;
}

MM_EvacuatorCopyspace *
MM_Evacuator::reserveOutsideCopyspace(EvacuationRegion *evacuationRegion, const uintptr_t slotObjectSizeAfterCopy, bool useLargeCopyspace)
{
	/* failover to refresh copyspace or inject into stack whitespace or allocate solo object */
	MM_EvacuatorCopyspace *copyspace = NULL;
	MM_EvacuatorWhitespace *whitespace = NULL;

	/* preference order of potential evacuation regions */
	EvacuationRegion regions[] = { *evacuationRegion, otherOutsideRegion(*evacuationRegion) };

	/* allow tlh allocation for whitespace if possible */
	if (!useLargeCopyspace) {

		/* try to find space in or refresh outside copyspace */
		for (uintptr_t regionIndex = 0; (NULL == whitespace) && (regionIndex < 2); regionIndex += 1) {

			const EvacuationRegion region = *evacuationRegion = regions[regionIndex];
			const uintptr_t copyspaceRemainder = _copyspace[region].getWhiteSize();

			/* try to reserve whitespace in copyspace to receive object, refreshing if possible */
			if (copyspaceRemainder >= slotObjectSizeAfterCopy) {

				/* use outside copyspace for evacuation region if object will fit */
				return &_copyspace[region];

			} else if (shouldRefreshCopyspace(region, slotObjectSizeAfterCopy, copyspaceRemainder)) {

				/* try to refresh whitespace for copyspace */
				if (_whiteList[region].top() >= OMR_MAX(slotObjectSizeAfterCopy, _copyspaceRefreshThreshold)) {
					/* take top of whitelist and swap remainder whitespace from copyspace into whitelist */
					whitespace = _whiteList[region].top(slotObjectSizeAfterCopy, _copyspace[region].trim());
				} else {
					/* allocate tlh from evacuation region and trim copyspace later */
					whitespace = _controller->getWhitespace(this, region, slotObjectSizeAfterCopy);
				}

				/* if whitespace is obtained clear copyspace overflow flush condition for evacuation region */
				if (NULL != whitespace) {
					setFlushCondition(copyspaceOverflowCondition(region), false);
					_copyspaceOverflow[region] = 0;
				}

			} else {

				/* raise copyspace overflow flush condition for evacuation region when remaining whitespace id too small to use as a workspace */
				if (_copyspaceRefreshThreshold > copyspaceRemainder) {
					setFlushCondition(copyspaceOverflowCondition(region), true);
				}

				/* flush condition will force small objects to be copied into copyspace until it can be refreshed */
				if (testFlushCondition(copyspaceOverflowCondition(region))) {
					/* force copyspace refresh if too much material is overflowing */
					_copyspaceOverflow[region] += slotObjectSizeAfterCopy;
				}

				/* try injecting into stack whitespace */
				MM_EvacuatorScanspace * const whiteFrame = _whiteStackFrame[region];
				if ((NULL != whiteFrame) && (slotObjectSizeAfterCopy <= whiteFrame->getWhiteSize())) {

					/* copy object into stack scanspace holding evacuation region whitespace */
					return static_cast<MM_EvacuatorCopyspace *>(whiteFrame);
				}

				/* break to failover to solo object copy */
				break;
			}
		}
	}

	Debug_MM_true(NULL == copyspace);

	/* if no whitespace try to allocate for solo object copy */
	if (NULL == whitespace) {

		/* force use of large object copyspace */
		useLargeCopyspace = true;

		for (uintptr_t regionIndex = 0; (NULL == whitespace) && (regionIndex < 2); regionIndex += 1) {

			*evacuationRegion = regions[regionIndex];

			/* whitespace taken from whitelist may exceed object size -- remainder will be trimmed to whitelist after copy is complete */
			whitespace = _whiteList[*evacuationRegion].top(slotObjectSizeAfterCopy);

			if (NULL == whitespace) {
				/* whitespace allocation from heap will leave no remainder after copy */
				whitespace = _controller->getObjectWhitespace(this, *evacuationRegion, slotObjectSizeAfterCopy);
			}
		}
	}

	/* at this point no whitespace to refresh copyspace means failure */
	if (NULL != whitespace) {
		Debug_MM_true(slotObjectSizeAfterCopy <= whitespace->length());
		Debug_MM_true(*evacuationRegion == getEvacuationRegion(whitespace));

		/* pull whitespace from evacuation region into outside or large copyspace */
		if (useLargeCopyspace) {

			/* select large copyspace to receive whitespace */
			copyspace = &_largeCopyspace;

		} else {

			/* select and clear outside copyspace to receive whitespace */
			copyspace = &_copyspace[*evacuationRegion];

			/* distribute work in copyspace to worklist */
			if (0 < copyspace->getCopySize()) {

				/* pull remaining work in copyspace into a workspace and put it on the worklist */
				MM_EvacuatorWorkspace *work = _freeList.next();
				work->base = (omrobjectptr_t)copyspace->rebase(&work->length);
				addWork(work);
			}

			/* trim remainder whitespace from copyspace to whitelist */
			_whiteList[*evacuationRegion].add(copyspace->trim());
		}

		/* pull the reserved whitespace into the selected copyspace */
		copyspace->setCopyspace((uint8_t*)whitespace, (uint8_t*)whitespace, whitespace->length(), whitespace->isLOA());

	} else {

		/* broadcast abort condition to other evacuators */
		setAbortedCycle();

		/* record every allocation failure after abort condition raised */
		if (survivor == regions[0]) {
			_stats->_failedFlipCount += 1;
			_stats->_failedFlipBytes += slotObjectSizeAfterCopy;
		} else {
			_stats->_failedTenureCount += 1;
			_stats->_failedTenureBytes += slotObjectSizeAfterCopy;
			_stats->_failedTenureLargest = OMR_MAX(slotObjectSizeAfterCopy, _stats->_failedTenureLargest);
		}

		/* abort object evacuation */
		*evacuationRegion = unreachable;
		copyspace = NULL;
	}

	return copyspace;
}

void
MM_Evacuator::receiveWhitespace(MM_EvacuatorWhitespace *whitespace)
{
	EvacuationRegion whiteRegion = getEvacuationRegion(whitespace);
	Debug_MM_true((NULL == whitespace) || (whiteRegion < evacuate));
	_whiteList[whiteRegion].add(whitespace);
}

omrobjectptr_t
MM_Evacuator::copyForward(MM_ForwardedHeader *forwardedHeader, fomrobject_t *referringSlotAddress, MM_EvacuatorCopyspace * const copyspace, const uintptr_t originalLength, const uintptr_t forwardedLength)
{
	/* if not already forwarded object will be copied to copy head in designated copyspace */
	omrobjectptr_t const copyHead = (omrobjectptr_t)copyspace->getCopyHead();
	Debug_MM_true(isInSurvivor(copyHead) || isInTenure(copyHead));

	/* try to set forwarding address to the copy head in copyspace; otherwise do not copy, just return address forwarded by another thread */
	omrobjectptr_t forwardedAddress = forwardedHeader->setForwardedObject(copyHead);
	if (forwardedAddress == copyHead) {
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
		_stats->countObjectSize(forwardedLength, _maxInsideCopySize);
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
		/* object model fixes the flags in the destination object */
		_objectModel->fixupForwardedObject(forwardedHeader, forwardedAddress, objectAge);

#if defined(EVACUATOR_DEBUG)
		if (_controller->_debugger.isDebugCopy()) {
			OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
			char className[32];
			omrobjectptr_t parent = (NULL != _scanStackFrame) ? _scanStackFrame->getActiveObjectScanner()->getParentObject() : NULL;
			omrtty_printf("%5lu %2lu %2lu:%c copy %3s; base:%lx; copy:%lx; end:%lx; free:%lx; %lx %s %lx -> %lx %lx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex,
					(NULL != _scanStackFrame) && ((uint8_t *)forwardedAddress >= _scanStackFrame->getBase()) && ((uint8_t *)forwardedAddress < _scanStackFrame->getEnd()) ? 'I' : 'O',
					isInSurvivor(forwardedAddress) ? "new" : "old",	(uintptr_t)copyspace->getBase(), (uintptr_t)copyspace->getCopyHead(), (uintptr_t)copyspace->getEnd(), copyspace->getWhiteSize(),
					(uintptr_t)parent, _delegate.debugGetClassname(forwardedAddress, className, 32), (uintptr_t)forwardedHeader->getObject(), (uintptr_t)forwardedAddress, forwardedLength);
		}
#endif /* defined(EVACUATOR_DEBUG) */

		/* advance the copy head in the receiving copyspace and track scan/copy progress */
		copyspace->advanceCopyHead(forwardedLength);
		if ((_scannedBytesDelta >= _copiedBytesReportingDelta) || ((_copiedBytesDelta[survivor] + _copiedBytesDelta[tenure]) >= _copiedBytesReportingDelta)) {
			_controller->reportProgress(this, _copiedBytesDelta, &_scannedBytesDelta);
		}

#if defined(EVACUATOR_DEBUG)
		_delegate.debugValidateObject(forwardedAddress);
#endif /* defined(EVACUATOR_DEBUG) */
	}

	Debug_MM_true(isInSurvivor(forwardedAddress) || isInTenure(forwardedAddress) || (isInEvacuate(forwardedAddress) && isAbortedCycle()));
	return forwardedAddress;
}

bool
MM_Evacuator::isSplitablePointerArray(MM_ForwardedHeader *forwardedHeader, uintptr_t objectSizeInBytes)
{
	/* large pointer arrays are split by default but scanners for these objects may inhibit splitting eg when pruning remembered set */
	return ((MM_EvacuatorBase::min_split_indexable_size < objectSizeInBytes) && _delegate.isIndexablePointerArray(forwardedHeader));
}

void
MM_Evacuator::splitPointerArrayWork(omrobjectptr_t pointerArray)
{
	uintptr_t elements = 0;
	_delegate.getIndexableDataBounds(pointerArray, &elements);

	/* distribute elements to segments as evenly as possible and take largest segment first */
	uintptr_t segments = elements / MM_EvacuatorBase::max_split_segment_elements;
	if (0 != (elements % MM_EvacuatorBase::max_split_segment_elements)) {
		segments += 1;
	}
	uintptr_t elementsPerSegment = elements / segments;
	uintptr_t elementsThisSegment = elementsPerSegment + (elements % segments);

	omrthread_monitor_enter(_mutex);

	/* record 1-based array offsets to mark split array workspaces */
	uintptr_t offset = 1;
	while (0 < segments) {

		/* add each array segment as a workspace */
		MM_EvacuatorWorkspace* work = _freeList.next();

		work->base = pointerArray;
		work->offset = offset;
		work->length = elementsThisSegment;

		_workList.add(work);

		offset += elementsThisSegment;
		elementsThisSegment = elementsPerSegment;
		segments -= 1;
	}

	omrthread_monitor_exit(_mutex);

	/* tell the world about it */
	_controller->notifyOfWork(this);
}

bool
MM_Evacuator::shouldRememberObject(omrobjectptr_t objectPtr)
{
	Debug_MM_true((NULL != objectPtr) && isInTenure(objectPtr));
	Debug_MM_true(_objectModel->getRememberedBits(objectPtr) < (uintptr_t)0xc0);

	/* scan object for referents in the nursery */
	GC_ObjectScannerState objectScannerState;
	GC_ObjectScanner *objectScanner = _delegate.getObjectScanner(objectPtr, &objectScannerState, GC_ObjectScanner::scanRoots);
	if (NULL != objectScanner) {

		GC_SlotObject *slotPtr;
		while (NULL != (slotPtr = objectScanner->getNextSlot())) {

			omrobjectptr_t slotObjectPtr = slotPtr->readReferenceFromSlot();
			if (NULL != slotObjectPtr) {

				if (isInSurvivor(slotObjectPtr)) {

					return true;
				}

				Debug_MM_true(isInTenure(slotObjectPtr) || (isInEvacuate(slotObjectPtr) && isAbortedCycle()));
			}
		}
	}

	/* the remembered state of a class object also depends on the class statics */
	if (_objectModel->hasIndirectObjectReferents((CLI_THREAD_TYPE*)_env->getLanguageVMThread(), objectPtr)) {

		return _delegate.objectHasIndirectObjectsInNursery(objectPtr);
	}

	return false;
}

bool
MM_Evacuator::rememberObject(omrobjectptr_t object)
{
	Debug_MM_true(isInTenure(object));
	Debug_MM_true(_objectModel->getRememberedBits(object) < (uintptr_t)0xc0);

	/* try to set the REMEMBERED bit in the flags field (if it hasn't already been set) */
	bool rememberedByThisEvacuator = _objectModel->atomicSetRememberedState(object, STATE_REMEMBERED);
	if (rememberedByThisEvacuator) {
		Debug_MM_true(_objectModel->isRemembered(object));

		/* the object has been successfully marked as REMEMBERED - allocate an entry in the remembered set */
		if ((_env->_scavengerRememberedSet.fragmentCurrent < _env->_scavengerRememberedSet.fragmentTop) ||
				(0 == allocateMemoryForSublistFragment(_env->getOmrVMThread(), (J9VMGC_SublistFragment*)&_env->_scavengerRememberedSet))
		) {

			/* there is at least 1 free entry in the fragment - use it */
			_env->_scavengerRememberedSet.count++;
			uintptr_t *rememberedSetEntry = _env->_scavengerRememberedSet.fragmentCurrent++;
			*rememberedSetEntry = (uintptr_t)object;

		} else {

			/* failed to allocate a fragment - set the remembered set overflow state */
			if (!_env->getExtensions()->isRememberedSetInOverflowState()) {
				_stats->_causedRememberedSetOverflow = 1;
			}
			_env->getExtensions()->setRememberedSetOverflowState();
		}
	}

	Debug_MM_true(_objectModel->getRememberedBits(object) < (uintptr_t)0xc0);
	return rememberedByThisEvacuator;
}

#if defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
uint64_t
MM_Evacuator::startWaitTimer()
{
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		omrtty_printf("%5lu %2lu %2lu:     stall; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%lx; vow:%lx\n", _controller->sampleEvacuatorFlags(), getVolumeOfWork());
	}
#endif /* defined(EVACUATOR_DEBUG) */
	return omrtime_hires_clock();
}

void
MM_Evacuator::endWaitTimer(uint64_t waitStartTime, MM_EvacuatorWorkspace *work)
{
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
	uint64_t waitEndTime = omrtime_hires_clock();

#if defined(EVACUATOR_DEBUG)
	if (_controller->_debugger.isDebugWork()) {
		uint64_t waitMicros = omrtime_hires_delta(waitStartTime, waitEndTime, OMRPORT_TIME_DELTA_IN_MICROSECONDS);
		omrtty_printf("%5lu %2lu %2lu:    resume; ", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex);
		_controller->printEvacuatorBitmap(_env, "stalled", _controller->sampleStalledMap());
		_controller->printEvacuatorBitmap(_env, "; resuming", _controller->sampleResumingMap());
		omrtty_printf("; flags:%lx; vow:%lx; work:0x%lx; length:%lx; micros:%llu\n", _controller->sampleEvacuatorFlags(),
				getVolumeOfWork(), (uintptr_t)work, ((NULL != work) ? work->length : 0), waitMicros);
	}
#endif /* defined(EVACUATOR_DEBUG) */

#if defined(J9MODRON_TGC_PARALLEL_STATISTICS)
	if (NULL == work) {
		_stats->addToCompleteStallTime(waitStartTime, waitEndTime);
	} else {
		_stats->addToWorkStallTime(waitStartTime, waitEndTime);
	}
#endif /* defined(J9MODRON_TGC_PARALLEL_STATISTICS) */
}
#endif /* defined(J9MODRON_TGC_PARALLEL_STATISTICS) || defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

#if defined(EVACUATOR_DEBUG)
void
MM_Evacuator::debugStack(const char *stackOp, bool treatAsWork)
{
	if (_controller->_debugger.isDebugStack() || (treatAsWork && (_controller->_debugger.isDebugWork() || _controller->_debugger.isDebugBackout()))) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
		MM_EvacuatorScanspace *scanspace = (NULL != _scanStackFrame) ? _scanStackFrame : _stackBottom;
		EvacuationRegion region = getEvacuationRegion(scanspace->getBase());
		char isWhiteFrame = (evacuate > region) && (_whiteStackFrame[region] == scanspace) ? '*' : ' ';
		const char *whiteRegion = (survivor == region) ? "survivor" : "tenure";
		omrtty_printf("%5lu %2lu %2lu:%6s[%2d];%cbase:%lx; copy:%lx; end:%lx; %s:%lx; scan:%lx; unscanned:%lx; sow:%lx; tow:%lx\n",
				_controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, stackOp, (scanspace - _stackBottom),
				isWhiteFrame, (uintptr_t)scanspace->getBase(), (uintptr_t)scanspace->getCopyHead(), (uintptr_t)scanspace->getEnd(),
				whiteRegion, scanspace->getWhiteSize(), (uintptr_t)scanspace->getScanHead(), scanspace->getWorkSize(),
				_copyspace[survivor].getCopySize(), _copyspace[tenure].getCopySize());
	}
}

void
MM_Evacuator::checkSurvivor()
{
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
	omrobjectptr_t object = (omrobjectptr_t)(_heapBounds[survivor][0]);
	omrobjectptr_t end = (omrobjectptr_t)(_heapBounds[survivor][1]);
	while (isInSurvivor(object)) {
		while (isInSurvivor(object) && _objectModel->isDeadObject(object)) {
			object = (omrobjectptr_t)((uintptr_t)object + _objectModel->getSizeInBytesDeadObject(object));
		}
		if (isInSurvivor(object)) {
			_delegate.debugValidateObject(object);
			object = (omrobjectptr_t)((uintptr_t)object + _objectModel->getConsumedSizeInBytesWithHeader(object));
		}
	}
	omrtty_printf("%5lu %2lu %2lu:  survivor; end:%lx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, (uintptr_t)object);
	Debug_MM_true(object == end);
}

void
MM_Evacuator::checkTenure()
{
	OMRPORT_ACCESS_FROM_ENVIRONMENT(_env);
	MM_GCExtensionsBase *extensions = _env->getExtensions();
	omrobjectptr_t object = (omrobjectptr_t)(_heapBounds[tenure][0]);
	omrobjectptr_t end = (omrobjectptr_t)(_heapBounds[tenure][1]);
	while (extensions->isOld(object)) {
		while (extensions->isOld(object) && _objectModel->isDeadObject(object)) {
			object = (omrobjectptr_t)((uintptr_t)object + _objectModel->getSizeInBytesDeadObject(object));
		}
		if (extensions->isOld(object)) {
			_delegate.debugValidateObject(object);
			Debug_MM_true(_objectModel->getRememberedBits(object) < (uintptr_t)0xc0);
			if (_objectModel->isRemembered(object)) {
				if (!shouldRememberObject(object)) {
					omrtty_printf("%5lu %2lu %2lu:downgraded; object:%lx; flags:%lx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, (uintptr_t)object, _objectModel->getObjectFlags(object));
				}
			} else if (shouldRememberObject(object)) {
				omrtty_printf("%5lu %2lu %2lu: !remember; object:%lx; flags:%lx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, (uintptr_t)object, _objectModel->getObjectFlags(object));
				Debug_MM_true(isAbortedCycle());
			}
			object = (omrobjectptr_t)((uintptr_t)object + _objectModel->getConsumedSizeInBytesWithHeader(object));
		}
	}
	omrtty_printf("%5lu %2lu %2lu:    tenure; end:%lx\n", _controller->getEpoch()->gc, _controller->getEpoch()->epoch, _workerIndex, (uintptr_t)object);
	Debug_MM_true(object == end);
}
#else
void MM_Evacuator::debugStack(const char *stackOp, bool treatAsWork) { }
#endif /* defined(EVACUATOR_DEBUG) */
