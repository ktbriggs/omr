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

#include <stdint.h>
#include "omrport.h"

#include "AllocateDescription.hpp"
#include "AtomicSupport.hpp"
#include "Bits.hpp"
#include "Dispatcher.hpp"
#include "EvacuatorController.hpp"
#include "EvacuatorScanspace.hpp"
#include "Math.hpp"
#include "MemorySubSpace.hpp"
#include "MemorySubSpaceSemiSpace.hpp"
#include "ScavengerCopyScanRatio.hpp"
#include "ScavengerStats.hpp"

bool
MM_EvacuatorController::setEvacuatorFlag(uintptr_t flag, bool value)
{
	uintptr_t oldFlags = _evacuatorFlags;
	if (value) {
		if (!isEvacuatorFlagSet(flag)) {
			acquireController();
			if (!isEvacuatorFlagSet(flag)) {
				oldFlags = VM_AtomicSupport::bitOr(&_evacuatorFlags, flag);
			}
			releaseController();
		}
	} else {
		if (isEvacuatorFlagSet(flag)) {
			acquireController();
			if (isEvacuatorFlagSet(flag)) {
				oldFlags = VM_AtomicSupport::bitAnd(&_evacuatorFlags, ~flag);
			}
			releaseController();
		}
	}
	return (flag == (flag & oldFlags));
}

bool
MM_EvacuatorController::setAborting()
{
	/* test & set the aborting flag, return false if not previously set */
	return setEvacuatorFlag(aborting, true);
}

bool
MM_EvacuatorController::initialize(MM_EnvironmentBase *env)
{
	bool result = true;

	/* evacuator model is not instrumented for concurrent scavenger */
	Assert_MM_true(!_extensions->isEvacuatorEnabled() || !_extensions->isConcurrentScavengerEnabled());

	if (_extensions->isEvacuatorEnabled()) {
		/* if jvm is only user process cpu would likely stall if thread yielded to wait on the controller mutex so enable spinning */
		if (0 != omrthread_monitor_init_with_name(&_controllerMutex, 0, "MM_EvacuatorController::_controllerMutex")) {
			_controllerMutex = NULL;
			return false;
		}

		/* evacuator never uses monitor-enter and requires monitor-try-enter semantics to *not* yield cpu for reporter mutex so disable spinning */
		if (0 != omrthread_monitor_init_with_name(&_reporterMutex, J9THREAD_MONITOR_DISABLE_SPINNING, "MM_EvacuatorController::_reporterMutex")) {
			omrthread_monitor_destroy(_controllerMutex);
			_controllerMutex = NULL;
			_reporterMutex = NULL;
			return false;
		}

		for (uintptr_t workerIndex = 0; workerIndex < _maxGCThreads; workerIndex++) {
			_evacuatorTask[workerIndex] = NULL;
		}

		/* initialize heap region bounds, copied here at the start of each gc cycle for ease of access */
		for (intptr_t space = (intptr_t)MM_Evacuator::survivor; space <= (intptr_t)MM_Evacuator::evacuate; space += 1) {
			_heapLayout[space][0] = NULL;
			_heapLayout[space][1] = NULL;
			_memorySubspace[space] = NULL;
		}

		_history.reset();

		result = (NULL != _controllerMutex);
	}

#if defined(EVACUATOR_DEBUG)
	_debugger.setDebugFlags();
#endif /* defined(EVACUATOR_DEBUG) */

	return result;
}

void
MM_EvacuatorController::tearDown(MM_EnvironmentBase *env)
{
	if (_extensions->isEvacuatorEnabled()) {
		for (uintptr_t workerIndex = 0; workerIndex < _maxGCThreads; workerIndex++) {
			if (NULL != _evacuatorTask[workerIndex]) {
				Debug_MM_true(NULL == _evacuatorTask[workerIndex]->getEnvironment());
				_evacuatorTask[workerIndex]->kill();
				_evacuatorTask[workerIndex] = NULL;
			}
		}

		MM_Forge *forge = env->getForge();
		forge->free((void *)_boundEvacuatorBitmap);
		forge->free((void *)_stalledEvacuatorBitmap);
		forge->free((void *)_resumingEvacuatorBitmap);
		forge->free((void *)_evacuatorMask);
		forge->free((void *)_evacuatorTask);

		if (NULL != _controllerMutex) {
			omrthread_monitor_destroy(_controllerMutex);
			_controllerMutex = NULL;
		}
		if (NULL != _reporterMutex) {
			omrthread_monitor_destroy(_reporterMutex);
			_reporterMutex = NULL;
		}
	}
}

bool
MM_EvacuatorController::collectorStartup(MM_GCExtensionsBase* extensions)
{
#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	if (extensions->isEvacuatorEnabled()) {
#if defined(EVACUATOR_DEBUG)
		if (_debugger.isDebugEnd()) {
#endif /* defined(EVACUATOR_DEBUG) */
			OMRPORT_ACCESS_FROM_OMRVM(_omrVM);
			_collectorStartTime = omrtime_hires_clock();
			omrtty_printf("%5llu      :   startup; stack-depth:%llu; object-size:%llx; frame-width:%llx; work-size:%llx; work-quanta:%llx\n", (uint64_t)_history.epoch()->gc,
				(uint64_t)_extensions->evacuatorMaximumStackDepth, (uint64_t)_extensions->evacuatorMaximumInsideCopySize, (uint64_t)_extensions->evacuatorMaximumInsideCopyDistance,
				(uint64_t)_extensions->evacuatorWorkQuantumSize, (uint64_t)_extensions->evacuatorWorkQuanta);
#if defined(EVACUATOR_DEBUG)
		}
#endif /* defined(EVACUATOR_DEBUG) */
	}
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

	return true;
}

void
MM_EvacuatorController::collectorShutdown(MM_GCExtensionsBase* extensions)
{
	flushTenureWhitespace(true);

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
	if (extensions->isEvacuatorEnabled()) {
#if defined(EVACUATOR_DEBUG)
		if (_debugger.isDebugEnd()) {
#endif /* defined(EVACUATOR_DEBUG) */
			OMRPORT_ACCESS_FROM_OMRVM(_omrVM);
			uint64_t collectorElapsedMicros = omrtime_hires_delta(_collectorStartTime, omrtime_hires_clock(), OMRPORT_TIME_DELTA_IN_MICROSECONDS);
			omrtty_printf("%5llu      :  shutdown; elapsed:%llu; flushed:%llx\n", (uint64_t)_history.epoch()->gc, (uint64_t)collectorElapsedMicros, (uint64_t)_globalTenureFlushedBytes);
#if defined(EVACUATOR_DEBUG)
		}
#endif /* defined(EVACUATOR_DEBUG) */
	}
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
}

void
MM_EvacuatorController::flushTenureWhitespace(bool shutdown)
{
	uint64_t flushed = 0;

	if (_extensions->isEvacuatorEnabled()) {
		for (uintptr_t workerIndex = 0; workerIndex < _maxGCThreads; workerIndex += 1) {
			if (NULL != _evacuatorTask[workerIndex]) {
				Debug_MM_true(NULL == _evacuatorTask[workerIndex]->getEnvironment());
				flushed += _evacuatorTask[workerIndex]->flushWhitespace(MM_Evacuator::tenure);
			}
		}
		_globalTenureFlushedBytes += flushed;
	}

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
#if defined(EVACUATOR_DEBUG)
	if (_debugger.isDebugEnd()) {
#endif /* defined(EVACUATOR_DEBUG) */
		OMRPORT_ACCESS_FROM_OMRVM(_omrVM);
		omrtty_printf("%5llu      : global gc; tenure; flushed:%llx\n", (uint64_t)_history.epoch()->gc, (uint64_t)flushed);
#if defined(EVACUATOR_DEBUG)
	}
#endif /* defined(EVACUATOR_DEBUG) */
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */
}

void
MM_EvacuatorController::masterSetupForGC(MM_EnvironmentStandard *env)
{
	_evacuatorIndex = 0;
	_evacuatorFlags = 0;
	setEvacuatorFlag(breadthFirstScan, env->getExtensions()->evacuatorMaximumStackDepth <= MM_EvacuatorBase::min_scan_stack_depth);
	_evacuatorCount = 0;
	_finalDiscardedBytes = 0;
	_finalFlushedBytes = 0;
	_copiedBytes[MM_Evacuator::survivor] = 0;
	_copiedBytes[MM_Evacuator::tenure] = 0;
	_scannedBytes = 0;

	/* reset the evacuator bit maps */
	uintptr_t tailWords = 0;
	uintptr_t bitmapWords = countEvacuatorBitmapWords(&tailWords);
	for (uintptr_t map = 0; map < bitmapWords; map += 1) {
		Debug_MM_true(0 == _boundEvacuatorBitmap[map]);
		_boundEvacuatorBitmap[map] = 0;
		Debug_MM_true(0 == _stalledEvacuatorBitmap[map]);
		_stalledEvacuatorBitmap[map] = 0;
		Debug_MM_true(0 == _resumingEvacuatorBitmap[map]);
		_resumingEvacuatorBitmap[map] = 0;
		_evacuatorMask[map] = 0;
	}
	_stalledEvacuatorCount = 0;

	/* set up controller's subspace and layout arrays to match collector subspaces */
	_memorySubspace[MM_Evacuator::evacuate] = _evacuateMemorySubSpace;
	_memorySubspace[MM_Evacuator::survivor] = _survivorMemorySubSpace;
	_memorySubspace[MM_Evacuator::tenure] = _tenureMemorySubSpace;

	_heapLayout[MM_Evacuator::evacuate][0] = (uint8_t *)_evacuateSpaceBase;
	_heapLayout[MM_Evacuator::evacuate][1] = (uint8_t *)_evacuateSpaceTop;
	_heapLayout[MM_Evacuator::survivor][0] = (uint8_t *)_survivorSpaceBase;
	_heapLayout[MM_Evacuator::survivor][1] = (uint8_t *)_survivorSpaceTop;
	_heapLayout[MM_Evacuator::tenure][0] = (uint8_t *)_extensions->_tenureBase;
	_heapLayout[MM_Evacuator::tenure][1] = _heapLayout[MM_Evacuator::tenure][0] + _extensions->_tenureSize;

	/* reset upper bounds for tlh allocation size, indexed by outside region -- these will be readjusted when thread count is stable */
	_copyspaceAllocationCeiling[MM_Evacuator::survivor] = _copyspaceAllocationCeiling[MM_Evacuator::tenure] = _maximumCopyspaceSize;
	_objectAllocationCeiling[MM_Evacuator::survivor] = _objectAllocationCeiling[MM_Evacuator::tenure] = ~(uintptr_t)0xff;

	/* prepare the evacuator delegate class and enable it to add private flags for the cycle */
	_evacuatorFlags |= MM_EvacuatorDelegate::prepareForEvacuation(env);
}

MM_Evacuator *
MM_EvacuatorController::bindWorker(MM_EnvironmentStandard *env)
{
	/* get an unbound evacuator instance */
	uintptr_t workerIndex = VM_AtomicSupport::add(&_evacuatorIndex, 1) - 1;

	/* instantiate evacuator task for this worker thread if required (evacuators are instantiated once and persist until vm shuts down) */
	if (NULL == _evacuatorTask[workerIndex]) {
		_evacuatorTask[workerIndex] = MM_Evacuator::newInstance(workerIndex, this, &_extensions->objectModel, _extensions->evacuatorMaximumStackDepth, _extensions->evacuatorMaximumInsideCopySize, _extensions->evacuatorMaximumInsideCopyDistance, _extensions->getForge());
		Assert_MM_true(NULL != _evacuatorTask[workerIndex]);
	}

	/* controller doesn't have final view on evacuator thread count until after first task is dispatched ... */
	if (0 == _evacuatorCount) {

		acquireController();

		/* ... so first evacuator to reach this point must complete thread count dependent initialization */
		if (0 == _evacuatorCount) {

			/* all evacuator threads must have same view of dispatched thread count at this point */
			_evacuatorCount = env->_currentTask->getThreadCount();
			fillEvacuatorBitmap(_evacuatorMask);

			/* set upper bounds for tlh allocation size, indexed by outside region -- reduce these for small survivor spaces */
			uintptr_t copyspaceSize = _maximumCopyspaceSize;
			uint64_t projectedEvacuationBytes = calculateProjectedEvacuationBytes();
			while ((copyspaceSize > _minimumCopyspaceSize) && ((4 * copyspaceSize * _evacuatorCount) > projectedEvacuationBytes)) {
				/* scale down tlh allocation limit until maximal cache size is small enough to ensure adequate distribution */
				copyspaceSize -= _minimumCopyspaceSize;
			}
			_copyspaceAllocationCeiling[MM_Evacuator::survivor] = _copyspaceAllocationCeiling[MM_Evacuator::tenure] = OMR_MAX(_minimumCopyspaceSize, copyspaceSize);

			/* reporting delta roughly partitions copy/scan byte count to produce a preset number of epochs per gc cycle */
			_copiedBytesReportingDelta = MM_Math::roundToCeiling(epochs_per_cycle, (uintptr_t)(projectedEvacuationBytes / (_evacuatorCount * epochs_per_cycle)));
			if (_copiedBytesReportingDelta < _minimumCopyspaceSize) {
				_copiedBytesReportingDelta = _minimumCopyspaceSize;
			}

			/* end of epoch is signaled when aggregate copy volume within epoch exceeds product of evacuator count and preset reporting delta */
			_nextEpochCopiedBytesThreshold = _copiedBytesReportingDelta * _evacuatorCount;
			_history.reset(_extensions->scavengerStats._gcCount, _copyspaceAllocationCeiling[MM_Evacuator::survivor], _copyspaceAllocationCeiling[MM_Evacuator::tenure]);

			/* mark start of first epoch */
			OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
			_epochTimestamp = omrtime_hires_clock();

#if defined(EVACUATOR_DEBUG)|| defined(EVACUATOR_DEBUG_ALWAYS)
#if defined(EVACUATOR_DEBUG)
			if (_debugger.isDebugEnd()) {
#endif /* defined(EVACUATOR_DEBUG) */
				omrtty_printf("%5llu      :  gc start; survivor{%llx %llx} tenure{%llx %llx} evacuate{%llx %llx}; threads:%llu; projection:%llx; allocation:%llx\n",
						(uint64_t)_extensions->scavengerStats._gcCount,
						(uint64_t)_heapLayout[0][0], (uint64_t)_heapLayout[0][1],
						(uint64_t)_heapLayout[1][0], (uint64_t)_heapLayout[1][1],
						(uint64_t)_heapLayout[2][0], (uint64_t)_heapLayout[2][1],
						(uint64_t)_evacuatorCount, projectedEvacuationBytes, (uint64_t)copyspaceSize);
#if defined(EVACUATOR_DEBUG)
			}
#endif /* defined(EVACUATOR_DEBUG) */
#endif /* defined(EVACUATOR_DEBUG)|| defined(EVACUATOR_DEBUG_ALWAYS) */
		}

		releaseController();
	}

	/* bind the evacuator to the gc cycle */
	_evacuatorTask[workerIndex]->bindWorkerThread(env, _tenureMask, _heapLayout, _copiedBytesReportingDelta);
	setEvacuatorBit(workerIndex, _boundEvacuatorBitmap);
	VM_AtomicSupport::readBarrier();

#if defined(EVACUATOR_DEBUG)
	Debug_MM_true(_evacuatorCount > workerIndex);
	Debug_MM_true(_evacuatorCount == env->_currentTask->getThreadCount());
	Debug_MM_true(testEvacuatorBit(workerIndex, _evacuatorMask));
	Debug_MM_true(isEvacuatorBitmapFull(_evacuatorMask));
	if (_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
		omrtty_printf("%5lu %2llu %2llu: %cbind[%2llu]; ", getEpoch()->gc, (uint64_t)getEpoch()->epoch, (uint64_t)workerIndex, env->isMasterThread() ? '*' : ' ', (uint64_t)_evacuatorCount);
		printEvacuatorBitmap(env, "bound", _boundEvacuatorBitmap);
		printEvacuatorBitmap(env, "; stalled", _stalledEvacuatorBitmap);
		printEvacuatorBitmap(env, "; resuming", _resumingEvacuatorBitmap);
		printEvacuatorBitmap(env, "; mask", _evacuatorMask);
		omrtty_printf("; flags%llx; threads:%llu; reporting:%llx\n", (uint64_t)_evacuatorFlags, (uint64_t)env->_currentTask->getThreadCount(), (uint64_t)_copiedBytesReportingDelta);
	}
#endif /* defined(EVACUATOR_DEBUG) */

	return _evacuatorTask[workerIndex];
}

void
MM_EvacuatorController::unbindWorker(MM_EnvironmentStandard *env)
{
	MM_Evacuator *evacuator = env->getEvacuator();

	/* passivate the evacuator instance */
	clearEvacuatorBit(evacuator->getWorkerIndex(), _boundEvacuatorBitmap);

	/* pull final remaining metrics from evacuator */
	if (isEvacuatorBitmapEmpty(_boundEvacuatorBitmap)) {
		Debug_MM_true(isEvacuatorBitmapEmpty(_stalledEvacuatorBitmap) || isAborting());
		_finalEvacuatedBytes = _copiedBytes[MM_Evacuator::survivor] + _copiedBytes[MM_Evacuator::tenure];
		Debug_MM_true((_finalEvacuatedBytes == _scannedBytes) || isAborting());
	}
	VM_AtomicSupport::addU64(&_finalDiscardedBytes, evacuator->getDiscarded());
	VM_AtomicSupport::addU64(&_finalFlushedBytes, evacuator->getFlushed());

#if defined(EVACUATOR_DEBUG)
	if (_debugger.isDebugCycle()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
		omrtty_printf("%5lu %2llu %2llu:    unbind; ", getEpoch()->gc, (uint64_t)getEpoch()->epoch, (uint64_t)evacuator->getWorkerIndex());
		printEvacuatorBitmap(env, "bound", _boundEvacuatorBitmap);
		printEvacuatorBitmap(env, "; stalled", _stalledEvacuatorBitmap);
		printEvacuatorBitmap(env, "; resuming", _resumingEvacuatorBitmap);
		omrtty_printf("; flags:%llx\n", (uint64_t)_evacuatorFlags);
	}
	if (_debugger.isDebugHeapCheck() && isEvacuatorBitmapEmpty(_boundEvacuatorBitmap)) {
		evacuator->checkSurvivor();
		evacuator->checkTenure();
	}
#endif /* defined(EVACUATOR_DEBUG) */

	evacuator->unbindWorkerThread(env);
}

bool
MM_EvacuatorController::isWaitingToCompleteStall(MM_Evacuator *worker, MM_EvacuatorWorkPacket *work)
{
	uintptr_t workerIndex = worker->getWorkerIndex();

	if (NULL == work) {

		/* this worker is stalled or stalling -- its stall bit will remain set until it acknowledges that its resume bit has been set */
		uint64_t otherStalledEvacuators = setEvacuatorBit(workerIndex, _stalledEvacuatorBitmap);

		/* the thread that atomically sets the last stall bit to complete the stalled bitmap will end the scan cycle and notify other stalled evacuators */
		if (0 == (otherStalledEvacuators & getEvacuatorBitMask(workerIndex))) {

			VM_AtomicSupport::add(&_stalledEvacuatorCount, 1);
			if (isEvacuatorBitmapFull(_stalledEvacuatorBitmap)) {

				/* check that none of the evacuators in the stalled bitmap are resuming (ie, with work) */
				if (isEvacuatorBitmapEmpty(_resumingEvacuatorBitmap)) {
					Debug_MM_true(_stalledEvacuatorCount == _evacuatorCount);
					Debug_MM_true(worker->getEnvironment()->_currentTask->getThreadCount() == _stalledEvacuatorCount);
					Debug_MM_true(isAborting() || ((_copiedBytes[MM_Evacuator::survivor] + _copiedBytes[MM_Evacuator::tenure]) == _scannedBytes));

					/* all evacuators are stalled at this point and can now complete or abort scan cycle -- set them all up to resume */
					fillEvacuatorBitmap(_resumingEvacuatorBitmap);

#if defined(EVACUATOR_DEBUG)
					if (_debugger.isDebugCycle() || _debugger.isDebugWork()) {
						MM_EnvironmentBase *env = worker->getEnvironment();
						OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
						omrtty_printf("%5lu %2llu %2llu:   complete; ", _history.epoch()->gc, (uint64_t)_history.epoch()->epoch, (uint64_t)worker->getWorkerIndex());
						printEvacuatorBitmap(env, "stalled", _stalledEvacuatorBitmap);
						printEvacuatorBitmap(env, "; resuming", _resumingEvacuatorBitmap);
						omrtty_printf("; flags:%llx; copied:%llx; scanned:%llx\n",  (uint64_t)_evacuatorFlags, (_copiedBytes[MM_Evacuator::survivor] + _copiedBytes[MM_Evacuator::tenure]), _scannedBytes);
					}
#endif /* defined(EVACUATOR_DEBUG) */

					/* notify the other stalled evacuators to  resume after they call this method once more */
					omrthread_monitor_notify_all(_controllerMutex);
				}
			}
		}
	} else {

		Debug_MM_true(!isAborting() && isStalledEvacuator(workerIndex));

		/* this evacuator has work and can resume, or no evacuators have work and all are resuming to complete scan */
		setEvacuatorBit(workerIndex, _resumingEvacuatorBitmap);
	}

	Debug_MM_true(testEvacuatorBit(workerIndex, _stalledEvacuatorBitmap));

	return !testEvacuatorBit(workerIndex, _resumingEvacuatorBitmap);
}

MM_EvacuatorWorkPacket *
MM_EvacuatorController::continueAfterStall(MM_Evacuator *worker, MM_EvacuatorWorkPacket *work)
{
	uintptr_t workerIndex = worker->getWorkerIndex();

	/* stalled and resuming bits must be set for the evacuator */
	Debug_MM_true(testEvacuatorBit(workerIndex, _stalledEvacuatorBitmap));
	Debug_MM_true(testEvacuatorBit(workerIndex, _resumingEvacuatorBitmap));

	/* clear the stalled and resuming bits for the evacuator */
	clearEvacuatorBit(workerIndex, _stalledEvacuatorBitmap);
	clearEvacuatorBit(workerIndex, _resumingEvacuatorBitmap);
	VM_AtomicSupport::subtract(&_stalledEvacuatorCount, 1);

	/* no work at this point means all evacuators are completing or aborting the scan cycle */
	if (NULL != work) {

		/* notify any stalled evacuator that there may be work available */
		omrthread_monitor_notify(_controllerMutex);

	} else {
		Debug_MM_true4(worker->getEnvironment(), isEvacuatorBitmapEmpty(_stalledEvacuatorBitmap) == isEvacuatorBitmapEmpty(_resumingEvacuatorBitmap), "*stalled=%llx; *resuming=%llx; *evacuators=%llx; *bound=%llx\n",
				*_stalledEvacuatorBitmap, *_resumingEvacuatorBitmap, *_evacuatorMask, *_boundEvacuatorBitmap);

		/* complete scan for this evacuator */
		worker->scanComplete();
	}

#if defined(EVACUATOR_DEBUG)
	if ((_debugger.isDebugCycle() || _debugger.isDebugWork()) && (NULL == work) && isEvacuatorBitmapEmpty(_stalledEvacuatorBitmap) && isEvacuatorBitmapEmpty(_resumingEvacuatorBitmap)) {
		MM_EnvironmentBase *env = worker->getEnvironment();
		OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
		omrtty_printf("%5lu %2llu %2llu:  end scan; ", _history.epoch()->gc, (uint64_t)_history.epoch()->epoch, (uint64_t)(uint64_t)worker->getWorkerIndex());
		printEvacuatorBitmap(env, "stalled", _stalledEvacuatorBitmap);
		printEvacuatorBitmap(env, "; resuming", _resumingEvacuatorBitmap);
		omrtty_printf("; flags:%llx; copied:%llx; scanned:%llx\n",  (uint64_t)_evacuatorFlags, (_copiedBytes[MM_Evacuator::survivor] + _copiedBytes[MM_Evacuator::tenure]), _scannedBytes);
	}
#endif /* defined(EVACUATOR_DEBUG) */

	return work;
}

void
MM_EvacuatorController::reportProgress(MM_Evacuator *worker,  uint64_t *copied, uint64_t *scanned)
{
#if !defined(EVACUATOR_DEBUG)
	VM_AtomicSupport::addU64(&_copiedBytes[MM_Evacuator::survivor], copied[MM_Evacuator::survivor]);
	VM_AtomicSupport::addU64(&_copiedBytes[MM_Evacuator::tenure], copied[MM_Evacuator::tenure]);
	VM_AtomicSupport::addU64(&_scannedBytes, *scanned);
#else
	uint64_t survivorCopied = VM_AtomicSupport::addU64(&_copiedBytes[MM_Evacuator::survivor], copied[MM_Evacuator::survivor]);
	uint64_t tenureCopied = VM_AtomicSupport::addU64(&_copiedBytes[MM_Evacuator::tenure], copied[MM_Evacuator::tenure]);
	uint64_t totalScanned = VM_AtomicSupport::addU64(&_scannedBytes, *scanned);

	uint64_t totalCopied = survivorCopied + tenureCopied;
	if ((totalCopied == totalScanned) || (totalCopied >= _nextEpochCopiedBytesThreshold)) {
		uint64_t lastEpochCopiedBytesThreshold = _nextEpochCopiedBytesThreshold;
		uint64_t nextEpochCopiedBytesThreshold = lastEpochCopiedBytesThreshold + (_copiedBytesReportingDelta * _evacuatorCount);
		if (lastEpochCopiedBytesThreshold == VM_AtomicSupport::lockCompareExchangeU64(&_nextEpochCopiedBytesThreshold, lastEpochCopiedBytesThreshold, nextEpochCopiedBytesThreshold)) {
			if (0 == omrthread_monitor_try_enter(_reporterMutex)) {
				if (nextEpochCopiedBytesThreshold == _nextEpochCopiedBytesThreshold) {
					OMRPORT_ACCESS_FROM_ENVIRONMENT(worker->getEnvironment());
					uint64_t currentTimestamp = omrtime_hires_clock();

					MM_EvacuatorHistory::Epoch *epoch = _history.add();
					epoch->gc = worker->getEnvironment()->_scavengerStats._gcCount;
					epoch->duration = omrtime_hires_delta(_epochTimestamp, currentTimestamp, OMRPORT_TIME_DELTA_IN_MICROSECONDS);
					epoch->survivorAllocationCeiling = _copyspaceAllocationCeiling[MM_Evacuator::survivor];
					epoch->tenureAllocationCeiling = _copyspaceAllocationCeiling[MM_Evacuator::tenure];
					epoch->stalled = _stalledEvacuatorCount;
					epoch->survivorCopied = survivorCopied;
					epoch->tenureCopied = tenureCopied;
					epoch->scanned = totalScanned;

					_debugger.setDebugCycleAndEpoch(epoch->gc, epoch->epoch);
					_epochTimestamp = currentTimestamp;
				}
				omrthread_monitor_exit(_reporterMutex);
			}
		}
	}
#endif /* defined(EVACUATOR_DEBUG) */
	copied[MM_Evacuator::survivor] = 0;
	copied[MM_Evacuator::tenure] = 0;
	*scanned = 0;
}

uint64_t
MM_EvacuatorController::calculateProjectedEvacuationBytes()
{
	return (_heapLayout[MM_Evacuator::survivor][1] - _heapLayout[MM_Evacuator::survivor][0]);
}

uintptr_t
MM_EvacuatorController::calculateOptimalWhitespaceSize(uintptr_t evacuatorVolumeOfWork, MM_Evacuator::EvacuationRegion region)
{
	/* be greedy -- unused tenure whitespace can be used in next generational gc unless flushed for global gc */
	uintptr_t whitesize = _copyspaceAllocationCeiling[region];

	/* survivor whitespace can't be retained between gc cycles so try to reduce aggregate remainder whitespace at end */
	if ((MM_Evacuator::survivor == region) && (_minimumWorkspaceSize > evacuatorVolumeOfWork)) {

		/* limit survivor allocations as evacuators stall out at the tail end of heap scan */
		if ((5 * _copiedBytes[MM_Evacuator::survivor]) > (4 * calculateProjectedEvacuationBytes())) {

			/* scale down aggressively by aggregate evacuator bandwidth (0 <= (x+x^3)/2 <= x <= 1 for 0 <= x <= 1) */
			double scale = (double)(_evacuatorCount - _stalledEvacuatorCount) / (double)_evacuatorCount;
			whitesize = (uintptr_t)((double)whitesize * ((scale + (scale * scale * scale)) / 2));
		}
	}

	/* round down by minimum copyspace size -- result will be object aligned */
	return MM_Math::roundToFloor(_minimumCopyspaceSize, OMR_MAX(_minimumCopyspaceSize, whitesize));
}

uintptr_t
MM_EvacuatorController::calculateOptimalWorkPacketSize(uint64_t evacuatorVolumeOfWork)
{
	uintptr_t worksize = _minimumWorkspaceSize;

	/* allow worklist volume to double if no other evacuators are stalled*/
	if (0 == _stalledEvacuatorCount) {
		worksize = OMR_MIN((uintptr_t)evacuatorVolumeOfWork, _maximumWorkspaceSize);
	}

	/* scale down work packet size to minimum if any other evacuators are stalled */
	return alignToObjectSize(OMR_MAX(worksize, _minimumWorkspaceSize));
}

MM_EvacuatorWhitespace *
MM_EvacuatorController::getWhitespace(MM_Evacuator *evacuator, MM_Evacuator::EvacuationRegion region, uintptr_t length)
{
	MM_EvacuatorWhitespace *whitespace = NULL;
	MM_EnvironmentBase *env = evacuator->getEnvironment();

	/* try to allocate a tlh unless object won't fit in outside copyspace remainder and remainder is still too big to whitelist */
	uintptr_t optimalSize =  (0 < length) ? calculateOptimalWhitespaceSize(evacuator->getVolumeOfWork(), region) : _minimumCopyspaceSize;
	uintptr_t maximumLength = optimalSize;
	if (length <= maximumLength) {

		/* try to allocate tlh in region to contain at least length bytes */
		uintptr_t limitSize = OMR_MAX(_minimumCopyspaceSize, length);
		while ((NULL == whitespace) && (optimalSize >= limitSize)) {

			void *addrBase = NULL, *addrTop = NULL;
			MM_AllocateDescription allocateDescription(0, 0, false, true);
			allocateDescription.setCollectorAllocateExpandOnFailure(MM_Evacuator::tenure == region);
			void *allocation = (MM_EvacuatorWhitespace *)getMemorySubspace(region)->collectorAllocateTLH(env, this, &allocateDescription, optimalSize, addrBase, addrTop);
			if (NULL != allocation) {

				 bool isLOA = (MM_Evacuator::tenure == region) && allocateDescription.isLOAAllocation();
				/* got a tlh of some size <= optimalSize */
				uintptr_t whitesize = (uintptr_t)addrTop - (uintptr_t)addrBase;
				whitespace = MM_EvacuatorWhitespace::whitespace(allocation, whitesize, isLOA);

				env->_scavengerStats.countCopyCacheSize(whitesize, _maximumCopyspaceSize);
				if (MM_Evacuator::survivor == region) {
					env->_scavengerStats._semiSpaceAllocationCountSmall += 1;
				} else {
					env->_scavengerStats._tenureSpaceAllocationCountSmall += 1;
				}

			} else {

				/* lower the tlh allocation ceiling for the region */
				uintptr_t allocationCeiling = _copyspaceAllocationCeiling[region] - _minimumCopyspaceSize;
				while (allocationCeiling < _copyspaceAllocationCeiling[region]) {
					VM_AtomicSupport::lockCompareExchange(&_copyspaceAllocationCeiling[region], _copyspaceAllocationCeiling[region], allocationCeiling);
				}

				/* and try again using a reduced optimalSize */
				optimalSize = _copyspaceAllocationCeiling[region];
			}
		}

		Debug_MM_true4(evacuator->getEnvironment(), (NULL == whitespace) || (MM_EvacuatorBase::max_copyspace_remainder <= whitespace->length()),
				"%s tlh whitespace should not be less minimum tlh size: requested=%llx; whitespace=%llx; limit=%llx\n",
				((MM_Evacuator::survivor == region) ? "survivor" : "tenure"), length, whitespace->length(), _copyspaceAllocationCeiling[region]);

		/* hand off any unused tlh allocation to evacuator to reuse later */
		if ((NULL != whitespace) && (whitespace->length() < length)) {
			evacuator->receiveWhitespace(whitespace);
			whitespace = NULL;
		}
	}

#if defined(EVACUATOR_DEBUG)
	if ((NULL != whitespace) && _debugger.isDebugAllocate()) {
		OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
		omrtty_printf("%5lu %2llu %2llu:  allocate; %s; %llx %llx %llx %llx %llx %llx %llx\n",
				getEpoch()->gc, (uint64_t)getEpoch()->epoch, (uint64_t)evacuator->getWorkerIndex(), ((MM_Evacuator::survivor == region) ? "survivor" : "tenure"),
				(uint64_t)whitespace, (uint64_t)((NULL != whitespace) ? whitespace->length() : 0), (uint64_t)length, (uint64_t)maximumLength, (uint64_t)optimalSize,
				(uint64_t)_copyspaceAllocationCeiling[region], (uint64_t)_objectAllocationCeiling[region]);
	}
	if ((NULL != whitespace) && _debugger.isDebugPoisonDiscard()) {
		MM_EvacuatorWhitespace::poison(whitespace);
	}
#endif /* defined(EVACUATOR_DEBUG) */

	return whitespace;
}

MM_EvacuatorWhitespace *
MM_EvacuatorController::getObjectWhitespace(MM_Evacuator *evacuator, MM_Evacuator::EvacuationRegion region, uintptr_t length)
{
	MM_EvacuatorWhitespace *whitespace = NULL;
	MM_EnvironmentBase *env = evacuator->getEnvironment();

	/* allocate minimal (this object's exact) size */
	MM_AllocateDescription allocateDescription(length, 0, false, true);
	allocateDescription.setCollectorAllocateExpandOnFailure(MM_Evacuator::tenure == region);
	void *allocation = getMemorySubspace(region)->collectorAllocate(env, this, &allocateDescription);
	if (NULL != allocation) {
		Debug_MM_true(isObjectAligned(allocation));

		bool isLOA = (MM_Evacuator::tenure == region) && allocateDescription.isLOAAllocation();
		whitespace = MM_EvacuatorWhitespace::whitespace(allocation, length, isLOA);

		env->_scavengerStats.countCopyCacheSize(length, _maximumCopyspaceSize);
		if (MM_Evacuator::survivor == region) {
			env->_scavengerStats._semiSpaceAllocationCountLarge += 1;
		} else {
			env->_scavengerStats._tenureSpaceAllocationCountLarge += 1;
		}

	} else {

		/* lower the object allocation ceiling for the region */
		while (length < _objectAllocationCeiling[region]) {
			VM_AtomicSupport::lockCompareExchange(&_objectAllocationCeiling[region], _objectAllocationCeiling[region], length);
		}
	}

	return whitespace;
}

/* calculate the number of active words in the evacuator bitmaps */
uintptr_t
MM_EvacuatorController::countEvacuatorBitmapWords(uintptr_t *tailWords)
{
	*tailWords = (0 != (_evacuatorCount & index_to_map_word_modulus)) ? 1 : 0;
	return (_evacuatorCount >> index_to_map_word_shift) + *tailWords;
}

/* test evacuator bitmap for all 0s (reliable only when caller holds controller mutex) */
bool
MM_EvacuatorController::isEvacuatorBitmapEmpty(volatile uint64_t *bitmap)
{
	uintptr_t tailWords = 0;
	uintptr_t bitmapWords = countEvacuatorBitmapWords(&tailWords);
	for (uintptr_t map = 0; map < bitmapWords; map += 1) {
		if (0 != bitmap[map]) {
			return false;
		}
	}
	return true;
}

/* test evacuator bitmap for all 1s (reliable only when caller holds controller mutex) */
bool
MM_EvacuatorController::isEvacuatorBitmapFull(volatile uint64_t *bitmap)
{
	uintptr_t tailWords = 0;
	uintptr_t bitmapWords = countEvacuatorBitmapWords(&tailWords);
	uintptr_t fullWords = bitmapWords - tailWords;
	for (uintptr_t map = 0; map < fullWords; map += 1) {
		if (~(uint64_t)0 != bitmap[map]) {
			return false;
		}
	}
	if (0 < tailWords) {
		uint64_t tailBits = ((uint64_t)1 << (_evacuatorCount & index_to_map_word_modulus)) - 1;
		if (tailBits != bitmap[fullWords]) {
			return false;
		}
	}
	return true;
}

/* fill evacuator bitmap with all 1s (reliable only when caller holds controller mutex) */
void
MM_EvacuatorController::fillEvacuatorBitmap(volatile uint64_t * bitmap)
{
	uintptr_t tailWords = 0;
	uintptr_t bitmapWords = countEvacuatorBitmapWords(&tailWords);
	uintptr_t fullWords = bitmapWords - tailWords;
	for (uintptr_t map = 0; map < fullWords; map += 1) {
		bitmap[map] = ~(uint64_t)0;
	}
	if (0 < tailWords) {
		bitmap[fullWords] = ((uint64_t)1 << (_evacuatorCount & index_to_map_word_modulus)) - 1;
	}
}

/* set evacuator bit in evacuator bitmap */
uint64_t
MM_EvacuatorController::setEvacuatorBit(uintptr_t evacuatorIndex, volatile uint64_t *bitmap)
{
	uint64_t evacuatorMask = 0;
	uintptr_t evacuatorMap = mapEvacuatorIndexToMapAndMask(evacuatorIndex, &evacuatorMask);
	return VM_AtomicSupport::bitOrU64(&bitmap[evacuatorMap], evacuatorMask);
}

/* clear evacuator bit in evacuator bitmap */
void
MM_EvacuatorController::clearEvacuatorBit(uintptr_t evacuatorIndex, volatile uint64_t *bitmap)
{
	uint64_t evacuatorMask = 0;
	uintptr_t evacuatorMap = mapEvacuatorIndexToMapAndMask(evacuatorIndex, &evacuatorMask);
	VM_AtomicSupport::bitAndU64(&bitmap[evacuatorMap], ~evacuatorMask);
}

#if defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS)
void
MM_EvacuatorController::printEvacuatorBitmap(MM_EnvironmentBase *env, const char *label, volatile uint64_t *bitmap)
{
	OMRPORT_ACCESS_FROM_ENVIRONMENT(env);

	uintptr_t tailWords = 0;
	uintptr_t bitmapWords = countEvacuatorBitmapWords(&tailWords);
	omrtty_printf("%s:%llx", label, bitmap[0]);
	for (uintptr_t map = 1; map < bitmapWords; map += 1) {
		omrtty_printf(" %llx", bitmap[map]);
	}
}

uint64_t
MM_EvacuatorController::sumStackActivations(uint64_t *stackActivations, uintptr_t maxFrame)
{
	uint64_t sum = 0;
	for (uintptr_t depth = 0; depth < maxFrame; depth += 1) {
		stackActivations[depth] = 0;
		for (uintptr_t evacuator = 0; evacuator < _evacuatorCount; evacuator += 1) {
			stackActivations[depth] += _evacuatorTask[evacuator]->getStackActivationCount(depth);
		}
		sum += stackActivations[depth];
	}
	return sum;
}

void
MM_EvacuatorController::waitToSynchronize(MM_Evacuator *worker, const char *id)
{
#if defined(EVACUATOR_DEBUG)
	Debug_MM_true(!testEvacuatorBit(worker->getWorkerIndex(), _stalledEvacuatorBitmap));
	Debug_MM_true(!testEvacuatorBit(worker->getWorkerIndex(), _resumingEvacuatorBitmap));
	if (_debugger.isDebugCycle()) {
		MM_EnvironmentBase *env = worker->getEnvironment();
		OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
		omrtty_printf("%5lu %2llu %2llu:      sync; ", getEpoch()->gc, (uint64_t)getEpoch()->epoch, (uint64_t)worker->getWorkerIndex());
		printEvacuatorBitmap(env, "stalled", _stalledEvacuatorBitmap);
		printEvacuatorBitmap(env, "; resuming", _resumingEvacuatorBitmap);
		omrtty_printf("; flags:%llx; workunit:%llx; %s\n", (uint64_t)_evacuatorFlags, (uint64_t)worker->getEnvironment()->getWorkUnitIndex(), MM_EvacuatorBase::callsite(id));
	}
#endif /* defined(EVACUATOR_DEBUG) */
}

void
MM_EvacuatorController::continueAfterSynchronizing(MM_Evacuator *worker, uint64_t startTime, uint64_t endTime, const char *id)
{
#if defined(EVACUATOR_DEBUG)
	Debug_MM_true(!testEvacuatorBit(worker->getWorkerIndex(), _stalledEvacuatorBitmap));
	Debug_MM_true(!testEvacuatorBit(worker->getWorkerIndex(), _resumingEvacuatorBitmap));
	if (_debugger.isDebugCycle()) {
		MM_EnvironmentBase *env = worker->getEnvironment();
		OMRPORT_ACCESS_FROM_ENVIRONMENT(env);
		uint64_t waitMicros = omrtime_hires_delta(startTime, endTime, OMRPORT_TIME_DELTA_IN_MICROSECONDS);
		omrtty_printf("%5lu %2llu %2llu:  continue; ", getEpoch()->gc, (uint64_t)getEpoch()->epoch, (uint64_t)worker->getWorkerIndex());
		printEvacuatorBitmap(env, "stalled", _stalledEvacuatorBitmap);
		printEvacuatorBitmap(env, "; resuming", _resumingEvacuatorBitmap);
		omrtty_printf("; flags:%llx; micros:%llx; %s\n", (uint64_t)_evacuatorFlags, waitMicros, MM_EvacuatorBase::callsite(id));
	}
#endif /* defined(EVACUATOR_DEBUG) */
}
#endif /* defined(EVACUATOR_DEBUG) || defined(EVACUATOR_DEBUG_ALWAYS) */

