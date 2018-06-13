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

#ifndef EVACUATORWORKLIST_HPP_
#define EVACUATORWORKLIST_HPP_

#include "omrmutex.h"

#include "Base.hpp"
#include "EvacuatorBase.hpp"
#include "Forge.hpp"
#include "ForwardedHeader.hpp"

/**
 * Location and length in bytes of a contiguous strip of copy in survivor or tenure space. These
 * are allocated from objects already evacuated, overlaying the object body starting at object
 * head + 8 bytes and arranged in linked lists. Each evacuator thread  maintains two such lists
 * -- one for its unscanned queue and one for free MM_EvacuatorWorkspace objects.
 *
 */
typedef struct MM_EvacuatorWorkspace {
	omrobjectptr_t base;			/* points to base of material to be scanned in workspace */
	uintptr_t length;				/* length of material to be scanned in workspace, starting from base + offset */
	uintptr_t offset;				/* >0 for split arrays only, offset from base to first slot to scan */
	MM_EvacuatorWorkspace *next;	/* points to next workspace in queue, or NULL */
} MM_EvacuatorWorkspace;

/**
 * Linked list of free MM_EvacuatorWorkspace is accessed only by owning thread, in LIFO order.
 */
class MM_EvacuatorFreelist : public MM_Base
{
/*
 * Data members
 */
public:

private:
	/* if starved for free elements, allocate a chunk of this size from system memory and refresh */
	static const uintptr_t workspace_chunksize = 128;

	/* underflow chunk obtained from evacuator stack or system memory to seed or refresh free list */
	typedef struct UnderflowChunk {
		MM_EvacuatorWorkspace workspace[workspace_chunksize];
		UnderflowChunk *next;
	} UnderflowChunk;

	MM_EvacuatorWorkspace *_head;	/* head of free list (last one in) */
	volatile uintptr_t _count;		/* number of elements on free list */
	UnderflowChunk *_underflow;		/* head of list of underflow chunks allocated from system memory */
	MM_Forge *_forge;				/* memory allocator for underflow chunks */

protected:

/*
 * Function members
 */
public:
	/**
	 * Mark workspace as free and return its successor
	 */
	MM_EvacuatorWorkspace *flush(MM_EvacuatorWorkspace *workspace)
	{
		MM_EvacuatorWorkspace *next = workspace->next;

		workspace->base = NULL;
		workspace->length = 0;
		workspace->offset = 0;
		workspace->next = NULL;

		return next;
	}

	/**
	 * Return the number of contained free elements
	 */
	uintptr_t getCount() { return _count; }

	/**
	 * Get the next available free workspace
	 *
	 * @return the next available free workspace, or NULL if none available
	 */
	MM_EvacuatorWorkspace *
	next()
	{
		Debug_MM_true((0 == _count) == (NULL == _head));
		if (0 == _count) {
			refresh();
		}
		MM_EvacuatorWorkspace *free = NULL;
		if (0 < _count) {
			free = _head;
			_head = free->next;
			free->next = NULL;
			free->base = NULL;
			free->length = 0;
			free->offset = 0;
			_count -= 1;
		}
		Debug_MM_true(NULL != free);
		return free;
	}

	/**
	 * Add a free workspace at the head of the list. This may include workspaces taken from
	 * forge memory owned by another evacuator.
	 *
	 * @param free the workspace to add
	 */
	void
	add(MM_EvacuatorWorkspace *free)
	{
		Debug_MM_true((0 == _count) == (NULL == _head));

		free->next = _head;
		free->base = NULL;
		free->length = 0;
		free->offset = 0;
		_head = free;
		_count += 1;

		Debug_MM_true((0 == _count) == (NULL == _head));
	}

	/**
	 * Claim 0 or more free elements from system (forge) memory or C++ stack space
	 *
	 * @return the number of free elements added
	 */
	uintptr_t
	refresh()
	{
		if ((NULL == _underflow) || (0 == _count)) {
			/* allocate a new underflow chunk and link it in at head of underflow list*/
			UnderflowChunk *underflowChunk = (UnderflowChunk *)_forge->allocate(sizeof(UnderflowChunk), OMR::GC::AllocationCategory::FIXED, OMR_GET_CALLSITE());
			underflowChunk->next = _underflow;
			_underflow = underflowChunk;

			/* add free elements from underflow chunk to free list */
			MM_EvacuatorWorkspace *workspace = underflowChunk->workspace;
			MM_EvacuatorWorkspace *end = workspace + workspace_chunksize;
			while (workspace < end) {
				add(workspace);
				workspace += 1;
			}
		}

		Debug_MM_true(0 < _count);
		return _count;
	}

	void
	reload()
	{
		if (NULL != _underflow) {
			_count = 0;
			_head = NULL;
			for (UnderflowChunk *underflowChunk = _underflow; NULL != underflowChunk; underflowChunk = underflowChunk->next) {
				/* add free elements from underflow chunk to free list */
				MM_EvacuatorWorkspace *workspace = underflowChunk->workspace;
				MM_EvacuatorWorkspace *end = workspace + workspace_chunksize;
				while (workspace < end) {
					Debug_MM_true(NULL == workspace->base);
					add(workspace);
					workspace += 1;
				}
			}
		} else {
			refresh();
		}
	}

	/**
	 * Releases all underflow chunks, if any are allocated, and resets list to empty state.
	 */
	void
	reset()
	{
		/* deallocate all underflow chunks allocated from system memory (forged) */
		while (NULL != _underflow) {
			UnderflowChunk *next = _underflow->next;
			_forge->free(_underflow);
			_underflow = next;
		}
		_head = NULL;
		_count = 0;
	}

	/**
	 * Constructor.
	 */
	MM_EvacuatorFreelist(MM_Forge *forge)
		: MM_Base()
		, _head(NULL)
		, _count(0)
		, _underflow(NULL)
		, _forge(forge)
	{ }
};

/**
 * FIFO queue of workspaces. Other threads may access this list if their own queues run dry.  The
 * queue volume is modified only within critical regions guarded by a mutex declared by the evacuator
 * that owns the queue. It is exposed to other evacuator threads as a volatile value to enable them to
 * check queue volume without taking the mutex.
 */
class MM_EvacuatorWorklist : public MM_Base
{
/*
 * Data members
 */
private:
	MM_EvacuatorWorkspace *_head;	/* points to head of worklist */
	MM_EvacuatorWorkspace *_tail;	/* points to tail of worklist */
	volatile uintptr_t _volume;		/* (bytes) volume of work contained in list */

protected:
public:

/*
 * Function members
 */
private:
protected:
public:
	/**
	 * Get the volume of work from a workspace
	 *
	 * @param[in] work pointer to workspace.
	 */
	uintptr_t volume(const MM_EvacuatorWorkspace *work) { return work->length * ((0 == work->offset) ? 1 : sizeof(fomrobject_t)); }

	/**
	 * Returns a pointer to the volatile sum of the number of bytes contained in workspaces in the list
	 */
	volatile uintptr_t *volume() { return &_volume; }

	/**
	 * Peek at the workspace at the head of the list, or NULL
	 */
	const MM_EvacuatorWorkspace *peek() { return _head; }

	/**
	 * Add a workspace at the end of the list, or merge it into the tail workspace. If merged the
	 * input workspace pointer will be returned, otherwise NULL is returned.
	 *
	 * @param work the workspace to add
	 * @return work if merged into tail workspace, otherwise NULL
	 */
	MM_EvacuatorWorkspace *
	add(MM_EvacuatorWorkspace *work)
	{
		Debug_MM_true((0 == _volume) == (NULL == _head));
		Debug_MM_true((NULL == _head) == (NULL == _tail));
		Debug_MM_true((_head != _tail) || (NULL == _head) || (volume(_head) == _volume));
		Debug_MM_true((NULL != work) && (NULL != work->base) && (0 < work->length));

		VM_AtomicSupport::add(&_volume, volume(work));

		work->next = NULL;
		if (NULL != _tail) {
			/* workspaces can be merged if not split array packets and work is contiguous with tail */
			if ((0 == (_tail->offset + work->offset)) && (((uintptr_t)_tail->base + _tail->length) == (uintptr_t)work->base)) {
				_tail->length += work->length;
			} else {
				_tail = _tail->next = work;
				work = NULL;
			}
		} else {
			_head = _tail = work;
			work = NULL;
		}

		Debug_MM_true((0 == _volume) == (NULL == _head));
		Debug_MM_true((NULL == _head) == (NULL == _tail));
		Debug_MM_true((_head != _tail) || (NULL == _head) || (volume(_head) == _volume));

		/* caller should discard (merged workspace to freelist) if not null */
		return work;
	}

	/**
	 * Get the next available free workspace.
	 *
	 * @return the next workspace, if available, or NULL
	 */
	MM_EvacuatorWorkspace *
	next()
	{
		Debug_MM_true((0 == _volume) == (NULL == _head));
		Debug_MM_true((NULL == _head) == (NULL == _tail));
		Debug_MM_true((_head != _tail) || (NULL == _head) || (volume(_head) == _volume));

		MM_EvacuatorWorkspace *work = _head;
		if (NULL != work) {

			VM_AtomicSupport::subtract(&_volume, volume(work));

			if (work != _tail) {
				Debug_MM_true(NULL != work->next);
				_head = work->next;
				work->next = NULL;
			} else {
				Debug_MM_true(NULL == work->next);
				_head = _tail = NULL;
			}

			Debug_MM_true((NULL != work->base) && (0 < work->length));
		}

		Debug_MM_true((0 == _volume) == (NULL == _head));
		Debug_MM_true((NULL == _head) == (NULL == _tail));
		Debug_MM_true((_head != _tail) || (NULL == _head) || (volume(_head) == _volume));
		Debug_MM_true((NULL == work) || ((NULL != work->base) && (0 < work->length)));
		return work;
	}

	/**
	 * Just clear count and list pointers. The freelist owns the forge memory
	 * allocated for one evacuator and this worklist may contain workspaces
	 * allocated and distributed from other evacuators. Each evacuator reclaims
	 * it's allocated forge memory at the beginning of each gc cycle.
	 */
	void
	flush(MM_EvacuatorFreelist *freeList)
	{
		Debug_MM_true((0 == _volume) == (NULL == _head));
		Debug_MM_true((NULL == _head) == (NULL == _tail));

		VM_AtomicSupport::set(&_volume, 0);

		while (NULL != _head) {
			_head = freeList->flush(_head);
		}
		_tail = NULL;

		Debug_MM_true((0 == _volume) == (NULL == _head));
	}

	/**
	 * Constructor
	 */
	MM_EvacuatorWorklist()
		: MM_Base()
		, _head(NULL)
		, _tail(NULL)
		, _volume(0)
	{ }
};

#endif /* EVACUATORWORKLIST_HPP_ */
