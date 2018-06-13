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

#ifndef EVACUATORHISTORY_HPP_
#define EVACUATORHISTORY_HPP_

class MM_EvacuatorHistory
{
/*
 * Data members
 */
public:
	/* controller selects evacuator sampling rate to produce a fixed number of epochs per gc cycle (sometimes this works out :) */
	static const uintptr_t epochs_per_cycle = 64;
	/* evacuators also report at up to 4 milestones during cycle */
	static const uintptr_t reports_per_cycle = epochs_per_cycle + 4;

	/* (rarely) if the epoch counter overflows epoch history record capacity the last history record is reused */
	typedef struct Epoch {
		uintptr_t gc;							/* sequential gc cycle number */
		uintptr_t epoch;						/* sequential epoch number */
		uint64_t duration;						/* epoch duration in microseconds */
		uint64_t survivorCopied;				/* cumulative survivor copied byte count */
		uint64_t tenureCopied;					/* cumulative tenure copied byte count */
		uint64_t scanned;						/* cumulative scanned byte count */
		uintptr_t stalled;						/* instantaneous sample of stalled evacuator count */
		uintptr_t survivorAllocationCeiling;	/* upper limit on TLH allocation size for survivor */
		uintptr_t tenureAllocationCeiling;		/* upper limit on TLH allocation size for tenure */
	} Epoch;

protected:

private:
	uintptr_t _epoch;					/* current epoch (incomplete if not at end of scan cycle) */
	Epoch _history[reports_per_cycle];	/* epochal record spanning one gc cycle */

/*
 * Function members
 */
private:
	/* overflow epochs are recorded at (replace) the last history record */
	uintptr_t epochToIndex(uintptr_t epoch) { return (epoch < reports_per_cycle) ? epoch : (reports_per_cycle - 1); }
	uintptr_t epochToIndex() { return epochToIndex(_epoch); }

protected:

public:
	/* number of committed epochs per this gc cyle */
	uintptr_t count() { return _epoch; }

	/* get a past (committed) epoch */
	Epoch *
	epoch(uintptr_t epoch)
	{
		Debug_MM_true((epoch < _epoch) || (0 == epoch));
		return &_history[epochToIndex(epoch)];
	}

	/* get the most recently committed epoch */
	Epoch *epoch() { return epoch((0 < _epoch) ? (_epoch - 1) : 0); }

	/* reserve tail of historic record to receive stats for closing epoch */
	Epoch *
	add()
	{
		uintptr_t next = VM_AtomicSupport::add(&_epoch, 1) - 1;
		Epoch *epoch = &_history[epochToIndex(next)];
		epoch->epoch = next;
		return epoch;
	}

	/* clear history for starting a gc cycle */
	void
	reset(uintptr_t gc = 0, uintptr_t survivorAllocationCeiling = 0, uintptr_t tenureAllocationCeiling = 0)
	{
		for (uintptr_t i = 0; i < reports_per_cycle; i += 1) {
			_history[i].gc = 0;
			_history[i].epoch = 0;
			_history[i].duration = 0;
			_history[i].survivorCopied = 0;
			_history[i].tenureCopied = 0;
			_history[i].scanned = 0;
			_history[i].stalled = 0;
			_history[i].survivorAllocationCeiling = 0;
			_history[i].tenureAllocationCeiling = 0;
		}
		_history[0].survivorAllocationCeiling = survivorAllocationCeiling;
		_history[0].tenureAllocationCeiling = tenureAllocationCeiling;
		_history[0].gc = gc;
		_epoch = 0;
	}

	MM_EvacuatorHistory()
	{
		reset();
	}
};

#endif /* EVACUATORHISTORY_HPP_ */
