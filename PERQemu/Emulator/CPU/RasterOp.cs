//
// RasterOp.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

using PERQemu.Memory;

namespace PERQemu.Processor
{

    public enum MulDivCommand
    {
        Off = 0,
        UnsignedDivide,
        UnsignedMultiply,
        SignedMultiply
    }

    /// <summary>
    /// Implements the PERQ's RasterOp hardware pipeline.  Works with the
    /// RasterOp microcode to feed quad words through the shifter/combiner
    /// to move rectangular regions of memory very quickly.
    /// </summary>
    public sealed class RasterOp
    {
        public RasterOp(MemoryBoard mem)
        {
            _ropShifter = new CPU.Shifter();        // Our own private Idaho
            _srcFifo = new Queue<ROpWord>(16);      // 4 quads (hardware limit)
            _destFifo = new Queue<ROpWord>(4);      // 1 quad
            _halfPipe = new ROpWord();              // 1 word, for overlap
            _memory = mem;                          // local reference
        }

        public void Reset()
        {
            _enabled = false;
            _setupDone = false;
            _state = State.Off;
            _srcFifo.Clear();
            _destFifo.Clear();

            Log.Debug(Category.RasterOp, "Reset");
        }

        public bool Enabled => _enabled;

        /// <summary>
        /// Sets the RasterOp Control Register, enabling or disabling the RasterOp
        /// hardware.  Is used to set the "phase" of the action, determine the direction
        /// of the transfer, and sync the RasterOp state clock with the Memory state.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CntlRasterOp(int value)
        {
            _latchOn = (value & 0x40) != 0;
            _extraSrcWord = (value & 0x20) != 0;
            _phase = (Phase)((value & 0x1c) >> 2);
            _enabled = (value & 0x2) != 0;
            _direction = (value & 0x1) != 0 ? Direction.RightToLeft : Direction.LeftToRight;

            if (_enabled)
            {
                // Call Setup to pre-compute masks and program the shifter.
                // If we're changing phase or returning from interrupt when
                // the datapath was switched off (rather than paused) the
                // call is safe (a no-op).
                Setup();

                // CntlRasterOp is always called in T1, two cycles ahead of
                // the Fetch4/Fetch4R which starts the source-dest-idle cycle.
                // This means state transitions happen in the next machine
                // cycle (T2), four cycles from the T2 when dest or source
                // fetch data appears on MDI.  Due to this latency, here the
                // Idle phase actually happens at the beginning of the three-
                // quad cycle, not at the end (as illustrated in Tony Duell's
                // CPU Tech Reference).  Force this by setting initial state
                // to SrcFetch.  Yeah, I know.
                _state = State.SrcFetch;
            }
            else
            {
                // The enabled bit is turned off, so set our state to Off.
                // At this point we just might be pausing for an interrupt,
                // or the transfer is complete.
                _state = State.Off;

                // If the latch bit is set we're pausing for a video interrupt;
                // otherwise, we're taking a general interrupt, or the transfer
                // is done (no way to tell).  At this point the destination
                // FIFO contains all four result words and the active Store4/4R
                // will complete regardless of our _enabled flag.  Sweet!
                if (!_latchOn)
                {
                    _setupDone = false;
                }
            }

            Log.Debug(Category.RasterOp,
                      "Ctrl:  {0} ({1:x2})\n\tPhase={2} Dir={3} XtraSrcWord={4} Latch={5}",
                      (_enabled ? "Enabled" : "Disabled"), value, _phase, _direction, _extraSrcWord, _latchOn);
        }

        /// <summary>
        /// Sets the RasterOp Width register, and clears the source and
        /// destination word FIFOs.  In the 16K CPU, the two upper bits also
        /// control the Multiply/Divide unit.
        /// </summary>
        public void WidRasterOp(int value)
        {
            _muldivInst = (MulDivCommand)((value >> 6) & 0x03);
            _widWordPosition = ((value >> 4) & 0x03);
            _widBitOffset = (value & 0xf);

            // Loading the width register clears the FIFOs
            _srcFifo.Clear();
            _destFifo.Clear();
            _halfPipe.Clear();

            Log.Debug(Category.RasterOp, "Width: MulDiv={0} WidWord={1} WidBits={2}",
                                         _muldivInst, _widWordPosition, _widBitOffset);
        }

        /// <summary>
        /// Returns the current status of the Multiply/Divide unit.  Always
        /// zero on the 4K CPU.
        /// </summary>
        public MulDivCommand MulDivInst
        {
            get { return _muldivInst; }
        }

        /// <summary>
        /// Loads the RasterOp Destination register.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void DstRasterOp(int value)
        {
            _function = (Function)(((int)_function & 0x4) | (((~value) & 0xc0) >> 6));
            _destWordPosition = (value & 0x30) >> 4;
            _destBitOffset = (value & 0xf);

            // Ugh, since the PERQ doesn't really have a NOOP, the assembler often
            // builds an instruction that does a dummy assignment to this register,
            // which means a massive amount of spurious log spewage.  This should
            // help, since the standard ucode does Wid, then Src, then Dst...
            if (_setupDone)
            {
                Log.Debug(Category.RasterOp, "Dest:  Func={0} WordPos={1} BitOffset={2}",
                                             _function, _destWordPosition, _destBitOffset);
            }
        }

        /// <summary>
        /// Loads the RasterOp Source register.  Upper bit also controls the
        /// PERQ1 power supply!
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SrcRasterOp(int value)
        {
            _function = (Function)(((int)_function & 0x3) | (((~value) & 0x40) >> 4));
            _srcWordPosition = (value & 0x30) >> 4;
            _srcBitOffset = (value & 0xf);

            Log.Debug(Category.RasterOp, "Src:   Func={0} WordPos={1} BitOffset={2}",
                                         _function, _srcWordPosition, _srcBitOffset);
        }

        /// <summary>
        /// Runs the next step in the RasterOp cycle, or a No-op if the
        /// hardware isn't enabled.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clock()
        {
            ROpWord w;

            _state = NextState();

            if (!_enabled)
            {
                return;
            }

            Log.Detail(Category.RasterOp,
                      "Clock: phase={0} state={1} Tstate={2} need1st={3} LeftOver={4}",
                      _phase, _state, _memory.TState, _srcNeedsAligned, _leftOver);

            switch (_state)
            {
                case State.Idle:

                    // Realign source FIFO leading edge if starting a new scanline
                    if (_srcNeedsAligned && _srcFifo.Count > 0)
                    {
                        ClearLeadingSrcWords();
                    }
                    break;

                case State.DestFetch:

                    // Fetch the dest word, set its mask and queue the result!
                    w = FetchNextWord();
                    w.Mask = ComputeDestWordMask(w.Index);
                    _destFifo.Enqueue(ComputeResult(w));
                    break;

                case State.SrcFetch:

                    // If we're in leftOver, need to clear the previous scanline
                    if (_leftOver)
                    {
                        ClearExtraSrcWords();
                    }

                    // Always queue up the incoming source word, but don't waste
                    // time setting its mask here -- we don't have enough info yet
                    w = FetchNextWord();
                    _srcFifo.Enqueue(w);
                    break;

                case State.Off:
                    // Nothing to do; should never happen
                    break;
            }
        }

        /// <summary>
        /// Return true if there are result words waiting to be written, regardless
        /// of our internal state.  This eliminates the "microcode bailed early" hack!
        /// </summary>
        public bool ResultReady => (_destFifo.Count > 0);

        /// <summary>
        /// Gets the next word from the result queue.  Expected to be called
        /// only if a Store4/4R cycle is currently in progress.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ushort Result()
        {
            ROpWord dest = _destFifo.Dequeue();

            Log.Detail(Category.RasterOp, "Returning result word: {0:x4}", dest.Data);

            return dest.Data;
        }

        /// <summary>
        /// Aligns and combines the Source and Destination words to produce a
        /// RasterOp result word.  Called during DestFetch as each word arrives
        /// to avoid complications from the delay in the overlapped Fetch/Store.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ROpWord ComputeResult(ROpWord dest)
        {
            Log.Detail(Category.RasterOp, "Result dest word: {0}", dest);

            #region Align Words

            //
            // The Emperor has made a critical error, and the time for our word
            // alignment has come.  Admiral Ackbar will explain the plan of attack:
            //
            // 1.   At the start of a scanline, our source FIFO is aligned and the
            //      half-pipeline register is "primed".  This means we should be
            //      guaranteed to have at least the first edge word in the FIFO;
            // 2.   If the source spans the quad but the destination doesn't (or
            //      vice versa), we have to account for the "two edges into one"
            //      problem - pull in the extra source word (source L+R -> dest B)
            //      or hold the source word one extra cycle (dest L+R <- source B);
            // 3.   In all other modes, we should be able to just pop the next word
            //      so the FIFOs move in lock step (while in the source region), he
            //      said, handwaving furiously.
            //
            // The hardware uses three PROMs to manage its state machine (RTI02),
            // source FIFO (RSC03) and destination pipeline (RDS00).  Here we distill
            // all that into a slightly convoluted set of rules to deal with all the
            // complicated edge alignment rules outlined above.
            //
            // Many Bothans died to bring us this information...
            //

            bool peek = false;
            bool pop = true;
            bool canPop = _srcFifo.Count > 0;

            // Grab new source word, or restore saved word (special case)
            ROpWord src = canPop ? _srcFifo.Peek() : _halfPipe;

            // Look at the destination word to find our edges and set the _leftOver flag
            switch (dest.Mask)
            {
                // Before first edge: return dest unmodified, don't touch source FIFO
                case CombinerFlags.DontMask:
                    return dest;

                // After second edge: return dest unmodified, pop the source word
                case CombinerFlags.Leftover:
                    if (canPop)
                    {
                        _srcFifo.Dequeue();
                        Log.Detail(Category.RasterOp, "Dropped src word: {0}", src);
                    }
                    return dest;

                // Full word (any phase): must always have a matching source word
                case CombinerFlags.FullWord:
                    break;

                // At an edge: flag the beginning or end of the update region
                // depending on transfer direction, and check for the merge case
                case CombinerFlags.LeftEdge:
                case CombinerFlags.RightEdge:
                    _leftOver = (dest.Mask == CombinerFlags.LeftEdge && _direction == Direction.RightToLeft) ||
                                (dest.Mask == CombinerFlags.RightEdge && _direction == Direction.LeftToRight);

                    // If it was hard to figure out, it should be hard to understand
                    peek = !_leftOver && _extraSrcWord && (_xOffset > 0);

                    if (_leftOver)
                    {
                        if (!canPop && _extraSrcWord)
                        {
                            src = _halfPipe;
                            pop = false;
                        }
                        else
                        {
                            pop = canPop;
                        }
                    }
                    break;

                // Both destination edges in one word
                case CombinerFlags.Both:
                    _leftOver = true;

                    if (_extraSrcWord && (_xOffset > 0))
                    {
                        peek = (_srcFifo.Count > 1);
                        Log.Debug(Category.RasterOp, "Merge {0} into Both! (phase={1}, xtra={2})", src, _phase, _extraSrcWord);
                    }
                    break;

                // Should never happen if our ROM tables are correct!
                default:
                    throw new InvalidOperationException($"RasterOp Destination result word has {dest.Mask} mask");
            }

            // Consume the current word?
            if (pop && canPop)
            {
                _srcFifo.Dequeue();
                canPop = _srcFifo.Count > 0;
            }

            // Grab the next one?
            if (peek && canPop)
            {
                src = _srcFifo.Dequeue();
            }

            Log.Detail(Category.RasterOp, "Next source word: {0}", src);

            #endregion

            #region Align Bits

            //
            // The destination word is in the update region and our source word is aligned.
            // If bit alignment is needed, feed the saved word from the half pipe to the
            // shifter with the current word.  Note that the MSB of the combined shifter
            // inputs is always the leftmost pixel in the update region (so, dependent upon
            // the direction of transfer).
            //
            ushort aligned;

            if (_xOffset != 0)
            {
                if (_direction == Direction.LeftToRight)
                {
                    Log.Detail(Category.RasterOp, "Result xtra (hi): {0:x4}", _halfPipe);
                    _ropShifter.Shift(src.Data, _halfPipe.Data);
                }
                else
                {
                    Log.Detail(Category.RasterOp, "Result xtra (lo): {0:x4}", _halfPipe);
                    _ropShifter.Shift(_halfPipe.Data, src.Data);
                }

                // Save the current source word
                _halfPipe = src;

                // Grab the aligned source word from the shifter
                aligned = _ropShifter.ShifterOutput;
            }
            else
            {
                // Already in alignment
                aligned = src.Data;
            }

            Log.Detail(Category.RasterOp, "Result aligned:  {0:x4}", aligned);

            #endregion

            #region Combine 'em

            //
            // Finally! Combine source & dest words using the appropriate mask
            //
            ushort combined;

            switch (dest.Mask)
            {
                case CombinerFlags.LeftEdge:
                    combined = Combine(dest.Data, aligned, _leftEdgeMask);
                    break;

                case CombinerFlags.RightEdge:
                    combined = Combine(dest.Data, aligned, _rightEdgeMask);
                    break;

                case CombinerFlags.Both:
                    combined = Combine(dest.Data, aligned, _bothEdgesMask);
                    break;

                case CombinerFlags.FullWord:
                    combined = Combine(dest.Data, aligned, 0xffff);
                    break;

                default:
                    combined = dest.Data;   // This can't actually happen (silence a warning)
                    break;
            }

            Log.Detail(Category.RasterOp, "Result combined: {0:x4} (func={1})", combined, _function);

            #endregion

            // For debugging, we return the ROpWord, updated with the combined
            // result. At some point when this is fully debugged, the dest FIFO
            // could be a simple queue of ushorts, which could improve efficiency.
            dest.Data = combined;
            return dest;
        }

        /// <summary>
        /// Precomputes several values for the current RasterOp call so that
        /// they don't have to be repeated during Result() calls.
        /// </summary>
        void Setup()
        {
            if (!_setupDone)
            {
                //
                // RasterOp shifter setup
                //
                // xOffset is the shift amount based on current register values;
                // set up the RasterOp shifter command.
                _xOffset = (16 + (_destBitOffset - _srcBitOffset)) & 0xf;
                _ropShifter.SetShifterCommand(CPU.ShifterCommand.Rotate, _xOffset, 0);

                //
                // Region bitmask setup
                //
                // Left edge is pretty straightforward
                _leftEdgeMask = (ushort)(0xffff >> _destBitOffset);

                // Right is more complex, because we have to take the remainder
                // of any bits that spill into the next word.  Crude?  But effective
                _rightEdgeMask = (ushort)~(0xffff >> (((_destBitOffset + _widBitOffset) & 0xf) + 1));

                // If both edges are in the same word, combine the edge masks
                _bothEdgesMask = (ushort)~(_leftEdgeMask ^ _rightEdgeMask);  // xnor 'em

                // Set flags to indicate we are outside the update region to begin
                _leftOver = false;
                _srcNeedsAligned = true;        // The Pittsburgh variable
                _setupDone = true;

                Log.Debug(Category.RasterOp, "Dest:  Func={0} WordPos={1} BitOffset={2}",
                                                   _function, _destWordPosition, _destBitOffset);
                Log.Debug(Category.RasterOp, "Setup: xOffset={0} Left={1:x4} Right={2:x4} Full={3:x4}",
                                                   _xOffset, _leftEdgeMask, _rightEdgeMask, _bothEdgesMask);
            }
        }

        /// <summary>
        /// Returns the next RasterOp state, determined by the current phase
        /// and Memory Tstate.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        State NextState()
        {
            // RasterOp is clocked after the Memory "tick", but before the
            // DispatchFunction which may (at the end of the cycle) specify a
            // phase change.  This means we can complete the current quadword
            // cycle without concern for a phase change harshing our mellow.
            int _stateClock = _memory.TState;

            State next = _state;

            // NOTE: our state change happens in T2, to coincide with MDI on
            // fetches, NOT in the T1 when CntlRasterOp() is called (or in
            // T3 when the Fetch is issued).
            switch (_state)
            {
                case State.Off:
                    // No-op.  Shouldn't ever happen.
                    break;

                case State.Idle:
                    if (_stateClock == 2)
                    {
                        if (_phase == Phase.FirstSource || _phase == Phase.XtraSource)
                        {
                            next = State.SrcFetch;
                        }
                        else
                        {
                            next = State.DestFetch;
                        }
                    }
                    break;

                case State.DestFetch:
                    if (_stateClock == 2)
                    {
                        next = State.SrcFetch;
                    }
                    break;

                case State.SrcFetch:
                    if (_stateClock == 2)
                    {
                        next = State.Idle;
                    }
                    break;

                default:
                    throw new InvalidOperationException($"Unexpected state {_state} in phase {_phase} NextState");
            }
            return next;
        }

        /// <summary>
        /// Populates and returns a ROpWord with the address, index and data
        /// of the current memory word (but throws an error if MDI is invalid).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ROpWord FetchNextWord()
        {
            var w = new ROpWord();

            if (_memory.MDIValid)
            {
                // The microcode calculates the addresses and initiates fetches;
                // we just pluck the next incoming word off the MDI.
                w.Address = _memory.MADR;
                w.Index = _memory.MIndex;
                w.Data = _memory.MDI;
                return w;
            }

#if DEBUG
            // For debugging we can try to continue, but we're pretty hosed at this point...
            Log.Error(Category.RasterOp, "FetchNextWord in {0} while MDI was invalid!", _state);
            w.Clear();
            return w;
#else
            throw new InvalidOperationException("FetchNextWord while MDI was invalid!");
#endif
        }

        /// <summary>
        /// Compute the destination mask like the ROM do.  May precompute this at
        /// startup and use the table lookup approach if it's faster (but no more
        /// need for the external file).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        CombinerFlags ComputeDestWordMask(int index)
        {
            CombinerFlags m;

            if (_direction == Direction.LeftToRight)
            {
                switch (_phase)
                {
                    case Phase.Begin:
                    case Phase.XtraSource:
                        m = (index < _destWordPosition) ? CombinerFlags.DontMask :
                            (index == _destWordPosition) ? CombinerFlags.LeftEdge :
                            CombinerFlags.FullWord;
                        break;

                    case Phase.Mid:
                    case Phase.FirstSource:
                        m = CombinerFlags.FullWord;
                        break;

                    case Phase.End:
                    case Phase.EndClear:
                        m = (index > _widWordPosition) ? CombinerFlags.Leftover :
                            (index == _widWordPosition) ? CombinerFlags.RightEdge :
                            CombinerFlags.FullWord;
                        break;

                    case Phase.BeginEnd:
                    case Phase.BeginEndClear:

                        var left = (index == _destWordPosition);
                        var right = (index == _widWordPosition);

                        if (left && right)
                            m = CombinerFlags.Both;
                        else if (left && !right)
                            m = CombinerFlags.LeftEdge;
                        else if (!left && right)
                            m = CombinerFlags.RightEdge;
                        else
                            m = (index < _destWordPosition) ? CombinerFlags.DontMask :
                                (index > _widWordPosition) ? CombinerFlags.Leftover :
                                CombinerFlags.FullWord;
                        break;

                    default:
                        throw new InvalidOperationException($"Illegal phase {_phase} in CompDWMask (LR)");
                }
            }
            else
            {
                switch (_phase)
                {
                    case Phase.Begin:
                    case Phase.XtraSource:
                        m = (index > _destWordPosition) ? CombinerFlags.DontMask :
                            (index == _destWordPosition) ? CombinerFlags.RightEdge :
                            CombinerFlags.FullWord;
                        break;

                    case Phase.Mid:
                    case Phase.FirstSource:
                        m = CombinerFlags.FullWord;
                        break;

                    case Phase.End:
                    case Phase.EndClear:
                        m = (index < _widWordPosition) ? CombinerFlags.Leftover :
                            (index == _widWordPosition) ? CombinerFlags.LeftEdge :
                            CombinerFlags.FullWord;
                        break;

                    case Phase.BeginEnd:
                    case Phase.BeginEndClear:

                        var left = (index == _widWordPosition);
                        var right = (index == _destWordPosition);

                        if (left && right)
                            m = CombinerFlags.Both;
                        else if (left && !right)
                            m = CombinerFlags.LeftEdge;
                        else if (!left && right)
                            m = CombinerFlags.RightEdge;
                        else
                            m = (index > _destWordPosition) ? CombinerFlags.DontMask :
                                (index < _widWordPosition) ? CombinerFlags.Leftover :
                                CombinerFlags.FullWord;
                        break;

                    default:
                        throw new InvalidOperationException($"Illegal phase {_phase} in CompDWMask (RL)");
                }
            }

            return m;
        }

        /// <summary>
        /// Drop extra source words outside of the update region when searching
        /// for the first edge of a new scan line.  Updates _srcNeedsAligned
        /// when found.  To avoid some overhead, is only called in Idle state
        /// if _srcNeedsAligned is true and the _srcFifo is not empty.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ClearLeadingSrcWords()
        {
            // We only need to align the first edge in a Begin phase; else no-op
            if (_phase == Phase.Begin || _phase == Phase.BeginEnd || _phase == Phase.BeginEndClear)
            {
                var w = _srcFifo.Peek();

                if (w.Index == _srcWordPosition)
                {
                    _srcNeedsAligned = false;   // Found our first edge
                    _halfPipe = w;              // Prime the half-pipeline register
                }

                if (_srcNeedsAligned)
                {
                    Log.Detail(Category.RasterOp, "--> Dropping leading word ({0})", w.Mask);
                    _srcFifo.Dequeue();
                }
            }
        }

        /// <summary>
        /// Drop extra words from the Source FIFO that are outside the update
        /// region.  Called in SrcFetch if _leftOver is true.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ClearExtraSrcWords()
        {
            // We only need to clear after the second edge in an End/Clear phase
            if (_phase == Phase.EndClear || _phase == Phase.BeginEndClear)
            {
                if (_srcFifo.Count == 0)
                {
                    // We've completed our DestFetch and used up all the source
                    // words; obviously there ain't no more to clear, so reset
                    // for the start of the next line.
                    _leftOver = false;
                    _srcNeedsAligned = true;

                    Log.Detail(Category.RasterOp, "<-- Reset for first edge");
                    return;
                }
#if DEBUG
                if (_srcFifo.Count > 8)
                {
                    Log.Warn(Category.RasterOp, "<-- Source FIFO overflow");
                }
#endif
                // We're here when there are still extra source words left
                // over from the previous scan line.  We only want to drop
                // words through the end of the current quad.
                var w = _srcFifo.Peek();

                if ((_direction == Direction.LeftToRight && w.Index == 3) ||
                    (_direction == Direction.RightToLeft && w.Index == 0))
                {
                    _leftOver = false;
                    _srcNeedsAligned = true;

                    Log.Detail(Category.RasterOp, "<-- End of scan line, reset for first edge ({0})", w.Mask);
                    _srcFifo.Dequeue();
                    return;
                }

                Log.Detail(Category.RasterOp, "--> Clearing extra word ({0})", w.Mask);
                _srcFifo.Dequeue();
            }
        }

        /// <summary>
        /// Masks and combines a source and destination word according to
        /// the current RasterOp function.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        ushort Combine(ushort dstWord, ushort srcWord, ushort mask)
        {
            switch (_function)
            {
                case Function.Insert:
                    // Nothing; leave source alone
                    break;

                case Function.InsertNot:
                    srcWord = (ushort)(~srcWord);
                    break;

                case Function.And:
                    srcWord = (ushort)(srcWord & dstWord);
                    break;

                case Function.AndNot:
                    srcWord = (ushort)((~srcWord) & dstWord);
                    break;

                case Function.Or:
                    srcWord = (ushort)(srcWord | dstWord);
                    break;

                case Function.OrNot:
                    srcWord = (ushort)((~srcWord) | dstWord);
                    break;

                case Function.Xor:
                    srcWord = (ushort)(srcWord ^ dstWord);
                    break;

                case Function.Xnor:
                    srcWord = (ushort)((srcWord & dstWord) | ((~srcWord) & (~dstWord)));
                    break;
            }

            // Return the finished destination word
            dstWord = (ushort)((dstWord & ~mask) | (srcWord & mask));

            return dstWord;
        }


        #region Debugging

#if DEBUG
        public void ShowState()
        {
            Console.WriteLine("RasterOp enabled is {0}.", _enabled);
            if (_enabled)
            {
                Console.WriteLine("\tState={0} Phase={1} Clock=T{2}",
                                 _state, _phase, _memory.TState);

                if (_setupDone)
                {
                    Console.WriteLine("\txOffset={0} srcAligned={1} LeftOver={2}",
                                     _xOffset, _srcNeedsAligned, _leftOver);
                    Console.WriteLine("\tMasks: Left={0:x4} Right={1:x4} Full={2:x4}",
                                    _leftEdgeMask, _rightEdgeMask, _bothEdgesMask);
                }
            }
        }

        public void ShowRegs()
        {
            Console.WriteLine("CtlRasterOp:  {0}\n\tPhase={1} Dir={2} XtraSrcWord={3} Latch={4}",
                             (_enabled ? "Enabled" : "Disabled"), _phase, _direction, _extraSrcWord, _latchOn);
            Console.WriteLine("SrcRasterOp:  Func={0} WordPos={1} BitOffset={2}",
                              _function, _srcWordPosition, _srcBitOffset);
            Console.WriteLine("DstRasterOp:  Func={0} WordPos={1} BitOffset={2}",
                              _function, _destWordPosition, _destBitOffset);
            Console.WriteLine("WidRasterOp:  WordPos={0} Bits={1} MulDiv={2}",
                              _widWordPosition, _widBitOffset, _muldivInst);
        }

        public void ShowFifos()
        {
            Console.WriteLine("Half pipeline register: {0}", _halfPipe);
            Console.WriteLine("Source FIFO:\n{0}", DumpFifo(_srcFifo));
            Console.WriteLine("Destination FIFO:\n{0}", DumpFifo(_destFifo));
        }

        // This is an expensive debugging aid...
        string DumpFifo(Queue<ROpWord> q)
        {
            string line = "";

            if (q.Count > 0)
            {
                var a = q.ToArray();

                for (int i = 0; i < q.Count; i++)
                {
                    line += $"{i}\t{a[i]}\n";
                }
            }
            else
            {
                line += "\t<empty>\n";
            }
            return line;
        }
#endif
        #endregion


        /// <summary>
        /// A memory word in the RasterOp datapath, augmented with debugging info
        /// (tracks the source address of a given word).
        /// </summary>
        struct ROpWord
        {
            public ROpWord(int addr, int idx, ushort val)
            {
                Address = addr;
                Index = idx;
                Data = val;
                Mask = CombinerFlags.Invalid;
            }

            public void Clear()
            {
                Address = 0;
                Index = 0;
                Data = 0;
                Mask = CombinerFlags.Invalid;
            }

            public override string ToString()
            {
                return string.Format("Addr={0:x6} Idx={1} Data={2:x4} Mask={3}",
                                     Address, Index, Data, Mask);
            }

            public int Address;
            public int Index;
            public ushort Data;
            public CombinerFlags Mask;
        }

        enum Direction
        {
            LeftToRight,
            RightToLeft
        }

        enum Function
        {
            Insert = 0,
            InsertNot,
            And,
            AndNot,
            Or,
            OrNot,
            Xor,
            Xnor
        }

        enum Phase
        {
            Begin = 0,
            Mid,
            End,
            BeginEnd,
            XtraSource,
            FirstSource,
            EndClear,
            BeginEndClear,
            Done
        }

        enum State
        {
            Idle = 0,
            DestFetch,
            SrcFetch,
            Off
        }

        [Flags]
        enum CombinerFlags
        {
            Invalid = 0x00,         // word not properly initialized (debugging)
            DontMask = 0x01,        // pass word unmodified; beginning of scan line
            LeftEdge = 0x02,        // word contains a left edge
            RightEdge = 0x04,       // word contains a right edge
            Both = 0x06,            // word contains both edges (shortcut)
            FullWord = 0x08,        // use all 16 bits
            Leftover = 0x10         // pass word; endscan line
        }

        // RasterOp state
        State _state;
        Phase _phase;
        bool _setupDone;

        // CntlRasterOp register
        bool _enabled;
        bool _latchOn;
        bool _extraSrcWord;
        Direction _direction;

        // WidRasterOp register
        int _widBitOffset;
        int _widWordPosition;
        MulDivCommand _muldivInst;

        // Src & DstRasterOp registers
        Function _function;
        int _srcBitOffset;
        int _srcWordPosition;
        int _destBitOffset;
        int _destWordPosition;

        // Bit offset between SrcX and DstX
        int _xOffset;

        // True if we're looking for the first edge (source FIFO)
        bool _srcNeedsAligned;

        // True if the second edge has been processed (dest FIFO)
        bool _leftOver;

        // Precompute bitmasks for regions that fall within one word, or left/right edges
        ushort _leftEdgeMask;
        ushort _rightEdgeMask;
        ushort _bothEdgesMask;

        // For fetching words directly, access to Tstate
        MemoryBoard _memory;

        // Our own Shifter (cheating; the hardware has only one)
        CPU.Shifter _ropShifter;

        // The "half-pipeline register"
        ROpWord _halfPipe;

        // Source words FIFO
        Queue<ROpWord> _srcFifo;

        // This does not actually exist in the real hardware.  This is sort of a
        // handwavy stand-in for the memory pipeline.  Result words are queued
        // here during the RasterOp cycle and pulled out when a Store is pending.
        Queue<ROpWord> _destFifo;
    }
}

