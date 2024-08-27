//
// MicropolisDiskController.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
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

using PERQmedia;
using PERQemu.Config;
using PERQemu.Processor;

namespace PERQemu.IO.DiskDevices
{
    /// <summary>
    /// Represents a Micropolis 8" hard drive controller which manages disk
    /// drives in the Disk8Inch class.  This is implemented in the PERQ as an
    /// adapter from the SA4000 interface to the Micropolis 1200-series drives.
    /// This version of the driver works with the EIO board.
    /// </summary>
    /// <remarks>
    /// This class manages the PERQ side of the interface, with the actual disk
    /// device attached to the Disk Interface Board (on the other end of the
    /// "nibble bus").  See Docs/HardDisk.txt for details about the issues and
    /// limitations surrounding development of this driver.
    /// </remarks>
    public sealed class MicropolisDiskController : IStorageController
    {
        public MicropolisDiskController(PERQSystem system)
        {
            _system = system;

            _dib = new DiskInterfaceBoard(this);
            _registerFile = new byte[16];
        }

        /// <summary>
        /// Perform a "hardware reset" of the controller and drive.
        /// </summary>
        public void Reset()
        {
            _dib.Reset();
            ResetFlags();

            Log.Info(Category.HardDisk, "Micropolis controller reset");
        }

        /// <summary>
        /// Resets the flags ("soft" reset under microcode control).
        /// </summary>
        void ResetFlags()
        {
            // Figure out what stuff gets reset, but assume everything?
            if (_busyEvent != null)
            {
                _system.Scheduler.Cancel(_busyEvent);
                _busyEvent = null;
            }

            _regSelect = 0;

            _flags = SMFlags.None;
            _status = SMStatus.Idle;
            _command = SMCommand.Idle;

            // Turn off interrupts
            SetInterrupt(false);
        }

        public void AttachDrive(uint unit, StorageDevice dev)
        {
            _dib.AttachDrive(unit, dev);
        }

        #region Registers and Status

        /// <summary>
        /// Reads the hard disk state machine status register.
        /// </summary>
        public int ReadStatus()
        {
            // Reset the status word from the DIB and add state machine code
            var status = _dib.Status.Current | (int)_status;

            /*
            // Set the SMSTATUS<3> bit if:
            //      mid-cycle is enabled,
            //      an error status is being returned
            //      block r/w/fmt command is complete
            // Do NOT set on seeks or other commands!
            if (_command != SMCommand.Unused &&
                _command != SMCommand.FixPH &&
                _command != SMCommand.Idle)
            {
                status |= (int)SMStatus.SMInt;
            }
            */

            // Reading clears interrupts
            SetInterrupt(false);

            Log.Debug(Category.HardDisk, "Micropolis status: 0x{0:x3}", status);
            return status;
        }

        /// <summary>
        /// Dispatch register writes.
        /// </summary>
        public void LoadRegister(byte address, int value)
        {
            switch (address)
            {
                case 0xd0:      // LD DF CTRL L
                    //
                    // Selects the address of next data register to load.
                    //
                    _regSelect = (value & 0xf);
                    Log.Detail(Category.HardDisk, "Micropolis register file pointer now {0}", _regSelect);
                    break;

                case 0xd1:      // LD DF DATA L
                    //
                    // Loads byte into the register file and increments the
                    // address pointer.  Data is inverted.
                    //
                    if (_regSelect > _registerFile.Length - 1)
                    {
                        Log.Warn(Category.HardDisk, "Register pointer overflow!");
                        _regSelect = 0;
                    }

                    Log.Detail(Category.HardDisk, "Micropolis register file[{0}]=0x{1:x2}",
                                                  _regSelect, (byte)~value);
                    _registerFile[_regSelect++] = (byte)~value;
                    break;

                case 0xd2:      // LD D CTRL 1 (SMCTL)
                    //
                    // Latches T2, BUSENB, T, DISK_INT_ENB, P_RESET_L, F<2:0>
                    // into the command register.
                    //
                    LoadCommandRegister(value);
                    break;

                case 0xd3:      // LD D CTRL 2 (DSKCTL)
                    //
                    // Latches DRIVE_SEL<1:0>, BA<1:0>, B<3:0> into the DIB.
                    //
                    _dib.WriteNibble((byte)value);
                    break;

                default:
                    throw new UnhandledIORequestException(address, value);
            }
        }

        /// <summary>
        /// Loads the controller command register and dispatches the appropriate
        /// command to the state machine (bits 2:0 of the control reg 0xc1).  The
        /// upper five bits are flags or condition codes.  This register directly
        /// affects the execution of the state machine microprogram.
        /// </summary>
        void LoadCommandRegister(int data)
        {
            // Unpack
            _flags = (SMFlags)(data & 0xf8);
            _command = (SMCommand)(data & 0x07);
            _dib.BusEnable = (_flags.HasFlag(SMFlags.BusEnable));

            Log.Detail(Category.HardDisk,
                      "Micropolis state control: 0x{0:x2} (command {1}, flags {2})",
                      data, _command, _flags);

            // Check the enable flag regardless of current command/state/status
            // to see if we should reset (assuming that clears up any issues)
            if (!_flags.HasFlag(SMFlags.Enable))
            {
                Log.Debug(Category.HardDisk, "Controller disabled!"); // debug

                // Assume we just bail out here?
                ResetFlags();
                return;
            }

            // If we're in an error state, ignore commands until Idle is forced
            if (_command != SMCommand.Idle && (_status > SMStatus.Busy && _status < SMStatus.SMInt))
            {
                // Todo: clearing SMCTL<4> is supposed to reset this too?
                Log.Debug(Category.HardDisk, "Command {0} ignored while in state {1}", _command, _status);
                return;
            }

            switch (_command)
            {
                case SMCommand.Idle:
                    _status = SMStatus.Idle;
                    break;

                case SMCommand.Read:
                case SMCommand.ReadChk:
                    _status = SMStatus.Busy;
                    ReadBlock();
                    break;

                case SMCommand.Write:
                case SMCommand.WriteChk:
                    _status = SMStatus.Busy;
                    WriteBlock();
                    break;

                default:
                    Log.Warn(Category.HardDisk, "Command {0} unknown or not yet implemented", _command);
                    break;
            }
        }


        /// <summary>
        /// Single step pulse (used by Shugart, not by Micropolis).
        /// </summary>
        public void DoSingleSeek()
        {
            // This means that the Z80 tried to step the heads!?!?
            Log.Warn(Category.HardDisk, "Micropolis single seek step ignored!");
        }

        /// <summary>
        /// Called by the DIB when a seek/restore is started.  Otherwise we'd have
        /// to possibly have the (fake) disk state machine poke through the command
        /// nibble to pick out the Restore flag from the data sent by the microcode
        /// and that state is all in the DIB itself now.  Meh.  Six of one...
        /// </summary>
        void SetBusy()
        {
            Log.Detail(Category.HardDisk, "DIB signaled seek/restore in progress");
            _status = SMStatus.Busy;
        }

        /// <summary>
        /// Called by the DIB when a status change requires an interrupt.
        /// </summary>
        void StatusChange()
        {
            Log.Detail(Category.HardDisk, "DIB signaled status change!");
            SetInterrupt(true);
        }

        #endregion

        #region Block Operations

        /// <summary>
        /// Extra sanity checking (for debugging) to make sure the register file,
        /// DIB and disk are all in sync.  Always returns true in release builds
        /// since the fake disk always returns the requested cyl/head/sec. :-)
        /// </summary>
        bool CheckBlockParameters(ushort cyl, byte head)
        {
#if DEBUG
            if (_dib.SelectedDrive == null)
                throw new InvalidOperationException($"{_command} but no disk selected");

            // Do this here (until debugged)
            if (_dib.SelectedDrive.CurHead != _dib.Head || _dib.Head != head)
            {
                Log.Warn(Category.HardDisk, "Wrong head selected: disk={0} DIB={1} regs={2}",
                                            _dib.SelectedDrive.CurHead, _dib.Head, head);
                return false;
            }

            if (_dib.SelectedDrive.CurCylinder != _dib.Cylinder || _dib.Cylinder != cyl)
            {
                Log.Warn(Category.HardDisk, "Cylinder out of sync: disk={0} DIB={1} regs={2}",
                                            _dib.SelectedDrive.CurCylinder, _dib.Cylinder, cyl);
                return false;
            }
#endif
            return true;
        }

        /// <summary>
        /// Check the logical header bytes against the register file.
        /// </summary>
        bool CheckLogicalHeaderBytes(byte[] LH)
        {
#if DEBUG
            // Check the header bytes against the register file values!  If
            // there are any discrepancies, report the error and abort due to
            // a logical header mismatch.  Realism baybeeee!!!  ALSO, this is
            // only an error condition if the T2 bit isn't set, right? :-)
            if ((LH[0] != _registerFile[5]) ||
                (LH[1] != _registerFile[9]) ||
                (LH[2] != _registerFile[6]) ||
                (LH[3] != _registerFile[10]) ||
                (LH[4] != _registerFile[7]) ||
                (LH[5] != _registerFile[11]))
            {
                Console.Write("LH mismatch: blk: ");
                for (int i = 0; i < 6; i++)
                {
                    Console.Write($"  {i}={LH[i]:x2}");
                }
                Console.Write("\n   register file:  ");
                for (int i = 5; i < 8; i++)
                {
                    Console.Write($"  {i}={_registerFile[i]:x2}  {i + 4}={_registerFile[i + 4]:x2}");
                }
                Console.WriteLine();
                return false;
            }
#endif
            return true;
        }

        /// <summary>
        /// Reads a block from the cyl/head/sec specified by the register file
        /// into memory at the address specified by the DMA controller.  Sets
        /// status and schedules the appropriate interrupt(s) to fire based on
        /// the read result.
        /// </summary>
        void ReadBlock()
        {
            var cyl = (ushort)((_registerFile[8] & 0xf0) << 4 | _registerFile[4]);
            var head = (byte)(_registerFile[8] & 0x0f);
            var sector = _registerFile[3];

            // Compute the delay to the start of the sector (or fake it if we
            // aren't modeling accurate disk timings.  We'll say that 100usec
            // is a reasonable minimum to the mid-sector point in case we bug
            // out on a header error (or fire the DB interrupt on a good read)

            // Fast...
            var delay = 100 * Conversion.UsecToNsec;

            // ...or accurate?
            if (Settings.Performance.HasFlag(RateLimit.DiskSpeed))
            {
                delay = _dib.SelectedDrive.ComputeRotationalDelay(_system.Scheduler.CurrentTimeNsec, sector);
            }

            // Sanity checks
            if (!CheckBlockParameters(cyl, head))
            {
                FinishCommand(delay, SMStatus.SMError);
                return;
            }

            // Read the sector from the disk
            var block = _dib.SelectedDrive.GetSector(cyl, head, sector);

            // Fetch the unfrobbed buffer addresses
            var data = _system.IOB.DMARegisters.GetDataAddress(ChannelName.HardDisk);
            var header = _system.IOB.DMARegisters.GetHeaderAddress(ChannelName.HardDisk);

#if DEBUG
            // This will always be 2?  Unless non-check reads don't xfer the header?  Hmm.
            var quads = _system.IOB.DMARegisters.GetHeaderCount(ChannelName.HardDisk);

            if (quads != 2)
            {
                Log.Warn(Category.HardDisk, "Bad DMA header count {0} on {1}", quads, _command);
            }
#endif

            // Copy the header to the header address
            for (int i = 0; i < block.Header.Length; i += 2)
            {
                int word = block.Header[i] | (block.Header[i + 1] << 8);
                _system.Memory.StoreWord(header + (i >> 1), (ushort)word);
            }

            // On a ReadChk, verify that the LH matches the registers
            if (_command == SMCommand.ReadChk)
            {
                if (!CheckLogicalHeaderBytes(block.Header))
                {
                    Log.Warn(Category.HardDisk,
                             "Logical header mismatch on read from {0}/{1}/{2}",
                             cyl, head, sector);

                    FinishCommand(delay, SMStatus.LHMismatch);
                    return;
                }
            }

            // Copy the data to the data buffer address
            for (int i = 0; i < block.Data.Length; i += 2)
            {
                int word = block.Data[i] | (block.Data[i + 1] << 8);
                _system.Memory.StoreWord(data + (i >> 1), (ushort)word);
            }

            Log.Debug(Category.HardDisk,
                      "Micropolis sector read from {0}/{1}/{2}, to memory at 0x{3:x6}",
                      cyl, head, sector, data);

            // Are we firing the mid-sector interrupt?
            if (_flags.HasFlag(SMFlags.MidIntDisable))
            {
                // Nope, exit through the gift shop
                FinishCommand(delay + BlockDelayNsec, SMStatus.Idle);
            }
            else
            {
                // Schedule the mid-cycle interrupt (which will then finish)
                MidSectorFinish(delay, SMStatus.Idle);
            }
        }

        /// <summary>
        /// Does a write to the cyl/head/sec specified by the controller registers.
        /// Same basic steps as Read/ReadCheck, but the other way around.
        /// </summary>
        void WriteBlock()
        {
            var cyl = (ushort)((_registerFile[8] & 0xf0) << 4 | _registerFile[4]);
            var head = (byte)(_registerFile[8] & 0x0f);
            var sector = _registerFile[3];
            var delay = 100 * Conversion.UsecToNsec;

            if (Settings.Performance.HasFlag(RateLimit.DiskSpeed))
            {
                delay = _dib.SelectedDrive.ComputeRotationalDelay(_system.Scheduler.CurrentTimeNsec, sector);
            }

            if (!CheckBlockParameters(cyl, head))
            {
                FinishCommand(delay, SMStatus.SMError);
                return;
            }

            var block = _dib.SelectedDrive.GetSector(cyl, head, sector);

            var data = _system.IOB.DMARegisters.GetDataAddress(ChannelName.HardDisk);
            var header = _system.IOB.DMARegisters.GetHeaderAddress(ChannelName.HardDisk);

            for (int i = 0; i < block.Data.Length; i += 2)
            {
                int word = _system.Memory.FetchWord(data + (i >> 1));
                block.Data[i] = (byte)(word & 0xff);
                block.Data[i + 1] = (byte)(word >> 8);
            }

            for (int i = 0; i < block.Header.Length; i += 2)
            {
                int word = _system.Memory.FetchWord(header + (i >> 1));
                block.Header[i] = (byte)(word & 0xff);
                block.Header[i + 1] = (byte)(word >> 8);
            }

            _dib.SelectedDrive.SetSector(block);

            Log.Debug(Category.HardDisk,
                      "Micropolis sector write complete to {0}/{1}/{2}, from memory at 0x{3:x6}",
                      cyl, head, sector, data);

            if (_flags.HasFlag(SMFlags.MidIntDisable))
            {
                FinishCommand(delay + BlockDelayNsec, SMStatus.Idle);
            }
            else
            {
                MidSectorFinish(delay, SMStatus.Idle);
            }
        }

        /// <summary>
        /// Does a "format" of the cyl/head/sec specified by the controller
        /// registers.  Does NOT commit to disk, only in memory copy is affected.
        /// </summary>
        void FormatBlock()
        {
            // Todo: think about how to support spare sectoring and bad block
            // maps - the PERQ allows rewriting the sector header to effect a
            // block sparing, so... any changes to the underlying storage dev?
            // split the phys/log header fields if they aren't already?

            // some format parameters come from the register file, so figure out
            // how that works
            var _sector = _registerFile[3];
            var sec = new Sector(_dib.Cylinder, _dib.Head, _sector,
                                 _dib.SelectedDrive.Geometry.SectorSize,
                                 _dib.SelectedDrive.Geometry.HeaderSize);

            var data = _system.IOB.DMARegisters.GetDataAddress(ChannelName.HardDisk);
            var header = _system.IOB.DMARegisters.GetHeaderAddress(ChannelName.HardDisk);

            // For Microp I think these get synthesized by the DIB or drive?
            for (int i = 0; i < sec.Data.Length; i += 2)
            {
                int word = _system.Memory.FetchWord(data + (i >> 1));
                sec.Data[i] = (byte)(word & 0xff);
                sec.Data[i + 1] = (byte)(word >> 8);
            }

            // Write the new header data...
            for (int i = 0; i < sec.Header.Length; i += 2)
            {
                int word = _system.Memory.FetchWord(header + (i >> 1));
                sec.Header[i] = (byte)(word & 0xff);
                sec.Header[i + 1] = (byte)(word >> 8);
            }

            // Write the sector to the disk...
            _dib.SelectedDrive.SetSector(sec);

            Log.Write(Category.HardDisk,
                      "Micropolis sector format of {0}/{1}/{2} complete, from memory at 0x{3:x6}",
                      _dib.Cylinder, _dib.Head, _sector, data);

            //SetBusyState(false, true);
        }

        #endregion

        #region Completion and Interrupts

        /// <summary>
        /// Schedule the mid-sector interrupt, and if not in error, proceed to
        /// complete the command normally.
        /// </summary>
        void MidSectorFinish(ulong delay, SMStatus exitCode)
        {
#if DEBUG
            if (_busyEvent != null)
                Log.Error(Category.HardDisk, "MidSectorFinish called while already busy");
            // but proceed anyway?
#endif
            if (exitCode == SMStatus.Idle)
            {
                Log.Debug(Category.HardDisk, "Firing mid-sector in {0}ms", delay * Conversion.NsecToMsec);

                _busyEvent = _system.Scheduler.Schedule(delay, (skewNsec, context) =>
                {
                    Log.Debug(Category.HardDisk, "Mid-sector interrupt firing, code={0}", exitCode);

                    // Per disk.doc, mid-sector int sets SMInt, low 3 bits to 1
                    _status = (SMStatus.SMInt | SMStatus.Busy);

                    // Fire mid-sector "DB" interrupt
                    SetInterrupt(true, true);

                    // Schedule a normal completion
                    FinishCommand(BlockDelayNsec, exitCode);
                });
            }
            else
            {
                _busyEvent = _system.Scheduler.Schedule(delay, (skewNsec, context) =>
                {
                    // Error out through FinishCommand()
                    FinishCommand(delay, exitCode);
                });
            }
        }

        /// <summary>
        /// Schedule the completion interrupt to fire.
        /// </summary>
        void FinishCommand(ulong delay, SMStatus exitCode)
        {
            _busyEvent = _system.Scheduler.Schedule(delay, (skewNsec, context) =>
            {
                Log.Debug(Category.HardDisk, "Command completion: {0}", exitCode);
                _status = (SMStatus.SMInt | exitCode);
                _busyEvent = null;

                SetInterrupt(true);
            });
        }

        /// <summary>
        /// Raise or clear the disk interrupt, depending on the caller's request
        /// and the state of the enable flags.  No-ops if no change is required.
        /// </summary>
        /// <remarks>
        /// There are five possible interrupts from the state machine:
        ///     * any transition of the Ready line;
        ///     * OnCyl going low (i.e., seek complete);
        ///     * the mid-sector interrupt;
        ///     * end of the sector interrupt (completion).
        /// If the header is in error, the command aborts and the completion
        /// interrupt is skipped.  All interrupts are cleared by reading the
        /// status register.
        /// </remarks>
        void SetInterrupt(bool raiseInterrupt, bool mid = false)
        {
            // If interrupts are disabled, force clear
            raiseInterrupt &= _flags.HasFlag(SMFlags.InterruptsOn);

            if (raiseInterrupt && !_interrupting)
            {
                // Extra check for mid-sector interrupt enabled
                if (!mid || (mid && !_flags.HasFlag(SMFlags.MidIntDisable)))
                {
                    _system.CPU.RaiseInterrupt(InterruptSource.HardDisk);
                    _interrupting = true;
                }
            }
            else if (!raiseInterrupt && _interrupting)
            {
                _system.CPU.ClearInterrupt(InterruptSource.HardDisk);
                _interrupting = false;
            }
        }

        #endregion

        // Debugging
        public void DumpStatus()
        {
            // Pretty print the register names since they contain useful stuff
            // In the Micropolis controller, only 11 of 16 are ever used
            string[] RN = { "Zero", "Sync", "Unused", "Sector", "CylLo",
                            "LH1lo", "LH2lo", "LH3lo", "Cyl/Hd",
                            "LH1hi", "LH2hi", "LH3hi" };
            string[] regs = new string[12];

            for (int i = 0; i < regs.Length; i++)
            {
                regs[i] = $"{i:d2} {RN[i]}=0x{_registerFile[i]:x2}";
            }

            Console.WriteLine("Micropolis driver status:");
            Console.WriteLine($"  SM command: {_command} (flags {_flags})");
            Console.WriteLine($"  SM status:  {_status}  Interrupt: {_interrupting}");

            Console.WriteLine($"Register file (pointer {_regSelect}):");
            PERQemu.CLI.Columnify(regs, 2, 16);
            Console.WriteLine();

            _dib.DumpDIBStatus();
        }

        /// <summary>
        /// Status bits from the state machine as of NDsk7.Miasma/EIODisk.Micro.
        /// This should work with the EIO based on POS G.6 and Accent S6 sources;
        /// the names of the entries agree, even if the Accent microcode seems to
        /// reverse codes 5 & 6 (and gives them all slightly different names).
        /// See Docs/HardDisk.txt for more.
        /// </summary>
        [Flags]
        enum SMStatus
        {                           // Accent eiodisk.mic:
            Idle = 0,               // DskDone
            Busy = 1,               // DskLHFound
            DataCRC = 2,            // DskDataErr
            PHMismatch = 3,         // DskPHMismatch
            LHMismatch = 4,         // DskLHMismatch
            HeadCRC = 5,            // DskBadCmd
            AbnormalError = 6,      // DskPHLHCRC
            SMError = 7,            // DskECCandCRC
            SMInt = 8               // State machine interrupt flag SMSTATUS<3>
        }

        /// <summary>
        /// Disk command bits [2:0] go to the hard disk state machine.  Again,
        /// there are way too many conflicting and contradictory sources and no
        /// single, definitive command set, so this list is the best guess and
        /// will have to be tested against each supported OS.
        /// </summary>
        enum SMCommand
        {
            Idle = 0,               // No-op (clear command)
            Format = 1,             // Format Write
            Write = 2,              // Write Data - Write Header
            WriteChk = 3,           // Write Data - Check Header
            FixPH = 4,              // Fix physical header (Feb83)
            Read = 5,               // Read Data - Read Header
            Unused = 6,             // Unused (Feb83)
            ReadChk = 7             // Read Data - Check Header
        }

        /// <summary>
        /// High bits of the command register are condition flags interpreted by
        /// the state machine and/or directly by the hardware.
        /// </summary>
        [Flags]
        enum SMFlags
        {
            None = 0x0,
            Enable = 0x08,          // Reset when L
            InterruptsOn = 0x10,    // Disk Interrupt Enable when H
            MidIntDisable = 0x20,   // T bit: Disable mid-cycle interrupts when H
            BusEnable = 0x40,       // BusEnable to drive when H
            SkipCRC = 0x80          // T2 bit: pass on CRC errors when H
        }

        /// <summary>
        /// Bit positions of the hardware flags from the DIB (SMStatus word).
        /// </summary>
        [Flags]
        public enum HWFlags
        {
            SeekError = 0x010,
            Fault = 0x020,
            OnCyl = 0x040,
            Ready = 0x080,
            Index = 0x100
        }

        // Work timing for reads/writes, assuming the interface's documented
        // 5.64Mbit/sec (705KB/sec) max transfer rate (MFM, not GCR encoding)
        // transferring 528-byte sectors.  See ShugartController.cs for more
        // info about how this is derived.
        readonly ulong BlockDelayNsec = 749 * Conversion.UsecToNsec;

        // Registers
        SMFlags _flags;
        SMCommand _command;
        SMStatus _status;

        byte[] _registerFile;
        int _regSelect;

        bool _interrupting;

        // The physical disk and controller
        DiskInterfaceBoard _dib;

        SchedulerEvent _busyEvent;
        PERQSystem _system;


        /// <summary>
        /// Emulates the Micropolis DIB (ICL part #5155480?).  This is the 4-bit
        /// nibble bus interposer board that converts Shugart 50-pin signaling from
        /// the EIO to the "raw" Micropolis 1200-series 50-pin drive interface.  It
        /// is NOT the M1220 intelligent controller.  See Docs/HardDisk.txt for more.
        /// </summary>
        internal class DiskInterfaceBoard
        {
            public DiskInterfaceBoard(MicropolisDiskController parent)
            {
                _control = parent;
                _status = new HWStatus();
                _status.DriveType = (int)DeviceType.Unused;      // until loaded
                _disk = null;
            }

            public HardDisk SelectedDrive => _disk;
            public HWStatus Status => _status;

            public byte Unit => _driveSelect;
            public ushort Cylinder => _cylinder;
            public byte Head => _head;

            public bool BusEnable
            {
                get { return _busEnable; }
                set { _busEnable = value; CheckNibble(); }
            }

            public void Reset()
            {
                _disk?.Reset();

                _command = M1200Command.None;
                _busEnable = false;
                _latchedBusEnable = false;
                _latchedNibble = 0;
                _latchedData = 0;
                _driveSelect = 0;
                _cylinder = 0;
                _head = 0;

                Log.Info(Category.HardDisk, "Micropolis DIB reset");
            }

            /// <summary>
            /// Attach a drive.  For now, only a single unit is supported.
            /// </summary>
            public void AttachDrive(uint unit, StorageDevice dev)
            {
                if (_disk != null)
                    throw new InvalidOperationException("MicropolisController only supports 1 disk");

                if (dev.Info.Type != DeviceType.Disk8Inch && dev.Info.Type != DeviceType.DCIOMicrop)
                    throw new InvalidConfigurationException($"Device type {dev.Info.Type} not supported by this controller");

                _disk = dev as HardDisk;
                _disk.SetSeekCompleteCallback(SeekCompletionCallback);

                // A small detail
                _status.DriveType = (int)DeviceType.Disk8Inch;

                Log.Info(Category.HardDisk, "Attached disk '{0}'", _disk.Info.Name);
            }

            /// <summary>
            /// Accept writes from the EIO over the "nibble bus".  Uses the current
            /// state of the BusEnable pin to determine if a high or low half of the
            /// selected register is to be updated, then latches that state.
            /// </summary>
            public void WriteNibble(byte val)
            {
                var unit = _driveSelect;

                // Split the byte into its parts:
                _driveSelect = (byte)((val & 0xc0) >> 6);
                _regSelect = (RegSelect)(val & 0x30);
                var B = (byte)(val & 0x0f);

                Log.Debug(Category.HardDisk,
                          "Micropolis disk control: 0x{0:x2} (unit {1}, reg {2}, nib 0x{3:x})",
                          val, _driveSelect, _regSelect, B);

                if (_regSelect == RegSelect.Latch)
                {
                    _latchedNibble = B;
                    Log.Detail(Category.HardDisk, "DIB: Latched nibble 0x{0:x}", B);
                }

                // Signal Ready change if the selected drive changed
                if (unit != _driveSelect) UpdateStatus();
            }

            /// <summary>
            /// Simulate the drive side of the nibble bus, taking appropriate action on
            /// transitions of the BusEnable signal.  This is the DIB's implementation
            /// of the Micropolis 1220 controller as it talks to the "raw" 120x drives.
            /// </summary>
            void CheckNibble()
            {
                // Check for BusEnable transitions
                var busEnRising = (!_latchedBusEnable && _busEnable);    // Rising edge this cycle
                var busEnFalling = (_latchedBusEnable && !_busEnable);   // Falling edge

                _latchedBusEnable = _busEnable;

                switch (_regSelect)
                {
                    case RegSelect.CylReg:
                        if (busEnFalling)
                        {
                            // Assign the low byte as Cyl<7:0>
                            _cylinder = _latchedData;
                            Log.Debug(Category.HardDisk, "DIB: Cylinder (lo) = {0}", _cylinder);
                        }
                        break;

                    case RegSelect.HdCylReg:
                        if (busEnFalling)
                        {
                            // Add high nibble as Cyl<11:8>
                            _cylinder |= (ushort)((_latchedData & 0xf0) << 4);

                            // Set the head and initiate a seek (and/or head select)
                            _head = (byte)(_latchedData & 0x0f);
                            Log.Debug(Category.HardDisk, "DIB: Cylinder = {0}, Head = {1}", _cylinder, _head);

                            // Start the seek!
                            DoSeek();
                        }
                        break;

                    case RegSelect.CtrlReg:
                        if (busEnFalling)
                        {
                            _command = (M1200Command)_latchedData;
                            if (_command != M1200Command.None)
                            {
                                Log.Debug(Category.HardDisk, "DIB: Command is {0}", _command);
                            }

                            // Only two command bits are significant; it's not clear if
                            // both may be issued at once, but since we don't emulate
                            // drive faults anyway it doesn't matter. :-)  Technically
                            // these must be pulsed true for 250ns min to initiate.

                            // Fault clear?  Then fall through?
                            if (_command.HasFlag(M1200Command.FaultReset))
                            {
                                _disk.FaultClear();
                            }

                            // Restore the heads?
                            if (_command.HasFlag(M1200Command.Restore))
                            {
                                DoRestore();
                            }
                        }
                        break;

                    case RegSelect.Latch:
                        if (busEnRising)
                        {
                            _latchedData = (byte)(_latchedNibble << 4);
                            Log.Detail(Category.HardDisk, "DIB: Latched hi data 0x{0:x}", _latchedNibble);
                        }
                        else if (busEnFalling)
                        {
                            _latchedData |= (byte)(_latchedNibble & 0xf);
                            Log.Detail(Category.HardDisk, "DIB: Latched lo data 0x{0:x}", _latchedNibble);
                        }
                        break;
                }
            }

            /// <summary>
            /// Seek to track 0 (recalibrate).
            /// </summary>
            void DoRestore()
            {
                _cylinder = 0;
                DoSeek();
            }

            /// <summary>
            /// Initiate a head select and a seek, if necessary.
            /// </summary>
            void DoSeek()
            {
                if (_cylinder > _disk.Geometry.Cylinders - 1 || _head > _disk.Geometry.Heads - 1)
                {
                    Log.Warn(Category.HardDisk, "Bad head or cylinder on seek!");
                    _status.SeekError = true;
                    SeekCompletionCallback(0U, null);
                    return;
                }

                // Force the OnCyl line to transition, reset any seek error flag,
                // and tell the state machine it's busy (in case the ucode checks)
                _status.OnCylinder = false;
                _status.SeekError = false;

                _control.SetBusy();

                // Do the head select
                if (_head != _disk.CurHead)
                {
                    _disk.HeadSelect(_head);
                }

                // Do the seek, even if the distance is zero cyls -- let the hard
                // disk fire the completion (with min delay) normally to reset OnCyl
                Log.Debug(Category.HardDisk, "DIB initiating seek ({0} cyls)", _cylinder - _disk.CurCylinder);
                _disk.SeekTo(_cylinder);
            }

            /// <summary>
            // Seek completion for the Micropolis.
            /// </summary>
            public void SeekCompletionCallback(ulong skewNsec, object context)
            {
                // Update hardware status
                _status.OnCylinder = (_disk.CurCylinder == _cylinder);
                _status.SeekError = !_status.OnCylinder;

                // Report OnCylinder change / seek completion
                UpdateStatus();
            }

            /// <summary>
            /// Update the status flags that come from the "hardware" and alert
            /// the controller that something has changed.  This is not terribly
            /// efficient.
            /// </summary>
            public void UpdateStatus()
            {
                _status.UnitReady = (_driveSelect == 0 && _disk.Ready);
                _status.Index = _disk.Index;
                _status.DriveFault = _disk.Fault;

                Log.Debug(Category.HardDisk, "DIB status change: 0x{0:x3}", _status.Current);
                Log.Detail(Category.HardDisk, "DIB {0}", _status);      // HW status string

                // Ready changes, faults, or OnCylinder asserted trigger an interrupt
                _control.StatusChange();
            }

            /// <summary>
            /// Hardware status bits passed through from the drive (or the DIB) and
            /// merged by the controller into the status word returned to the PERQ.
            /// For my sanity, the active low bits are returned as such for the
            /// microcode's interpretation but are positive logic here for ease in
            /// debugging.  "NotTrk0orNotSkEr"?  Seriously.  Result is pre-shifted
            /// left by four bits so it can be merged directly.
            /// </summary>
            public struct HWStatus
            {
                public int DriveType;       // <300> 01=Undefined, 11=Micropolis
                public bool Index;          // <100> from drive
                public bool UnitReady;      // <080> aka DskFault or NotFault
                public bool OnCylinder;     // <040> aka DskOnCyl or NotOnCyl
                public bool DriveFault;     // <020> aka DskReady or NotUnitReady
                public bool SeekError;      // <010> aka DskSeekErr or NotTrk0orNotSker

                public int Current
                {
                    get
                    {
                        return (DriveType << 9) |
                            (Index ? (int)HWFlags.Index : 0) |
                            (UnitReady ? 0 : (int)HWFlags.Ready) |     // active L
                            (OnCylinder ? 0 : (int)HWFlags.OnCyl) |    // active L
                            (DriveFault ? 0 : (int)HWFlags.Fault) |    // active L
                            (SeekError ? 0 : (int)HWFlags.SeekError);  // active L
                    }
                }

                public override string ToString()
                {
                    return $"HWStatus: Idx={Index} Rdy={UnitReady} OnCyl={OnCylinder} Fault={DriveFault} SeekErr={SeekError}";
                }
            }

            // Debugging
            public void DumpDIBStatus()
            {
                Console.WriteLine("Micropolis DIB status:");
                Console.WriteLine($"  {_status}");
                Console.WriteLine($"  Command byte: {_command}");
                Console.WriteLine($"  Current Cyl:  {_cylinder}  Head: {_head}  BusEn: {BusEnable}");
                Console.WriteLine($"  Latched byte: 0x{_latchedData:x2}  Latched BusEn: {_latchedBusEnable}");
                Console.WriteLine();
                Console.WriteLine("Drive mechanical status:");
                Console.WriteLine($"  Index: {_disk.Index}  Ready: {_disk.Ready}  Trk0: {_disk.Track0}  Fault: {_disk.Fault}");
                Console.WriteLine($"  Current Cyl:  {_disk.CurCylinder}  Head: {_disk.CurHead}  Seek complete: {_disk.SeekComplete}");
            }

            /// <summary>
            /// Micropolis drive/controller register select (combinations of BA1/BA0).
            /// These are the definitions mapped through to the M1203 (raw drive)!
            /// </summary>
            enum RegSelect
            {
                CylReg = 0x00,      //  BA0 - Cylinder<7:0>
                HdCylReg = 0x20,    //  BA1 - Cylinder<11:8> + Head<3:0>
                CtrlReg = 0x10,     //  BA2 - Control reg (bits below)
                Latch = 0x30        //  BA3 - Nibble to byte latches
            }

            /// <summary>
            /// Control register bits (when BA = 2).  Unused bits are assumed zero.
            /// </summary>
            [Flags]
            enum M1200Command
            {
                None = 0x0,
                WriteEnable = 0x1,
                Unused2 = 0x2,
                TrackOffsetPlus = 0x4,
                TrackOffsetMinus = 0x8,
                FaultReset = 0x10,
                Unused20 = 0x20,
                Restore = 0x40,
                PreampHighGain = 0x80
            }

            // Local registers
            byte _driveSelect;
            ushort _cylinder;
            byte _head;

            // Nibble in progress
            RegSelect _regSelect;
            byte _latchedNibble;
            byte _latchedData;
            bool _busEnable;
            bool _latchedBusEnable;

            MicropolisDiskController _control;
            M1200Command _command;
            HWStatus _status;

            // Officially, only one drive supported :-(
            HardDisk _disk;
        }
    }
}
