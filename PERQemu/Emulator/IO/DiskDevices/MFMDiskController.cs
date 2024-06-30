//
// MFMDiskController.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
    /// Represents the 5.25" hard drive controller on the EIO board which manages
    /// disk drives in the Disk5Inch class.  This is implemented in the PERQ as an
    /// adapter from the SA4000 interface to a variety of MFM/ST-506 style drives.
    /// This version emulates both types of DIB (one which supports write pre-
    /// compensation, and the other that doesn't).  See Docs/HardDisks.txt for more.
    /// </summary>
    public sealed class MFMDiskController : IStorageController
    {
        public MFMDiskController(PERQSystem system)
        {
            _system = system;

            _dib = new DiskInterfaceBoard(this);
            _registerFile = new byte[16];
        }

        public void Reset()
        {
            _dib.Reset();
            ResetFlags();

            Log.Info(Category.HardDisk, "MFM controller reset");
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

            _flags = SMControl.None;
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

            // Reading clears interrupts
            SetInterrupt(false);

            Log.Info(Category.HardDisk, "MFM status: 0x{0:x3}", status);
            return status;
        }

        public void LoadRegister(byte address, int value)
        {
            switch (address)
            {
                case 0xd0:      // LD DF CTRL L
                    //
                    // Selects the address of next data register to load.
                    //
                    _regSelect = (value & 0xf);
                    Log.Info(Category.HardDisk, "MFM register file pointer now {0}", _regSelect);
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

                    Log.Info(Category.HardDisk, "MFM register file[{0}]=0x{1:x2}",
                                                  _regSelect, (byte)~value);
                    _registerFile[_regSelect++] = (byte)~value;
                    break;

                case 0xd2:      // SMCTL
                    LoadCommandRegister(value);
                    break;

                case 0xd3:      // DSKCTL
                    _dib.WriteNibble((byte)value);
                    break;

                default:
                    throw new UnhandledIORequestException(address, value);
            }
        }

        /// <summary>
        /// Load the state machine control register to set flags and/or issue
        /// a command to the state machine (bits 2:0).
        /// </summary>
        /// <remarks>
        /// For MFM these flags are slightly different than the EIO Micropolis:
        ///     bits  name      description
        ///     ----  ------    -----------
        ///     7,5   T         Testable bits (state machine conditions)
        ///     6     Format    Write sector marks on the track when set
        ///     4     IntEn     Enable interrupts when set
        ///     3     Enable    Enable controller when set
        ///     2:0   F         Function
        /// Unlike the Micropolis implementation, T and T2 (bits 5 & 7) are not
        /// explicitly defined; need to research these in the 5.25" state machine
        /// nanocode.  Also, there's no mask to disable only the mid-sector IRQ;
        /// interrupts are just on or off.  Format is the only command bit, so
        /// there's no explicit Reset command here?
        /// </remarks>
        void LoadCommandRegister(int data)
        {
            // Unpack
            _flags = (SMControl)(data & 0xf8);
            _command = (SMCommand)(data & 0x07);

            Log.Info(Category.HardDisk, "MFM control: flags {0}, command {1}", _flags, _command);

            // Assume that disabling the Enable bit performs a Reset, and should
            // supercede (or never be asserted in conjunction with) Format.
            if (!_flags.HasFlag(SMControl.Enable))
            {
                Log.Info(Category.HardDisk, "MFM controller disabled!");

                // Assume we just bail out here?
                ResetFlags();
                return;
            }

            // check status/busy/error etc?

            if (_flags.HasFlag(SMControl.Format))
            {
                Log.Warn(Category.HardDisk, "MFM Format control bit asserted! (command {0})", _command);

                // not sure yet what to do here; maybe just set _command=Format and fall thru?
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
        /// Hook for Z80-issued single step pulse (not used by MFM).
        /// </summary>
        public void DoSingleSeek()
        {
            Log.Warn(Category.HardDisk, "MFM single seek step ignored!");
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
            if (_dib.SelectedDrive.CurHead != _dib.Head || _dib.Head != head || head != _registerFile[2])
            {
                Log.Warn(Category.HardDisk, "Wrong head selected: disk={0} DIB={1} regs={2} / {3}",
                                            _dib.SelectedDrive.CurHead, _dib.Head, head, _registerFile[2]);
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
                Console.WriteLine();
                Console.Write("   register file:  ");
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
                MidSectorFinish(delay, SMStatus.SMError);
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

            Log.Info(Category.HardDisk,
                      "MFM sector read from {0}/{1}/{2}, to memory at 0x{3:x6}",
                      cyl, head, sector, data);

            // For MFM, no separate flag to ignore mid-sector interrupts
            if (_flags.HasFlag(SMControl.InterruptsOn))
            {
                MidSectorFinish(delay, SMStatus.Idle);
            }
            else
            {
                FinishCommand(delay + BlockDelayNsec, SMStatus.Idle);
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
                MidSectorFinish(delay, SMStatus.SMError);
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

            Log.Info(Category.HardDisk,
                      "MFM sector write complete to {0}/{1}/{2}, from memory at 0x{3:x6}",
                      cyl, head, sector, data);

            // Todo: wait, do we do these on WRITES too?  Uh...
            if (_flags.HasFlag(SMControl.InterruptsOn))
            {
                MidSectorFinish(delay, SMStatus.Idle);
            }
            else
            {
                FinishCommand(delay + BlockDelayNsec, SMStatus.Idle);
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

            // Todo: MFM formatting details in adap.doc?
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
                      "MFM sector format of {0}/{1}/{2} complete, from memory at 0x{3:x6}",
                      _dib.Cylinder, _dib.Head, _sector, data);

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
            Log.Info(Category.HardDisk, "Firing mid-sector in {0}ms", delay * Conversion.NsecToMsec);

            _busyEvent = _system.Scheduler.Schedule(delay, (skewNsec, context) =>
            {
                Log.Info(Category.HardDisk, "Mid-sector interrupt firing, code={0}", exitCode);

                if (exitCode == SMStatus.Idle)
                {
                    // Per adap.doc, mid-sector int sets SMInt, low 3 bits to 1
                    _status = (SMStatus.SMInt | SMStatus.Busy);

                    // Fire mid-sector "DB" interrupt
                    SetInterrupt(true);

                    // Schedule a normal completion
                    FinishCommand(BlockDelayNsec, exitCode);
                }
                else
                {
                    // Save the error status
                    _status = exitCode;

                    // Fire off the interrupt
                    SetInterrupt(true);
                }
            });
        }

        /// <summary>
        /// Schedule the completion interrupt to fire.
        /// </summary>
        void FinishCommand(ulong delay, SMStatus exitCode)
        {
            _busyEvent = _system.Scheduler.Schedule(delay, (skewNsec, context) =>
            {
                Log.Info(Category.HardDisk, "Command completion: {0}", exitCode);
                _status = exitCode;
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
        void SetInterrupt(bool raiseInterrupt)
        {
            // If interrupts are disabled, force clear
            raiseInterrupt &= _flags.HasFlag(SMControl.InterruptsOn);

            if (raiseInterrupt && !_interrupting)
            {
                _system.CPU.RaiseInterrupt(InterruptSource.HardDisk);
                _interrupting = true;
            }
            else if (!raiseInterrupt && _interrupting)
            {
                _system.CPU.ClearInterrupt(InterruptSource.HardDisk);
                _interrupting = false;
            }
        }

        /// <summary>
        /// Called by the DIB when a status change requires an interrupt.
        /// </summary>
        void StatusChange()
        {
            Log.Info(Category.HardDisk, "DIB signaled status change!");
            SetInterrupt(true);
        }

        #endregion

        // Debugging
        public void DumpStatus()
        {
            // Pretty print the register names since they contain useful stuff
            // In the MFM controller, only 12 of 16 are ever used
            string[] RN = { "Zero", "Sync", "Head", "Sector", "CylLo",
                            "LH1lo", "LH2lo", "LH3lo", "Cyl/Hd",
                            "LH1hi", "LH2hi", "LH3hi" };
            string[] regs = new string[12];

            for (int i = 0; i < regs.Length; i++)
            {
                regs[i] = $"{i:d2} {RN[i]}=0x{_registerFile[i]:x2}";
            }

            Console.WriteLine("MFM driver status:");
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
        {                           // Accent eio5disk.mic:
            Idle = 0,               // DskDone
            Busy = 1,               // DskLHFound
            DataCRC = 2,            // DskDataErr       (not used by eio5disk?)
            PHMismatch = 3,         // DskPHMismatch    (PH mismatch, seek err in adap.doc)
            LHMismatch = 4,         // DskLHMismatch
            HeadCRC = 5,            // DskBadCmd        (LH CRC error in adap.doc)
            AbnormalError = 6,      // DskPHLHCRC       (not defined in adap.doc)
            SMError = 7,            // DskDataBlkCRC    (not defined in adap.doc)
            SMInt = 8               // State machine interrupt flag SMSTATUS<3>
        }

        /// <summary>
        /// State machine control register for 5.25" interface.  Bits [2:0] are
        /// the F (function) bits.  Bit 6 is the only bit that directly affects
        /// the DIB; see "adap.doc" for more info.
        /// </summary>
        [Flags]
        enum SMControl
        {
            None = 0x0,
            Enable = 0x08,          // Enable controller when H (Reset when L)
            InterruptsOn = 0x10,    // Disk Interrupt Enable when H
            T = 0x20,               // T bit: ?
            Format = 0x40,          // Write sector marks on the track when H
            T2 = 0x80               // T2 bit: ?
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
        /// Bit positions of the hardware flags from the DIB (SMStatus word).
        /// </summary>
        [Flags]
        public enum HWFlags
        {
            Track0 = 0x010,         // True when heads over track 00 (active low)
            Fault = 0x020,          // Fault line from DIB (active low)
            OnCyl = 0x040,          // Seek Complete from DIB (active low)
            Ready = 0x080,          // Ready line from DIB (active low)
            Index = 0x100           // Toggles on each revolution? Not the pulse?
        }

        // MFM sector timing based on 5Mbit/sec typical transfer rate for 528-byte
        // sectors.  See ShugartController.cs for information about computing this.
        readonly ulong BlockDelayNsec = 845 * Conversion.UsecToNsec;

        SMStatus _status;
        SMControl _flags;
        SMCommand _command;

        bool _interrupting;

        byte[] _registerFile;
        int _regSelect;

        // The physical disks and controller
        DiskInterfaceBoard _dib;

        SchedulerEvent _busyEvent;
        PERQSystem _system;


        /// <summary>
        /// The 5.25" Disk Interface Board.  Uses a six-bit bus, similar to but
        /// very distinct from the 8" DIB used in the Micropolis driver.  See
        /// Docs/HardDisk.txt for more info.
        /// </summary>
        internal class DiskInterfaceBoard
        {
            public DiskInterfaceBoard(MFMDiskController parent)
            {
                _control = parent;
                _drives = new HardDisk[2];    // Four, someday? :-)

                _status.DriveType = (int)DeviceType.Unused;
            }

            public HardDisk SelectedDrive => _drives[_selected];
            public HWStatus Status => _status;

            public int Unit => _selected;
            public byte Head => _head;
            public ushort Cylinder => _cylinder;

            public void Reset()
            {
                // Stop any seek in progress
                _control._system.Scheduler.Cancel(_seekEvent);
                _seekState = SeekState.Idle;

                foreach (var disk in _drives)
                {
                    disk?.Reset();
                }

                _selected = 0;

                Log.Info(Category.HardDisk, "MFM DIB reset");
            }

            /// <summary>
            /// Attach a drive.  Two drives can be physically connected to the
            /// DIB, though the software could theoretically support four.
            /// </summary>
            public void AttachDrive(uint unit, StorageDevice dev)
            {
                // Sanity checks (redundant)
                if (unit < 1 || unit > 2)
                    throw new InvalidOperationException($"MFMController unit {unit} out of range");

                // Map PERQSystem volumes 1..2 => units 0..1
                unit--;

                if (_drives[unit] != null)
                    throw new InvalidOperationException($"MFMController drive {unit} is already assigned");

                if (dev.Info.Type != DeviceType.Disk5Inch)
                    throw new InvalidConfigurationException($"Device type {dev.Info.Type} not supported by this controller");

                _drives[unit] = dev as HardDisk;
                _drives[unit].SetSeekCompleteCallback(SeekCompletionCallback);

                // A small detail
                _status.DriveType = (int)DeviceType.Disk5Inch;

                Log.Info(Category.HardDisk, "Attached disk '{0}'", _drives[unit].Info.Name);
            }

            /// <summary>
            /// If selected unit changed, update the internal state.
            /// </summary>
            void UnitSelect(int unit)
            {
                if (unit != _selected && _drives[unit] != null)
                {
                    _selected = unit;
                    Log.Info(Category.HardDisk, "DIB: Unit {0} selected", _selected);

                    if (_seekState != SeekState.Idle)
                        Log.Warn(Category.HardDisk, "Unit select change while seek {0}!?", _seekState);
                }

                // Update for new unit status (or error)
                UpdateStatus();
            }

            /// <summary>
            /// Initiate a seek operation when the SeekLowCount register is written.
            /// The 5.25"/MFM controller goes _back_ to the Shugart SA-4000 style of
            /// pulsing the disk step line "manually", using a 555 timer circuit on
            /// the DIB.  The currently selected unit must remain so during the seek
            /// operation.
            /// </summary>
            void StartSeek()
            {
                // Step count from the microcode is one less than the desired number!
                _seekCount++;

                if (_seekCount <= 0)
                {
                    Log.Warn(Category.HardDisk, "MFM bad seek count {0} ignored", _seekCount);
                    return;
                }

                // Sanity checks
                if (_seekState != SeekState.Idle)
                {
                    Log.Warn(Category.HardDisk, "MFM StartSeek while {0} (unit {1})", _seekState, _selected);
                    return;
                }

                _seekState = SeekState.Stepping;

                // Set our destination and check it / clip to range
                if (_seekDir > 0)
                {
                    _cylinder = (ushort)Math.Min(_drives[_selected].CurCylinder + _seekCount,
                                                 _drives[_selected].Geometry.Cylinders - 1);
                }
                else
                {
                    _cylinder = (ushort)Math.Max(_drives[_selected].CurCylinder - _seekCount, 0);
                }

                Log.Info(Category.HardDisk, "MFM unit {0} starting seek from {1} to {2} ({3} steps)",
                                            _selected, _drives[_selected].CurCylinder, _cylinder, _seekCount);

                _seekEvent = _control._system.Scheduler.Schedule(StepRate, SeekStepPulse);
            }

            /// <summary>
            /// Pulse the STEP line to the selected drive to move the heads one
            /// track based on the current direction (control register bit).
            /// </summary>
            /// <remarks>
            /// The Shugart uses the Z80 to count pulses; the MFM DIB generates a
            /// 100KHz clock to count off the step count (buffered by the drive).
            /// Per adap.doc, by design the microcode should never change the unit
            /// selection or write to the count/direction registers during a seek.
            /// </remarks>
            void SeekStepPulse(ulong skewNsec, object context)
            {
                // Do it
                _seekCount--;
                _drives[_selected].SeekStep(_seekDir);

                // Are we there yet?
                if (_seekCount > 0)
                {
                    _seekEvent = _control._system.Scheduler.Schedule(StepRate, SeekStepPulse);
                }
                else
                {
                    _seekEvent = null;
                    _seekState = SeekState.WaitingForSeekComplete;
                }

                Log.Info(Category.HardDisk, "MFM unit {0} seek count {1} ({2})",
                                            _selected, _seekCount, _seekState);
            }

            /// <summary>
            // Seek completion for the MFM controller.
            /// </summary>
            public void SeekCompletionCallback(ulong skewNsec, object context)
            {
                _seekState = SeekState.Idle;
                Log.Info(Category.HardDisk, "MFM unit {0} seek complete ({1})",
                                            _selected, _seekState);
                UpdateStatus();
            }

            /// <summary>
            /// Accept writes from the EIO over the "nibble bus".  For the 5.25"
            /// controller, the BA bits directly write from the IOB into the DIB
            /// and there are six bits of data, not four:
            ///     7:6     BA      Address of the adapter register
            ///     5:0     BUS     Data to be loaded
            /// Unlike the 8" DIB, only three registers are valid.  Writing into
            /// the SeekLowReg initiates a seek operation.
            /// </summary>
            public void WriteNibble(byte val)
            {
                var regSelect = (RegSelect)(val & 0xc0);

                switch (regSelect)
                {
                    case RegSelect.SelReg:
                        var unit = (val & 0x20) >> 5;
                        _seekDir = (val & 0x10) >> 4;
                        _rwc = (val & 0x08) >> 3;

                        // Do all the things if selected unit changed
                        UnitSelect(unit);

                        // Determine if RWC (write precomp) is really a head select bit...
                        if (_rwc > 0 && _drives[_selected].Geometry.Heads > 7)
                        {
                            _head = (byte)(val & 0x0f);
                            Log.Info(Category.HardDisk, "MFM disk control: unit {0}, dir {1}, head {2}",
                                                        _selected, _seekDir, _head);
                        }
                        else
                        {
                            _head = (byte)(val & 0x07);
                            Log.Info(Category.HardDisk, "MFM disk control: unit {0}, dir {1}, rwc {2}, head {3}",
                                                        _selected, _seekDir, _rwc, _head);
                        }

                        // ...and select the head
                        _drives[_selected].HeadSelect(_head);
                        break;

                    case RegSelect.SeekHiReg:
                        _seekCount = (val & 0x3f) << 6;
                        Log.Info(Category.HardDisk, "DIB: seek count (high): 0x{0:x}", _seekCount);
                        break;

                    case RegSelect.SeekLowReg:
                        _seekCount |= (val & 0x3f);
                        Log.Info(Category.HardDisk, "DIB: seek count (low): 0x{0:x} ({1})", (val & 0x3f), _seekCount);

                        StartSeek();
                        break;

                    case RegSelect.Illegal:
                        Log.Warn(Category.HardDisk, "DIB: write to illegal register (ignored)");
                        break;

                    default:
                        throw new InvalidOperationException($"Bad write to MFM DIB 0x{val:x2}");
                }
            }

            /// <summary>
            /// Update the status flags that come from the "hardware" and alert
            /// the controller that something has changed.  Also sets the current
            /// (local) head and cyl to the selected drive's position.  This is
            /// slightly dubious.
            /// </summary>
            public void UpdateStatus()
            {
                _head = _drives[_selected]?.CurHead ?? 0;
                _cylinder = _drives[_selected]?.CurCylinder ?? 0;

                _status.UnitReady = _drives[_selected]?.Ready ?? false;
                _status.DriveFault = _drives[_selected]?.Fault ?? true;
                _status.Index = _drives[_selected]?.Index ?? false;

                _status.OnCylinder = _status.UnitReady && (_seekState == SeekState.Idle);
                _status.Track0 = (_cylinder == 0);

                Log.Info(Category.HardDisk, "DIB status change: 0x{0:x3}", _status.Current);
                Log.Info(Category.HardDisk, "DIB {0}", _status);      // HW status string

                // Ready changes, faults, or OnCylinder asserted trigger an interrupt
                _control.StatusChange();
            }

            /// <summary>
            /// Hardware status bits passed through from the drive (or the DIB) and
            /// merged by the controller into the status word returned to the PERQ.
            /// For my sanity, the active low bits are returned as such for the
            /// microcode's interpretation but are positive logic here for ease in
            /// debugging.  Result is pre-shifted left by four bits so it can be
            /// merged directly.
            /// </summary>
            public struct HWStatus
            {
                public int DriveType;       // <300> 01=Undefined, 00=MFM / 5.25"
                public bool Index;          // <100> from drive
                public bool UnitReady;      // <080> aka DskFault or NotFault
                public bool OnCylinder;     // <040> aka DskOnCyl or NotOnCyl
                public bool DriveFault;     // <020> aka DskReady or NotUnitReady
                public bool Track0;         // <010> aka DskSeekErr or NotTrk0orNotSker

                public int Current
                {
                    get
                    {
                        return (DriveType << 9) |
                            (Index ? (int)HWFlags.Index : 0) |
                            (UnitReady ? 0 : (int)HWFlags.Ready) |      // active L
                            (OnCylinder ? 0 : (int)HWFlags.OnCyl) |     // active L
                            (DriveFault ? 0 : (int)HWFlags.Fault) |     // active L
                            (Track0 ? 0 : (int)HWFlags.Track0);         // active L
                    }
                }

                public override string ToString()
                {
                    return $"HWStatus: Idx={Index} Rdy={UnitReady} OnCyl={OnCylinder} Fault={DriveFault} Trk0={Track0}";
                }
            }

            // Debugging
            public void DumpDIBStatus()
            {
                Console.WriteLine("MFM DIB status:");
                Console.WriteLine($"  {_status}");
                Console.WriteLine($"  Unit: {_selected}  Head: {_head}  Cyl: {_cylinder}  RWC: {_rwc}");
                Console.WriteLine($"  Seek dir: {_seekDir}  Seek count: {_seekCount}  State: {_seekState}");
                Console.WriteLine();

                for (var unit = 0; unit < _drives.Length; unit++)
                {
                    if (_drives[unit] != null)
                    {
                        Console.WriteLine($"Drive {unit} mechanical status:");
                        Console.WriteLine($"  Index: {_drives[unit].Index}  Ready: {_drives[unit].Ready}  Trk0: {_drives[unit].Track0}  Fault: {_drives[unit].Fault}");
                        Console.WriteLine($"  Current Cyl:  {_drives[unit].CurCylinder}  Head: {_drives[unit].CurHead}  Seek complete: {_drives[unit].SeekComplete}");
                    }
                }
            }

            /// <summary>
            /// MFM controller register select (combinations of BA1/BA0).
            /// </summary>
            enum RegSelect
            {
                Illegal = 0x00,     //  BA0 - not used/illegal
                SeekLowReg = 0x40,  //  BA1 - seek count <5:0>
                SeekHiReg = 0x80,   //  BA2 - seek count <11:6>
                SelReg = 0xc0       //  BA3 - control bits
            }

            enum SeekState
            {
                Idle = 0,
                Stepping,
                WaitingForSeekComplete
            }

            // MFM DIB has a 100KHz step function (555 timer)
            readonly ulong StepRate = 10 * Conversion.UsecToNsec;

            int _selected;
            int _rwc;
            byte _head;
            ushort _cylinder;

            int _seekDir;
            int _seekCount;
            SeekState _seekState;
            SchedulerEvent _seekEvent;

            MFMDiskController _control;
            HWStatus _status;

            // Drives attach here!
            HardDisk[] _drives;
        }
    }
}
