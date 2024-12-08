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

            Log.Debug(Category.HardDisk, "MFM controller reset");
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
            var status = _dib.ReadStatus() | (int)_status;

            // Reading clears interrupts
            SetInterrupt(false);

            // And our state machine status?
            _status = SMStatus.Idle;

            Log.Debug(Category.HardDisk, "MFM status: 0x{0:x3}", status);
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
                    Log.Debug(Category.HardDisk, "MFM register file pointer now {0}", _regSelect);
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

                    Log.Detail(Category.HardDisk, "MFM register file[{0}]=0x{1:x2}",
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

            Log.Debug(Category.HardDisk, "MFM control: flags {0}, command {1}", _flags, _command);

            // Assume that disabling the Enable bit performs a Reset, and should
            // supercede (or never be asserted in conjunction with) Format.
            if (!_flags.HasFlag(SMControl.Enable))
            {
                Log.Debug(Category.HardDisk, "MFM controller disabled!");

                // Assume we just bail out here?
                ResetFlags();
                return;
            }

            // Check for the Format bit
            if (_flags.HasFlag(SMControl.Format))
            {
                // When the Format bit is set in the control word, hardware on the
                // DIB itself writes new address marks on the current track (MRKxx
                // PALs) and the microcode just waits for the next Index pulse to 
                // know that it's complete.  Here we sanity check that the command is
                // set to Idle, and if so rewrite it -- the FormatBlock() routine 
                // checks the control bit and simulates the track format operation.
                if (_command != SMCommand.Idle)
                {
                    Log.Warn(Category.HardDisk, "MFM Format control bit asserted unexpectedly! (command {0})", _command);
                    _command = SMCommand.Idle;
                }
                else
                {
                    // Go format the track!
                    _command = SMCommand.Format;
                }
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
                    WriteBlock(_command == SMCommand.Write);
                    break;

                case SMCommand.Format:
                    _status = SMStatus.Busy;
                    FormatBlock(_flags.HasFlag(SMControl.Format));
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

            // The 5.25" SYSB code explicitly does NOT use register[2] for the head
            // select, although it's defined; checking it causes the boot to fail at
            // DDS 157 even if the OS code programs the value correctly.  Instead the
            // head is stored in reg[9] as the low 4 bits like the Micropolis does it,
            // even though that's more work to set up in the microcode.  Sigh.
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
                // todo: StringBuilder, log it properly
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
        void WriteBlock(bool writeHeader)
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

            if (writeHeader)
            {
                for (int i = 0; i < block.Header.Length; i += 2)
                {
                    int word = _system.Memory.FetchWord(header + (i >> 1));
                    block.Header[i] = (byte)(word & 0xff);
                    block.Header[i + 1] = (byte)(word >> 8);
                }
            }

            _dib.SelectedDrive.SetSector(block);

            Log.Debug(Category.HardDisk,
                      "MFM sector write complete to {0}/{1}/{2}, from memory at 0x{3:x6}",
                      cyl, head, sector, data);

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
        /// <remarks>
        /// MFM formatting is done in two revolutions:  when the Format bit in
        /// the control register is set the adapter writes 16 sector marks on the
        /// current track, numbered sequentially and spaced to accommodate the
        /// size of the PH, LH and DB sections with the appropriate preambles
        /// and sync patterns, gaps and checksums.  It's also implied that it
        /// zeroes the data too.  The microcode simply waits for the next Index
        /// mark to turn off the Format bit -- and does NOT expect any interrupts
        /// from the hard disk in that time.  
        /// 
        /// During the second revolution, the microcode issues a series of Format
        /// commands to the state machine (like the Shugart controller) where each
        /// sector's LH bytes are written and the data blocks initialized with a
        /// pattern - here we just hand that off to WriteBlock() and let the usual
        /// completion/interrupts happen.
        /// </remarks>
        void FormatBlock(bool writeMarks)
        {
            if (writeMarks)
            {
                // Tell the DIB to format the whole track!  No sector interrupts
                // generated, all blocks (header and data) zeroed
                if (_dib.WriteMarks())
                {
                    Log.Debug(Category.HardDisk,
                              "MFM unit {0} track format of cyl {1}/hd {2} complete",
                              _dib.Unit, _dib.Cylinder, _dib.Head);
                    
                    _status = SMStatus.Idle;
                }
                return;
            }

            // Regular Format command: treat it like a normal Write
            WriteBlock(true);
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
                    Log.Debug(Category.HardDisk, "Mid-sector interrupt firing");

                    // Per adap.doc, mid-sector int sets SMInt, low 3 bits to 1
                    _status = (SMStatus.SMInt | SMStatus.Busy);

                    // Fire mid-sector "DB" interrupt
                    SetInterrupt(true);

                    // Schedule a normal completion
                    FinishCommand(BlockDelayNsec, exitCode);
                });
            }
            else
            {
                // Error out through FinishCommand()
                _busyEvent = _system.Scheduler.Schedule(delay, (skewNsec, context) =>
                {
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
        /// There are five possible interrupts from the controller:
        ///     * any transition of the Ready line;
        ///     * OnCyl going low (i.e., seek complete);
        ///     * the mid-sector interrupt;
        ///     * end of the sector interrupt (completion).
        /// If the header is in error, the command aborts and the completion
        /// interrupt is skipped.  The SMInt flag is set for the mid- and end-of-
        /// sector interrupts, but not for the hardware status line transitions.
        /// All interrupts are cleared by reading the status register.
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
            Log.Debug(Category.HardDisk, "DIB signaled status change!");
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

        SMCommand _command;
        SMStatus _status;
        SMControl _flags;

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

            public int Unit => _selected;
            public byte Head => _head;
            public ushort Cylinder => _cylinder;

            public void Reset()
            {
                // Stop any seek in progress
                _control._system.Scheduler.Cancel(_seekEvent);
                _seekEvent = null;
                _seekState = SeekState.Idle;

                foreach (var disk in _drives)
                {
                    disk?.Reset();
                }

                _selected = 0;
                _latchedIndex = false;

                Log.Debug(Category.HardDisk, "MFM DIB reset");
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
                _drives[unit].SetSeekCompleteCallback(SeekComplete);

                // Only register one, even in multiple drive systems
                if (unit == 0)
                {
                    _drives[unit].SetIndexPulseCallback(IndexPulse);
                }

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
                    Log.Debug(Category.HardDisk, "DIB: Unit {0} selected", _selected);

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

                // Doc says that OnCyl remains asserted until the first step pulse is
                // issued, but if we're having overruns force the status update a bit early
                // (OnCyl does NOT trigger an interrupt when de-asserted)
                _seekState = SeekState.Stepping;
                UpdateStatus();

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

                Log.Debug(Category.HardDisk, "MFM unit {0} starting seek from {1} to {2} ({3} steps)",
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

                Log.Detail(Category.HardDisk, "MFM unit {0} seek count {1} ({2})",
                                            _selected, _seekCount, _seekState);
            }

            /// <summary>
            // Seek completion for the MFM controller.
            /// </summary>
            public void SeekComplete(ulong skewNsec, object context)
            {
                _seekState = SeekState.Idle;
                Log.Debug(Category.HardDisk, "MFM unit {0} seek complete ({1})",
                                            _selected, _seekState);
                UpdateStatus();
            }

            /// <summary>
            /// Fired on the leading edge of a drive's index pulse transition.
            /// </summary>
            /// <remarks>
            /// For 5.25" disk formatting, the EIO uses the Index pulse coming
            /// from the selected drive to toggle a flip flop once per rotation.
            /// The microcode uses this when formatting!  The DIB passes the
            /// buffered but unmodified Index bit from the currently selected
            /// drive, so I only register one callback when drive 0 is loaded.
            /// It's close enough.  99.999% of PERQemu users will likely never
            /// attempt to run DiskTest to low-level format a drive. :-)
            /// </remarks>
            public void IndexPulse(ulong last)
            {
                _latchedIndex = !_latchedIndex;
                Log.Detail(Category.HardDisk, "EIO latched Index pulse {0} last {1}ns",
                                              _latchedIndex, last);
            }

            /// <summary>
            /// Simulate formatting a track by writing new sector marks and zeroes
            /// to the LH and DB sections of the current drive/head/cylinder.
            /// </summary>
            public bool WriteMarks()
            {
                // Sanity checks
                if (SelectedDrive == null)
                {
                    Log.Error(Category.HardDisk, "MFM Format request but drive not mounted (unit {0})", _selected);
                    return false;
                }

                if (_seekState != SeekState.Idle)
                {
                    Log.Error(Category.HardDisk, "MFM Format while {0} (unit {1})", _seekState, _selected);
                    return false;
                }

                Log.Detail(Category.HardDisk, "DIB: Writing sector marks to unit {0}: cyl {1}/hd {2}",
                                            _selected, _cylinder, _head);

                // Just blast all the sectors on the track
                for (ushort sec = 0; sec < SelectedDrive.Geometry.Sectors; sec++)
                {
                    // We could just rewrite them in place but low-level formatting
                    // is an extremely rare operation and I'm being lazy
                    SelectedDrive.SetSector(new Sector(_cylinder, _head, sec,
                                                       _drives[_selected].Geometry.SectorSize,
                                                       _drives[_selected].Geometry.HeaderSize));
                }

                return true;
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
                            Log.Debug(Category.HardDisk, "MFM disk control: unit {0}, dir {1}, head {2}",
                                                        _selected, _seekDir, _head);
                        }
                        else
                        {
                            _head = (byte)(val & 0x07);
                            Log.Debug(Category.HardDisk, "MFM disk control: unit {0}, dir {1}, rwc {2}, head {3}",
                                                        _selected, _seekDir, _rwc, _head);
                        }

                        // ...and select the head
                        _drives[_selected].HeadSelect(_head);
                        break;

                    case RegSelect.SeekHiReg:
                        _seekCount = (val & 0x3f) << 6;
                        Log.Detail(Category.HardDisk, "DIB: seek count (high): 0x{0:x}", _seekCount);
                        break;

                    case RegSelect.SeekLowReg:
                        _seekCount |= (val & 0x3f);
                        Log.Detail(Category.HardDisk, "DIB: seek count (low): 0x{0:x} ({1})", (val & 0x3f), _seekCount);

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
                var oldReady = _status.UnitReady;
                var oldOnCyl = _status.OnCylinder;

                _head = SelectedDrive?.CurHead ?? 0;
                _cylinder = SelectedDrive?.CurCylinder ?? 0;

                _status.UnitReady = SelectedDrive?.Ready ?? false;
                _status.DriveFault = SelectedDrive?.Fault ?? true;
                _status.Index = SelectedDrive != null ? _latchedIndex : false;

                _status.OnCylinder = _status.UnitReady && (_seekState == SeekState.Idle);
                _status.Track0 = (_cylinder == 0);

                Log.Detail(Category.HardDisk, "DIB status change: 0x{0:x3}", _status.Current);
                Log.Debug(Category.HardDisk, "DIB {0}", _status);      // HW status string

                // Ready changes or OnCylinder asserted trigger an interrupt
                if (oldReady != _status.UnitReady || (oldOnCyl == false && _status.OnCylinder == true))
                {
                    _control.StatusChange();
                }
            }

            /// <summary>
            /// Refresh and return the status for the current selected drive.
            /// </summary>
            public int ReadStatus()
            {
                UpdateStatus();
                return _status.Current;
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
                Console.WriteLine($"  Unit: {_selected}  Cyl: {_cylinder}  Head: {_head}  Idx: {_latchedIndex}  RWC: {_rwc}");
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
            bool _latchedIndex;

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
