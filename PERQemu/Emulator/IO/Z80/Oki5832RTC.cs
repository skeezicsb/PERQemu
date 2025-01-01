//
// Oki5832RTC.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Oki M5M5832 Real-time Clock chip.  Provides a battery-backed calendar/
    /// clock for the EIO board.
    /// </summary>
    /// <remarks>
    /// This implementation uses the host's clock to provide the time and date,
    /// which is typically requested once by the OS at bootup.  In one of the
    /// most baffling decisions Three Rivers made, the RTC chip is not writable
    /// without a special clip and boot floppy; the extra latch and Z80 code to
    /// set the date and time after a battery replacement (or time zone change,
    /// to account for drift, etc.) was designed but then *deliberately removed*.
    /// WHY.  Seriously.  WTF.
    /// 
    /// All surviving real PERQ-2s out there are likely either running with dead
    /// batteries or with incorrect date and time settings, as the boot floppy
    /// and special DIL clip hack required to program the RTC is likely lost to
    /// the sands of time.  I have drawn up a small pluggable daughterboard that
    /// can be socketed into existing EIO boards (both 20- and 24-bit).  This
    /// module will simulate the original EIO board design that includes the
    /// write latch so that a new programming procedure can be developed and
    /// tested before PCBs are fabricated.
    /// </remarks>
    public class Oki5832RTC : IZ80Device
    {
        public Oki5832RTC(byte baseAddress)
        {
            _baseAddress = baseAddress;
            _ports = new byte[] { _baseAddress, (byte)(_baseAddress + 1) };

            _registers = new byte[16];

            // Populate the registers with the current host time
            _startDate = DateTime.Now.ToUniversalTime();
            UpdateRegistersFromHostTime();
        }

        public string Name => "RTC";
        public byte[] Ports => _ports;

        public bool IntLineIsActive => false;
        public byte? ValueOnDataBus => null;

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        public void Reset()
        {
            _busy = false;
            _writing = false;
            _command = Control.None;
            _regSelect = 0;

            Log.Debug(Category.RTC, "Reset");
        }

        /// <summary>
        /// Reads from the RTC provide one nibble at a time.
        /// </summary>
        public byte Read(byte portAddress)
        {
            // Reads from the control register??
            if (portAddress == _baseAddress)
            {
                throw new InvalidOperationException("Read from RTC control register!?");
            }

            Log.Debug(Category.RTC, "Read value {0} from register {1}", _registers[_regSelect], _regSelect);
            return _registers[_regSelect];
        }

        /// <summary>
        /// Writes to the control port select the register index (low nibble) and
        /// control bits (high nibble).  Writes to the data port load one BCD
        /// digit into the selected register.
        /// </summary>
        /// <remarks>
        /// Each register holds a single decimal digit; two of them use some
        /// extra bits to set flags.  See the Notes below for details.
        /// </remarks>
        public void Write(byte portAddress, byte value)
        {
            //
            // Control port?
            //
            if (portAddress == _baseAddress)
            {
                _command = (Control)(value & 0xf0);
                _regSelect = (byte)(value & 0x0f);
                Log.Debug(Category.RTC, "Command is: {0}; register: {1}", _command, _regSelect);

                // Hold bit transitions start or end a read/write sequence
                if (!_busy && _command.HasFlag(Control.Hold))
                {
                    // Assuming a read, populate the registers with the current date/time
                    UpdateRegistersFromHostTime((PERQemu.Sys.Uptime + PERQemu.Sys.Scheduler.CurrentTimeNsec) - _startTime);
                    _busy = true;
                    Log.Detail(Category.RTC, "Starting a new read/write sequence");
                }

                if (_busy && !_command.HasFlag(Control.Hold))
                {
                    Log.Detail(Category.RTC, "{0} sequence complete", _writing ? "Write" : "Read");

                    // If we just wrote the time, update our base!
                    if (_writing)
                    {
                        UpdateStartTimeFromRegisters();
                        _writing = false;
                    }

                    _busy = false;
                }

                return;
            }

            //
            // Data port: the Z80 clocks the data in, then strobes the Write
            // bit on the control port.  We don't fuss with all the specifics
            // of read/write timings (which are very slow on this chip).
            //
            // Deal with a couple of special cases:
            //     Register 5 (Hour10s) bit 3 is set for 24-hour mode, clear for
            //         a 12-hour clock; the PERQ always sets this bit
            //     Register 8 (Day10s) bit 2 is set in leap years to indicate that
            //         February has 29 days
            //
            // Note that 3RCC says that the hardware ignores writes to the seconds
            // registers, and the datasheet confirms this, but the Z80 code sends
            // them anyway.  My modified module (Clock.pas) just sets the seconds
            // field to zero, even though the emulator is perfectly happy to set
            // them.  It seems that 3RCC always set the clock to GMT at the factory
            // as well, so the "/SETGMTOFFSET" switch was used to compute and store
            // the offset by running SETTIME and putting in the local time.  There
            // doesn't appear to be a way to read or display the current offset!
            //
            if (_regSelect == 5)
            {
                if ((value & 0x08) != 0)
                {
                    Log.Debug(Category.RTC, "Setting 24-hour mode");
                    value &= 0x07;      // Clear bit 3 to keep the value in range
                }
            }
            else if (_regSelect == 8)
            {
                if ((value & 0x04) != 0)
                {
                    Log.Debug(Category.RTC, "Leap year bit set");
                    value &= 0x03;      // Clear high bits
                }
            }

            // The Z80 range checks everything, but clip to the low nibble just to be sure
            _registers[_regSelect] = (byte)(value & 0x0f);
            _writing = true;
            Log.Debug(Category.RTC, "Wrote value {0} to register {1}", value, _regSelect);
        }

        /// <summary>
        /// Populate the register file with the date/time since launch, or since
        /// the chip was last programmed.
        /// </summary>
        void UpdateRegistersFromHostTime(ulong elapsed = 0)
        {
            var dt = _startDate.AddMilliseconds(elapsed * Conversion.NsecToMsec);

            _registers[0] = (byte)(dt.Second % 10);
            _registers[1] = (byte)(dt.Second / 10);
            _registers[2] = (byte)(dt.Minute % 10);
            _registers[3] = (byte)(dt.Minute / 10);
            _registers[4] = (byte)(dt.Hour % 10);
            _registers[5] = (byte)(dt.Hour / 10);
            _registers[6] = (byte)dt.DayOfWeek;
            _registers[7] = (byte)(dt.Day % 10);
            _registers[8] = (byte)(dt.Day / 10);
            _registers[9] = (byte)(dt.Month % 10);
            _registers[10] = (byte)(dt.Month / 10);
            _registers[11] = (byte)(dt.Year % 10);
            _registers[12] = (byte)((dt.Year - 1980) / 10);

            Log.Info(Category.RTC, "Current PERQ date/time: {0}", dt);
        }

        /// <summary>
        /// Sets the virtual host time from the current register settings.  This saves
        /// the current scheduler timestamp so that subsequent reads from the RTC give
        /// the elapsed time since the last time it was programmed.
        /// </summary>
        void UpdateStartTimeFromRegisters()
        {
            try
            {
                // Convert the internal registers to a date time to make sure it's legit
                var dt = new DateTime(_registers[12] * 10 + _registers[11] + 1980,  // yr
                                      _registers[10] * 10 + _registers[9],          // mon
                                      _registers[8] * 10 + _registers[7],           // day
                                      _registers[5] * 10 + _registers[4],           // hr
                                      _registers[3] * 10 + _registers[2],           // min
                                      _registers[1] * 10 + _registers[0]);          // sec

                Log.Info(Category.RTC, "PERQ set the date/time: {0}", dt);

                // Save our new baseline date/time
                _startDate = dt;
                _startTime = PERQemu.Sys.Uptime + PERQemu.Sys.Scheduler.CurrentTimeNsec;
            }
            catch
            {
                Log.Info(Category.RTC, "PERQ tried to set bad date/time");
            }
        }

        // Debugging
        public void DumpRTC()
        {
            var elapsed = PERQemu.Sys.Uptime + PERQemu.Sys.Scheduler.CurrentTimeNsec - _startTime;
            elapsed = (ulong)(elapsed * Conversion.NsecToMsec * Conversion.MsecToSec);

            Console.WriteLine($"RTC: running since {_startDate} ({elapsed} sec since last update)");
            Console.WriteLine("Registers:");
            Console.WriteLine($"  Year: {_registers[12]}{_registers[11]}\t(+1980)");
            Console.WriteLine($"  Mon:  {_registers[10]}{_registers[9]}");
            Console.WriteLine($"  Day:  {_registers[8]}{_registers[7]}");
            Console.WriteLine($"  Hour: {_registers[5]}{_registers[4]}");
            Console.WriteLine($"  Min:  {_registers[3]}{_registers[2]}");
            Console.WriteLine($"  Sec:  {_registers[1]}{_registers[0]}");
        }

        /// <summary>
        /// Significant bits in the control register.  The low four bits are
        /// the currently selected index into the register file.
        /// </summary>
        [Flags]
        enum Control
        {
            None = 0x0,
            Read = 0x10,
            Write = 0x20,
            Hold = 0x40
        }

        DateTime _startDate;
        ulong _startTime;

        byte[] _registers;
        byte _regSelect;
        Control _command;

        bool _busy;
        bool _writing;

        byte _baseAddress;
        byte[] _ports;
    }
}

/*
    Notes:

    Note that the PERQ always selects 24-hour time and ignores the Day of Week
    register (6).  This chip only tracks 2-digit years, so it's not Y2K compliant.

    Setting the date and time on the hardware requires a special DIL clip and
    perfboard that physically enables the Write pin, plus a special boot floppy
    that almost certainly doesn't exist anywhere anymore (though we did find the
    DP drawings of it!).  But the EIO Z80 code includes the write routine in the
    standard build!  This means we'll be able to use the emulator to recreate the
    date/time setting software... [I have done this, and will distribute it when
    tested more thoroughly. -- skz]

    The initial implementation always updated the chip's registers from the host
    OS on every read.  This has now been modified to initialize the chip to GMT
    based on the host's clock at power on (of the virtual machine).  Subsequent
    reads from the RTC are offset from the start time by the number of processor
    cycles actually executed, so the chip runs at emulation speed.  The next
    step is to save and restore the programmed RTC registers so that if you set
    the clock back to 1983 on an emulated machine, it will resume where it left
    off when restarted! :-)

    Taking that to the next level of absurdity, we could compute the "age" of a
    virtual PERQ based on its estimated manufacturing date (between 1980-1985ish)
    and use that to slowly age the display phosphor -- "burning in" the Accent
    Icons window title bar -- or slowly introduce random disk errors as the
    number of power on hours increases.  If you want a truly accurate emulation
    experience, it's the little details that matter!  [At this point the author
    was escorted to the asylum by some nice gentlemen in clean white coats.]
    
 */
