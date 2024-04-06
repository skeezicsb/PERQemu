//
// Z80DebugCommands.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
using System.Text;
using System.Diagnostics;

namespace PERQemu
{
    public partial class DebugCommands
    {
        //
        // Z80 Debugging
        //

        [Command("debug z80", "Enter the Z80 debugging subsystem", Prefix = true)]
        void SetZ80DebugPrefix()
        {
            PERQemu.CLI.SetPrefix("debug z80");
        }

        [Command("debug z80 done", "Return to PERQ debugger")]
        public void Z80DebugDone()
        {
            PERQemu.CLI.SetPrefix("debug");
        }

        [Command("debug z80 commands", "Show Z80 debugging commands")]
        public void ShowZ80DebugCommands()
        {
            PERQemu.CLI.ShowCommands("debug z80");
        }

        [Command("debug z80 inst", "Run one Z80 opcode")]
        public void DebugZ80Inst()
        {
            if (CheckSys())
            {
                PERQemu.Controller.TransitionTo(RunState.RunZ80Inst);
                PERQemu.Sys.PrintStatus();
            }
        }

        [Command("debug z80 show registers", "Display contents of the Z80 registers")]
        void ShowZ80State()
        {
            if (CheckSys()) PERQemu.Sys.IOB.Z80System.ShowZ80State();
        }

        [Command("debug z80 show memory", "Show contents of Z80 memory")]
        void ShowZ80Memory(ushort address, ushort bytes = 64)
        {
            if (!CheckSys()) return;

            // Since the (sparse) Z80 memory maps are different for each IO board,
            // just take whatever address they give us and catch the exception if
            // the Z80 says nuh uh
            try
            {
                if (bytes == 1)     // previous behavior
                {
                    byte value = PERQemu.Sys.IOB.Z80System.Memory[address];
                    Console.WriteLine($"Address: 0x{address:x4}  Value: 0x{value:x2}");
                    return;
                }

                // Show memory in formatted blocks of 16 bytes
                // (For now, start on even byte to more easily compare with word-
                // sized PERQ memory display.  May remove this once a particular
                // debugging scenario is satisfied since it's weirdly specific)
                var start = (address & 0xfffe);
                var end = Math.Min(0xffff, start + bytes);

                var line = new StringBuilder();
                var chars = new StringBuilder();

                for (var i = start; i < end; i += 16)
                {
                    line.Clear();
                    line.AppendFormat("{0:x4}: ", i);
                    chars.Clear();

                    // Bytes in hex (Todo: add output radix support)
                    for (var j = i; j < i + 16; j++)
                    {
                        var b = PERQemu.Sys.IOB.Z80System.Memory[j];
                        line.AppendFormat("{0:x2} ", b);

                        // ASCII
                        chars.Append(PERQemu.CLI.IsPrintable((char)b) ? (char)b : '.');
                    }

                    Console.WriteLine($"{line} {chars}");
                }
            }
            catch (Exception e)
            {
                Console.WriteLine($"Couldn't read {address}: {e.Message}");
            }
        }

        //[Conditional("DEBUG")]
        [Command("debug dump fifos")]
        [Command("debug z80 dump fifos")]
        void DumpFifos()
        {
            PERQemu.Sys.IOB.Z80System.DumpFifos();
        }

        //[Conditional("DEBUG")]
        [Command("debug z80 dump interrupts")]
        void DumpIRQs()
        {
            if (CheckSys()) PERQemu.Sys.IOB.Z80System.DumpIRQStatus();
        }

        //[Conditional("DEBUG")]
        [Command("debug z80 dump dma registers")]
        void DumpDMA()
        {
            if (CheckSys()) PERQemu.Sys.IOB.Z80System.DumpDMAStatus();
        }

        //[Conditional("DEBUG")]
        [Command("debug z80 dump scheduler queue")]
        void DumpZ80Scheduler()
        {
            if (CheckSys()) PERQemu.Sys.IOB.Z80System.Scheduler.DumpEvents("Z80");
        }

        [Command("debug z80 dump rtc")]
        void DumpRTC()
        {
            var rtc = new IO.Z80.Oki5832RTC(0xaa);  // fake address for testing

            // This is super basic for now; just verify the registers are valid
            Console.WriteLine(rtc);
        }

        // todo: ram & rom disassembler, like the perq microcode disassembler?
        // todo: i/o port reads - and writes!?
        // todo: interrogate memory, fifos, peripheral controllers & registers, etc.
    }
}
