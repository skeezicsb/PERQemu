//
// Z80DebugCommands.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.Collections.Generic;

using PERQemu.IO.Z80;
using PERQemu.Debugger;

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

        #region Breakpoints
#if DEBUG
        //
        // Breakpoints
        //

        [Command("debug z80 show breakpoints", "Show the status of all Z80 breakpoints")]
        public void ShowZ80Breakpoints()
        {
            ShowZ80Breakpoints(BreakpointType.All);
        }

        [Command("debug z80 show breakpoints", "Show the status of Z80 breakpoints")]
        public void ShowZ80Breakpoints(BreakpointType type)
        {
            if (!CheckSys()) return;

            if (type == BreakpointType.None)
            {
                Console.WriteLine("Don't be silly.");
                return;
            }

            ShowBPInternal(GetZ80Breakpoints(type));
        }

        [Command("debug z80 set breakpoint", "Set a Z80 breakpoint")]
        public void SetZ80Breakpoint(BreakpointType type, int watch, bool pause = false)
        {
            if (!CheckSys()) return;

            if (GetZ80Breakpoints(type).Count != 1)
            {
                Console.WriteLine($"Can't set Z80 breakpoint of type {type}.");
                return;
            }

            _bpList = GetZ80Breakpoints(type)[0];
            SetBPInternal(type, watch, pause);
        }

        [Command("debug z80 reset breakpoints", "Reset Z80 breakpoint counters")]
        public void ResetZ80Breakpoints(BreakpointType type)
        {
            if (!CheckSys()) return;

            if (GetZ80Breakpoints(type).Count > 0)
            {
                foreach (var list in GetZ80Breakpoints(type))
                {
                    list.ResetCounts();
                }
                Console.WriteLine($"Z80 {type} breakpoint counters reset.");
            }
        }

        [Command("debug z80 clear breakpoint", "Clear a Z80 breakpoint")]
        public void ClearZ80Breakpoint(BreakpointType type, int watch)
        {
            if (!CheckSys()) return;

            if (GetZ80Breakpoints(type).Count != 1)
            {
                Console.WriteLine($"Can't clear a Z80 breakpoint of type {type}.");
                return;
            }

            _bpList = GetZ80Breakpoints(type)[0];
            ClearBPInternal(type, watch);
        }

        [Command("debug z80 edit breakpoint", "Change or reset a Z80 breakpoint", Prefix = true)]
        public void EditZ80Breakpoint(BreakpointType type, int watch)
        {
            if (!CheckSys()) return;

            if (GetZ80Breakpoints(type).Count != 1)
            {
                Console.WriteLine($"Can't edit Z80 breakpoints of type {type}.");
                return;
            }

            _bpList = GetZ80Breakpoints(type)[0];

            if (EditBPInternal(type, watch))
            {
                PERQemu.CLI.SetPrefix("debug z80 edit breakpoint");
            }
        }

        [Command("debug z80 edit breakpoint cancel", "Cancel and return to previous level")]
        public void Z80EditCancel()
        {
            Console.WriteLine("Canceled.");
            SetZ80DebugPrefix();
        }

        [Command("debug z80 edit breakpoint done", "Save changes and return to previous level")]
        public void Z80EditDone()
        {
            // Save/apply the current temporary action
            _bpList.Watch(_bp.Value, _bpAction);

            Console.WriteLine("Saved.");
            SetZ80DebugPrefix();
        }

        List<BreakpointList> GetZ80Breakpoints(BreakpointType bp)
        {
            var selected = new List<BreakpointList>();

            switch (bp)
            {
                case BreakpointType.Interrupt:
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedIRQs);
                    break;

                case BreakpointType.IOPort:
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedIOPorts);
                    break;

                case BreakpointType.MemoryLoc:
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedMemoryAddr);
                    break;

                case BreakpointType.uAddress:
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedInstructionAddr);
                    break;

                case BreakpointType.All:
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedIRQs);
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedIOPorts);
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedMemoryAddr);
                    selected.Add(PERQemu.Sys.IOB.Z80System.Debugger.WatchedInstructionAddr);
                    break;
            }

            return selected;
        }

#endif
        #endregion

        //[Conditional("DEBUG")]
        [Command("debug z80 poke")]
        void PokeZ80Mem(ushort addr = 0x6800, byte val = 0)
        {
            PERQemu.Sys.IOB.Z80System.Memory[addr] = val;
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
            // If the PERQ is running and has an EIO, dump the active RTC!
            if (!CheckSys()) return;

            if (PERQemu.Config.Current.IOBoard == Config.IOBoardType.EIO)
            {
                var eio = PERQemu.Sys.IOB.Z80System as EIOZ80;

                eio.RTC.DumpRTC();
                return;
            }

            Console.WriteLine("This PERQ doesn't have an RTC chip.");
        }

        // todo: ram & rom disassembler, like the perq microcode disassembler?
        // todo: i/o port reads - and writes!?
        // todo: interrogate memory, fifos, peripheral controllers & registers, etc.
    }
}
