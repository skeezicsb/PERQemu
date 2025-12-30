//
// CommandProcessor.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.Diagnostics;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

using PERQemu.UI;

namespace PERQemu
{
    /// <summary>
    /// CommandProcessor is the top-level console command interpreter.
    /// It wraps the CommandExecutor which does the dirty work.
    /// </summary>
    public class CommandProcessor
    {
        public CommandProcessor()
        {
            InitDDS();

            // Catch interrupts in the console
            Console.CancelKeyPress += OnCtrlC;

            // List of classes to interrogate for CommandAttributes
            var commandObjects = new List<object> {
                    this,
                    new ExecCommands(),
                    new DebugCommands(),
                    new ConfigCommands(),
                    new KeymapCommands(),
                    new StorageCommands(),
                    new SettingsCommands()
                };

            // Build the tree
            _exec = new CommandExecutor(commandObjects);

            // Start up the line editor
            _editor = new CommandPrompt(_exec.CommandTreeRoot);
        }

        public string CurrentPrefix => _editor.CurrentPrefix;

        /// <summary>
        /// Enter a subsystem.
        /// </summary>
        public void SetPrefix(string pfx, SaveNeededCallback check = null)
        {
            _exec.CurrentRoot = _editor.SetPrefix(pfx, check);
        }

        /// <summary>
        /// Return to the top-level prompt.
        /// </summary>
        public void ResetPrefix()
        {
            _exec.CurrentRoot = _editor.ResetPrefix();
        }

        /// <summary>
        /// Catch ^C on the console and return to the prompt.
        /// </summary>
        void OnCtrlC(object sender, ConsoleCancelEventArgs e)
        {
            e.Cancel = true;
            Console.WriteLine("^C");

            PERQemu.Controller.Break();
        }

        /// <summary>
        /// Read a script or configuration file, quiet-like.  Always begins
        /// execution at the command tree root, restores prefix when done.
        /// </summary>
        public void ReadScript(string script)
        {
            var curPrefix = _editor.CurrentPrefix;

            try
            {
                ResetPrefix();
                _exec.ExecuteScript(Paths.Canonicalize(script));
                SetPrefix(curPrefix);
            }
            catch
            {
                throw;
            }
            finally
            {
                // Restore our prefix
                SetPrefix(curPrefix);
            }
        }

        /// <summary>
        /// Run the main CLI loop.
        /// </summary>
        public void Run()
        {
            // This one's cheesy.  Use a null callback to tell the HRTimer to
            // return so we can check the Console for a keystroke.  Studies show
            // that the fastest typists in the world can't go much faster than
            // 50-60ms between keystrokes.  Yep.  I checked. :-)
            var consoleTimerHandle = HighResolutionTimer.Register(48d, null, "Console");
            HighResolutionTimer.Enable(consoleTimerHandle, true);

            // Sign up for controller events
            PERQemu.Controller.RunStateChanged += OnRunStateChange;

            // Here we go!
            _running = true;

            while (_running)
            {
                try
                {
                    // GetLine actually runs the Timer loop (while between
                    // keystrokes) which is brilliant and a little embarrassing
                    var cmd = _editor.GetLine().Trim();

                    if (cmd != string.Empty)
                    {
                        _exec.ExecuteLine(cmd);
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.Message);
                    Console.WriteLine(e.InnerException?.Message);
                }
            }

            // Close up shop
            PERQemu.Controller.RunStateChanged -= OnRunStateChange;
            HighResolutionTimer.Unregister(consoleTimerHandle);
        }

        /// <summary>
        /// Periodically checks the Console to see if a key is available.  If
        /// so, returns it to the editor, otherwise it runs the high-ish res
        /// timer loop.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ConsoleKeyInfo GetKeyEventually()
        {
            // We actually schedule events to run the SDL loop and check the
            // keyboard, so all we do here is what the previously-dedicated
            // HRT thread did.  Heh.
            while (true)
            {
                HighResolutionTimer.Run();

                UpdateDDS();
                if (Console.KeyAvailable) break;
            }

            return Console.ReadKey(true);
        }

        #region DDS Shenanigans

        /// <summary>
        /// Updates the DDS in the console window title bar.
        /// </summary>
        /// <remarks>
        /// Someday this will probably move to the GUI/SDL since the console
        /// window title bar updates are kind of problematic.  Updating too fast
        /// causes some terminal emulators to glitch; inserting a pause/yield on
        /// the main thread bogs down the whole emulator.  Because POS G wraps
        /// the DDS around several times during boot, we buffer the updates to
        /// at most every 50ms or so (in real time).
        /// 
        /// Update:  FLEX OS is pathological in its use of StackReset, and makes
        /// this rate limiting even more critical.  Left unchecked, it runs the
        /// counter _continuously_!  We now count rollovers in the state change
        /// event and test here if the number is absurd -- and basically force
        /// it to display 888 (which is the effect you see on the hardware when
        /// FLEX is running the 7-segment display at ~5MHz).
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateDDS()
        {
            // No change?  Quick out
            if (_ddsCount == 0 && _dds == _lastDDS) return;

            // Bump the last value and update the running avg
            var last = _ddsRate[_lastCount & 0xf];
            _avgCount += (Math.Abs(_ddsCount - last) - _avgCount) / _ddsRate.Length;

            // Save the update and reset the counter
            _ddsRate[++_lastCount & 0xf] = _ddsCount;
            _ddsCount = 0;

            // Now clamp the DDS display at 888 if it's running amok.  If we're
            // averaging >65 updates over 16 x 50ms samples the display is rolling
            // over continuously.  POS G wraps several times at boot but settles
            // at 999, so we're not in a big hurry to get here, so we'll tune this
            // a bit purely for aesthetics.
            var display = (_lastCount > _ddsRate.Length && last > 0 && _avgCount > 69) ? 888 : _dds;

            // Update the display, if changed
            if (_lastDDS != display)
            {
                Console.Title = $"DDS {display:d3}";
                _lastDDS = display;
            }
        }

        /// <summary>
        /// Reset the DDS and the histogram data.  For now we do all this work to
        /// keep from bumping the Console's title line (slow) but in future we'll
        /// be painting the actual 7-segment display on a GUI form of some kind,
        /// so minimizing updates will be more important.  But still... 
        /// </summary>
        void InitDDS()
        {
            _ddsRate = new int[16];

            _dds = _lastDDS = 0;
            _ddsCount = _lastCount = _avgCount = 0;
        }

        /// <summary>
        /// Cache DDS changes (CPU StackReset calls) and check for rollover.
        /// <remarks>
        void OnDDSChange(MachineStateChangeEventArgs a)
        {
            _dds = (int)a.Args[0];
            _ddsCount += (_dds == 0 ? 1000 : 1);    // rollover?
        }

        /// <summary>
        /// Catch changes in the machine state to update the CLI.
        /// </summary>
        void OnRunStateChange(RunStateChangeEventArgs a)
        {
            var state = a.State;

            // Add or remove the DDS change hook when the machine powers up or down
            if (state == RunState.WarmingUp)
            {
                PERQemu.Sys.DDSChanged += OnDDSChange;
                Console.Title = "DDS 000";
            }
            else if (state == RunState.Reset)
            {
                InitDDS();
                _lastDDS = -1;      // Force initial update :-)
            }
            else if (state == RunState.ShuttingDown)
            {
                PERQemu.Sys.DDSChanged -= OnDDSChange;
                Console.Title = "PERQemu";
            }
        }

        #endregion

        #region CLI Utility Routines

        /// <summary>
        /// Print a nice columnar list of (reasonably short) strings.
        /// </summary>
        public void Columnify(string[] items, int leftCol = 4, int tabWidth = 15)
        {
            Console.Write(" ".PadLeft(leftCol));
            var col = leftCol;

            foreach (var i in items)
            {
                if (col + tabWidth > Console.WindowWidth)
                {
                    Console.WriteLine();
                    Console.Write(" ".PadLeft(leftCol));
                    col = leftCol;
                }

                // If you want to print long strings, pass a wider tabWidth.
                // But catch the occasional edge case and try to make it pretty.
                if (i.Length > tabWidth)
                {
                    var extra = i.Length % tabWidth;

                    Console.Write(i);
                    col += i.Length;

                    if ((col + tabWidth - extra) < Console.WindowWidth)
                    {
                        Console.Write(" ".PadRight(tabWidth - extra));
                        col += tabWidth - extra;
                    }
                }
                else
                {
                    Console.Write(i.PadRight(tabWidth));
                    col += tabWidth;
                }
            }
            Console.WriteLine();
        }

        public bool IsPrintable(char c)
        {
            return (char.IsLetterOrDigit(c) ||
                    char.IsSymbol(c) ||
                    char.IsPunctuation(c));
        }

        public void UpdateKeywordMatchHelpers(string key, string[] values)
        {
            _exec.UpdateKeywordMatchHelpers(key, values);
        }

        #endregion

        #region Basic Commands

        //
        // Basic built-in commands
        //

        [Command("about", "About PERQemu")]
        void About()
        {
            PERQemu.PrintBanner();
        }

        [Command("commands", "Show console commands and their descriptions")]
        public void ShowCommands()
        {
            _exec.ShowCommands("root");
        }

        public void ShowCommands(string prefix)
        {
            _exec.ShowCommands(prefix);
        }

        [Command("history", "Show command history")]
        public void ShowHistory()
        {
            _editor.ShowHistory();
            // probably should just expose the list (readonly) and loop it here?
        }

        [Command("save history", "Save command history")]
        public void SaveHistory()
        {
            Console.WriteLine("[Not yet implemented]");

            // build path OutputDir/cmdhistory.txt
            // open output stream and dump it
        }

        [Command("gui", "Start the graphical interface")]
        void LaunchGUI()
        {
            // Sigh.  Maybe by v2.0?
            Console.WriteLine("Nope.  No cross-platform GUI available yet.");
        }

        [Command("done", Discreet = true)]
        void Done()
        {
            Console.WriteLine("Already at top-level.");
        }

        [Command("exit")]
        [Command("quit", "Quit PERQemu")]
        void Quit()
        {
            PERQemu.Controller.PowerOff();
            _running = false;
        }

        [Command("quit without save", "Quit PERQemu without committing changes")]
        void QuitNow()
        {
            PERQemu.Controller.PowerOff(false);     // Stop the machine, if running
            Settings.Changed = false;               // Force the "without save" part
            _running = false;
        }

        #endregion

        #region CLI Help (for now)

        // Todo:  a nice help system, either built-in (for offline operation) or
        // web-based.  For now, dump out some basic help (in 80 columns).  If this
        // starts to get large, split off into a HelpCommands class and add to the
        // list above.

        [Command("help", "Show PERQemu help")]
        void Help()
        {
            Console.WriteLine("This is PERQemu, an emulator for the Three Rivers PERQ workstation.\n" +
                              "Type 'commands' at the prompt to see which commands are available.\n");

            Console.WriteLine("The command line editor provides tab completion and prompts to assist you.\n" +
                              "Use the arrow keys to retrieve and edit previous command lines, or press\n" +
                              "ESC to erase the current line.  Type 'help editor' for more information.\n");

            Console.WriteLine("Many more commands are available to configure and customize the emulated\n" +
                              "PERQ, debug software running on it (or the emulator itself), and to set\n" +
                              "preferences that tailor PERQemu to your environment.  Type 'help ' and\n" +
                              "press the TAB key for more on-line assistance, or consult the User's Guide\n" +
                              "included with the distribution.");
        }

        [Command("help editor", "Show command line editor help")]
        void HelpCLI()
        {
            Console.WriteLine("Some commands accept optional arguments, while some require them; press the\n" +
                              "TAB key at any point to see a list of possible completions for the current\n" +
                              "input word, or preview the next expected argument.  Pressing the SPACE BAR\n" +
                              "or TAB key will expand the current input up to the longest unambiguous match.\n");

            Console.WriteLine("String arguments must be surrounded by quotes if they contain spaces.  Numeric\n" +
                              "arguments may be specified in decimal (default), or in another common base\n" +
                              "depending on preference or context:\n" +
                              "\tBase    \t Prefix    \tExample\n" +
                              "\tBinary: \t    b      \tb10001100\n" +
                              "\tOctal:  \t  o or %   \to377 or %177600\n" +
                              "\tDecimal:\td (or none)\td12345 or 54321\n" +
                              "\tHex:    \t0x, x or $ \t0xff, x3eff, $80000\n");

            //Console.WriteLine("The default output radix may be set with 'settings radix <base>'.\n" +
            //                  "[Setting output radix not yet implemented]");
        }

        [Command("help configure", "Show configuration help")]
        void HelpConfig()
        {
            Console.WriteLine("The Configurator lets you load, modify and save PERQ configurations.  Several\n" +
                              "predefined system types are provided.  The default configuration is a typical\n" +
                              "PERQ-1A similar to the machine emulated by earlier versions of PERQemu; type\n" +
                              "'configure show' to see the current selection.\n");

            Console.WriteLine("Type 'configure' by itself to enter the interactive configuration subsystem.\n" +
                              "Tab completion will guide the configuration process by prompting you for any\n" +
                              "expected parameters.  The Configurator will limit or warn you of invalid\n" +
                              "selections that aren't supported by the emulator.  Type 'configure commands'\n" +
                              "for a list of available options.  The prompt will change to 'configure*>' if\n" +
                              "you have made changes that haven't been saved.  Type 'done' when finished.");
        }

        [Command("help debugger", "Show debugger help")]
        void HelpDebug()
        {
            Console.WriteLine("PERQemu provides extensive for debugging the emulator itself or code running\n" +
                              "on the virtual machine.  In 'release builds' a limited set of commands to\n" +
                              "read debugging information is available; in 'debug builds' a more extensive\n" +
                              "command set allows manipulation of internal state and deeper interrogation\n" +
                              "of the emulation environment.  [This is still under active development and\n" +
                              "will be fully documented at a future date.]");
        }

        [Command("help logging", "Show logging help")]
        void HelpLogging()
        {
            Console.WriteLine("PERQemu provides voluminous debugging output and a set of commands to choose\n" +
                              "what type and how much information to display.  Logging incurs a substantial\n" +
                              "performance penalty, so 'release builds' offer only a limited subset of the\n" +
                              "available output.  Logging commands are available in the 'debug' subsystem.");
        }

        #endregion


        [Conditional("DEBUG")]
        [Command("debug dump command tree")]
        public void DumpCommandTree()
        {
            Console.WriteLine("\nCommand tree:");
            _exec.DumpCommandTree(_exec.CurrentRoot, 0);
        }


        // FLEX has made the DDS handilng kind of ridiculous
        int _dds;
        int _lastDDS;

        int[] _ddsRate;
        int _ddsCount;
        int _lastCount;
        int _avgCount;

        bool _running;

        CommandExecutor _exec;
        CommandPrompt _editor;
    }
}
