//
// KeyboardMap.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using SDL2;

using System;
using System.Collections.Generic;

using PERQemu.Config;

namespace PERQemu.UI
{
    /// <summary>
    /// Name the PERQ1 and PERQ2 keyboard keys.
    /// </summary>
    public enum KeyCap
    {
        // Key not present
        None,

        // Alphanumerics
        A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
        Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,

        // Punctuation and symbols
        Minus, Equals, Quote, BackQuote, LeftBracket, RightBracket, Slash, BackSlash,
        Comma, Period, SemiColon, BackSpace, Tab, Ins, Del, Return, Space,

        // Special keys common to both
        Help, Oops, LineFeed,

        // Special keys exclusive to the PERQ-2
        AccEsc, RejDel, Setup, Break, NoScroll, PF1, PF2, PF3, PF4, Left, Right, Up, Down,

        // PERQ-2 keypad
        Pad0, Pad1, Pad2, Pad3, Pad4, Pad5, Pad6, Pad7, Pad8, Pad9,
        PadComma, PadPeriod, PadMinus, Enter,

        // Some pseudo keys for convenience (modern keypads generally include these)
        PadPlus, PadMultiply, PadDivide, PadEquals
    }

    /// <summary>
    /// Maps the SDL keyboard codes to the PERQ ASCII representation.
    /// </summary>
    public sealed class KeyboardMap
    {
        public KeyboardMap(ChassisType machine)
        {
            _map = new Dictionary<SDL.SDL_Keycode, KeyCap>();

            if (machine == ChassisType.PERQ1)
            {
                _keyCodes = KeyboardCodes.PERQ1Codes;
                SetupPERQ1Map();
            }
            else
            {
                _keyCodes = KeyboardCodes.PERQ2Codes;
                SetupPERQ2Map();
            }

            // Set the initial state of the CAPS LOCK and NUM LOCK keys,
            // based on the Console's setting?
            _lockCaps = Console.CapsLock;
            _lockNums = Console.NumberLock;
        }

        public bool CapsLock => _lockCaps;
        public bool NumLock => _lockNums;

        // Toggle the state of a given "lock" key
        public void SetLockKeyState(SDL.SDL_Keycode keycode)
        {
            switch (keycode)
            {
                case SDL.SDL_Keycode.SDLK_CAPSLOCK:
                    _lockCaps = !_lockCaps;
                    Log.Debug(Category.Keyboard, "CAPS LOCK is {0}", _lockCaps);
                    break;

                case SDL.SDL_Keycode.SDLK_NUMLOCKCLEAR:
                    _lockNums = !_lockNums;
                    Log.Debug(Category.Keyboard, "NUM LOCK is {0}", _lockNums);
                    break;
            }
        }

        /// <summary>
        /// Establish a key mapping, overriding any existing assignment.
        /// </summary>
        public void SetKeyMapping(SDL.SDL_Keycode hostKey, KeyCap perqKey)
        {
            _map[hostKey] = perqKey;
        }

        /// <summary>
        /// Return a current key mapping, or None if not assigned.
        /// </summary>
        public KeyCap GetKeyMapping(SDL.SDL_Keycode hostKey)
        {
            return _map.ContainsKey(hostKey) ? _map[hostKey] : KeyCap.None;
        }

        /// <summary>
        /// Map an SDL keycode to a PERQ key and return the appropriate "raw" encoding,
        /// or zero if the SDL key is not mapped (or the keyboard in use doesn't have it).
        /// </summary>
        public byte GetKeyValue(SDL.SDL_Keycode hostKey, bool shift, bool control)
        {
            // If not assigned, quietly ignore it
            if (!_map.ContainsKey(hostKey)) return 0;

            // Map it and get the raw code to send
            var mappedKey = _map[hostKey];
            var perqKey = _keyCodes[(int)mappedKey];

            Log.Info(Category.Keyboard, "Mapped {0} --> {1} [{2}]",
                                        hostKey.ToString(), mappedKey, perqKey.Name);

            // Account for the state of CAPS LOCK.  On a real PERQ 1, CAPS LOCK is
            // _really_ a SHIFT LOCK -- applies to every key, not just letters!  On
            // the PERQ 2, and here, we'll do the more expected CAPS LOCK behavior
            shift |= (_lockCaps && (mappedKey >= KeyCap.A && mappedKey <= KeyCap.Z));

            // I may be a little bit deranged but this makes me smile
            return (shift ? (control ? perqKey.CtrlShift : perqKey.Shift) :
                            (control ? perqKey.Control : perqKey.Normal));
        }

        /// <summary>
        /// Set up default map for the original PERQ-1 keyboard.
        /// </summary>
        void SetupPERQ1Map()
        {
            // Alphabetic
            SetKeyMapping(SDL.SDL_Keycode.SDLK_a, KeyCap.A);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_b, KeyCap.B);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_c, KeyCap.C);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_d, KeyCap.D);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_e, KeyCap.E);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_f, KeyCap.F);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_g, KeyCap.G);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_h, KeyCap.H);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_i, KeyCap.I);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_j, KeyCap.J);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_k, KeyCap.K);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_l, KeyCap.L);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_m, KeyCap.M);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_n, KeyCap.N);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_o, KeyCap.O);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_p, KeyCap.P);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_q, KeyCap.Q);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_r, KeyCap.R);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_s, KeyCap.S);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_t, KeyCap.T);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_u, KeyCap.U);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_v, KeyCap.V);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_w, KeyCap.W);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_x, KeyCap.X);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_y, KeyCap.Y);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_z, KeyCap.Z);

            // Numeric
            SetKeyMapping(SDL.SDL_Keycode.SDLK_1, KeyCap.Num1);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_2, KeyCap.Num2);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_3, KeyCap.Num3);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_4, KeyCap.Num4);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_5, KeyCap.Num5);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_6, KeyCap.Num6);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_7, KeyCap.Num7);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_8, KeyCap.Num8);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_9, KeyCap.Num9);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_0, KeyCap.Num0);

            // Punctuation
            SetKeyMapping(SDL.SDL_Keycode.SDLK_MINUS, KeyCap.Minus);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_EQUALS, KeyCap.Equals);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKQUOTE, KeyCap.BackQuote);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_LEFTBRACKET, KeyCap.LeftBracket);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RIGHTBRACKET, KeyCap.RightBracket);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSLASH, KeyCap.BackSlash);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SEMICOLON, KeyCap.SemiColon);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_QUOTE, KeyCap.Quote);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_COMMA, KeyCap.Comma);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PERIOD, KeyCap.Period);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SLASH, KeyCap.Slash);

            // Editing keys
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SPACE, KeyCap.Space);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSPACE, KeyCap.BackSpace);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_TAB, KeyCap.Tab);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_ESCAPE, KeyCap.Ins);     // Labeled INS on PERQ-1
            SetKeyMapping(SDL.SDL_Keycode.SDLK_INSERT, KeyCap.Ins);     // Same as ESC on PERQ-1
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DELETE, KeyCap.Del);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RETURN, KeyCap.Return);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DOWN, KeyCap.LineFeed);

            // Function keys

            // Note: some of these assignments are ineffective, as most standard
            // USB PC-101 keyboards don't have HELP or CLEAR; even on extended
            // Apple keyboards that do, SDL seems to ignore them?  Hmmm.

            SetKeyMapping(SDL.SDL_Keycode.SDLK_F5, KeyCap.Help);        // HELP -- was F1
            SetKeyMapping(SDL.SDL_Keycode.SDLK_HELP, KeyCap.Help);      // HELP
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F6, KeyCap.Oops);        // OOPS -- was F2
            SetKeyMapping(SDL.SDL_Keycode.SDLK_CLEAR, KeyCap.Oops);     // OOPS
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F7, KeyCap.LineFeed);    // LF   -- was F3

            Log.Debug(Category.Keyboard, "PERQ-1 map initialized");
        }

        /// <summary>
        /// Map the PERQ 2 "VT100-style" keyboard.
        /// </summary>
        void SetupPERQ2Map()
        {
            // Alphabetic
            SetKeyMapping(SDL.SDL_Keycode.SDLK_a, KeyCap.A);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_b, KeyCap.B);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_c, KeyCap.C);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_d, KeyCap.D);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_e, KeyCap.E);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_f, KeyCap.F);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_g, KeyCap.G);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_h, KeyCap.H);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_i, KeyCap.I);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_j, KeyCap.J);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_k, KeyCap.K);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_l, KeyCap.L);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_m, KeyCap.M);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_n, KeyCap.N);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_o, KeyCap.O);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_p, KeyCap.P);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_q, KeyCap.Q);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_r, KeyCap.R);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_s, KeyCap.S);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_t, KeyCap.T);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_u, KeyCap.U);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_v, KeyCap.V);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_w, KeyCap.W);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_x, KeyCap.X);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_y, KeyCap.Y);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_z, KeyCap.Z);

            // Numeric
            SetKeyMapping(SDL.SDL_Keycode.SDLK_1, KeyCap.Num1);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_2, KeyCap.Num2);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_3, KeyCap.Num3);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_4, KeyCap.Num4);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_5, KeyCap.Num5);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_6, KeyCap.Num6);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_7, KeyCap.Num7);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_8, KeyCap.Num8);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_9, KeyCap.Num9);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_0, KeyCap.Num0);

            // Punctuation
            SetKeyMapping(SDL.SDL_Keycode.SDLK_MINUS, KeyCap.Minus);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_EQUALS, KeyCap.Equals);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKQUOTE, KeyCap.BackQuote);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_LEFTBRACKET, KeyCap.LeftBracket);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RIGHTBRACKET, KeyCap.RightBracket);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSLASH, KeyCap.BackSlash);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SEMICOLON, KeyCap.SemiColon);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_QUOTE, KeyCap.Quote);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_COMMA, KeyCap.Comma);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PERIOD, KeyCap.Period);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SLASH, KeyCap.Slash);

            // Editing keys
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SPACE, KeyCap.Space);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_BACKSPACE, KeyCap.BackSpace);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_TAB, KeyCap.Tab);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_ESCAPE, KeyCap.AccEsc);      // Same as INS on
            SetKeyMapping(SDL.SDL_Keycode.SDLK_INSERT, KeyCap.AccEsc);      // PERQ-1 (sends ESC)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DELETE, KeyCap.RejDel);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RETURN, KeyCap.Return);

            // Arrows
            SetKeyMapping(SDL.SDL_Keycode.SDLK_UP, KeyCap.Up);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_DOWN, KeyCap.Down);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_LEFT, KeyCap.Left);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_RIGHT, KeyCap.Right);

            // PERQ/VT100 keys (see note above)
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F5, KeyCap.Help);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_HELP, KeyCap.Help);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F6, KeyCap.Oops);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_CLEAR, KeyCap.Oops);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_CLEAR, KeyCap.Oops);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SYSREQ, KeyCap.Break);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_SCROLLLOCK, KeyCap.NoScroll);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F7, KeyCap.LineFeed);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PAGEDOWN, KeyCap.LineFeed);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_PAGEUP, KeyCap.Setup);

            // Keypad
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F1, KeyCap.PF1);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F2, KeyCap.PF2);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F3, KeyCap.PF3);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_F4, KeyCap.PF4);

            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_PERIOD, KeyCap.PadPeriod);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_MINUS, KeyCap.PadMinus);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_COMMA, KeyCap.PadComma);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_ENTER, KeyCap.Enter);

            // Not present on PERQ, map to expected key on modern keyboards:
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_PLUS, KeyCap.PadPlus);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_EQUALS, KeyCap.PadEquals);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_DIVIDE, KeyCap.PadDivide);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_MULTIPLY, KeyCap.PadMultiply);

            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_0, KeyCap.Pad0);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_1, KeyCap.Pad1);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_2, KeyCap.Pad2);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_3, KeyCap.Pad3);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_4, KeyCap.Pad4);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_5, KeyCap.Pad5);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_6, KeyCap.Pad6);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_7, KeyCap.Pad7);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_8, KeyCap.Pad8);
            SetKeyMapping(SDL.SDL_Keycode.SDLK_KP_9, KeyCap.Pad9);

            Log.Debug(Category.Keyboard, "PERQ-2 (VT100) map initialized");
        }

        /// <summary>
        /// Show the current keyboard mappings.  Quick and dirty, for now.
        /// </summary>
        public void PrintMap()
        {
            var list = new List<string>();

            Console.WriteLine("Host key to PERQ key map:");
            foreach (var k in _map.Keys)
            {
                list.Add($"{k.ToString()} => {_map[k]}");
            }

            PERQemu.CLI.Columnify(list.ToArray(), 4, 36);

        /*
            To confirm the raw data:
                
            Console.WriteLine("Raw encoding:");
            foreach (var pk in _keyCodes)
            {
                Console.WriteLine($"[{pk.Name}: 0x{pk.Normal:x2} 0x{pk.Shift:x2} 0x{pk.Control:x2} 0x{pk.CtrlShift:x2}]");
            }
        */
        }

        // Map from SDL2 codes to a PERQ key
        Dictionary<SDL.SDL_Keycode, KeyCap> _map;

        // Which hardware map to use
        PERQKey[] _keyCodes;

        // Have to catch/test for CAPS LOCK status ourselves, and maintain local state.  Ugh.
        bool _lockCaps;
        bool _lockNums;
    }
}

